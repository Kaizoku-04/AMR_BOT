"""In-process message bus: the /swarm/* topics without DDS, for deterministic faster-than-real-time runs.

Every endpoint (mock robot, mock station, the WMS under test) subscribes callbacks per topic and publishes messages;
a published message is delivered to every other subscriber of the topic at the next `flush()` (one sim step later:
the order in which endpoints run within a step can't decide anything). A message can be lost per recipient with
probability `drop_rate` (seeded), on the topics in `drop_topics` (default all). Messages are the real open_amr_msgs
classes. DDS hands every receiver its own copy; here a message is deep-copied at publish on the topics whose publishers
keep and change their message objects (mission_generator re-offers by `t.round += 1` on the Task it published:
shared, every receiver's stored Task changed with it, 2026-11-05), and shared on the others (heartbeats, bids,
claims, transfers are built fresh by every publisher; copying 5 Hz heartbeats would cost most of the run time).

The ROS node (node.py) does the same over real DDS topics; this bus exists so that a shift of thousands of orders
runs in minutes inside a pytest, with no executor, no discovery and no wall-clock timing in the loop.
"""
import copy
import random

COPY_TOPICS = ('/swarm/tasks', '/swarm/operator', '/wms/inventory')


class Bus:
    def __init__(self, seed=1, drop_rate=0.0, drop_topics=None):
        self.subs = {}                 # topic -> [(endpoint name, callback)]
        self.queue = []                # (topic, sender, msg)
        self.rng = random.Random(seed)
        self.drop_rate = drop_rate
        self.drop_topics = set(drop_topics) if drop_topics else None
        self.sent = self.dropped = 0
        self.taps = []                 # callbacks(topic, sender, msg) that see everything sent (logs, metrics)

    def subscribe(self, topic, name, cb):
        self.subs.setdefault(topic, []).append((name, cb))

    def publish(self, topic, sender, msg):
        if topic in COPY_TOPICS or topic.startswith('/wms/'):
            msg = copy.deepcopy(msg)
        self.queue.append((topic, sender, msg))
        for tap in self.taps:
            tap(topic, sender, msg)

    def lossy(self, topic):
        return self.drop_rate > 0 and (self.drop_topics is None or topic in self.drop_topics)

    def flush(self, rounds=1):
        """Deliver what was published (messages published while delivering wait for the next flush)."""
        for _ in range(rounds):
            q, self.queue = self.queue, []
            for topic, sender, msg in q:
                for name, cb in self.subs.get(topic, ()):
                    if name == sender:
                        continue
                    self.sent += 1
                    if self.lossy(topic) and self.rng.random() < self.drop_rate:
                        self.dropped += 1
                        continue
                    cb(msg)
