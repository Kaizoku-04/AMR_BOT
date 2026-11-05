# An operator's command to the fleet on /swarm/operator (reliable, transient local so agents started later get it).
# Phase 4a: RELEASE_ROBOT, ACK_FAULT. Phase 6 (2026-11-05): PAUSE / RESUME, OUT_OF_SERVICE / IN_SERVICE.
# Commands apply in the order received (transient local: a robot that starts later replays them in order).
uint8 RELEASE_ROBOT=0    # target (a robot id) is gone: stop treating its last reservations and charger as held, and
                         # count it out of the fleet (quorum). Applies to silence that began before `stamp`; if the
                         # robot is heard again after `stamp` it is back in the fleet.
uint8 ACK_FAULT=1        # target (a robot id) was looked at by a person: retry what it gave up on (docking: back to
                         # the staging node, fresh attempts)

uint8 PAUSE=2            # target: a robot id, or '' = the whole fleet. Takes no new work and stops at the next node it
                         # holds (as when isolated); keeps its task, heartbeat (status PAUSED) and reservations. An empty
                         # robot paused longer than reclaim_stuck_s (120 s) gives its task back, as when stuck.
uint8 RESUME=3           # target: a robot id, or '' = the whole fleet (clears a fleet pause and every robot pause)
uint8 OUT_OF_SERVICE=4   # target: a robot id. Takes no new work; finishes the task it has, then idles / charges /
                         # parks as usual (heartbeat out_of_service true) until IN_SERVICE
uint8 IN_SERVICE=5       # target: a robot id. Back in service
builtin_interfaces/Time stamp    # sim time
uint8 command
string target
string issued_by                 # operator who issued it (event log)
string reason
