# Heartbeat of one swarm_agent on /swarm/state (5 Hz). Every agent derives the same decisions from these.
uint8 IDLE=0
uint8 TO_PICK=1        # driving to a task's origin (station queue or bin)
uint8 TO_DROP=2        # loaded, driving to the destination
uint8 AT_STATION=3     # at a bay / bin, waiting for a transfer
uint8 LOADED=4         # carrying a box (informational; combined with TO_DROP while driving)
uint8 CHARGING=5
uint8 YIELDING=6       # holding before a node reserved by a peer
uint8 STUCK=7
uint8 TO_CHARGE=8
uint8 ISOLATED=9       # hears less than a quorum of the fleet: stopped at its next held node until it does again

builtin_interfaces/Time stamp
string robot_id
uint32 seq                         # heartbeat counter (acknowledged reservations: a node is driven onto only once
string[] heard_ids                 #   every peer's heartbeat reports, in heard_ids/heard_seq, having received a
uint32[] heard_seq                 #   heartbeat of mine with seq >= the first one that listed the node)
uint8 status
geometry_msgs/Pose2D pose          # map frame
float32 speed
float32 battery                    # state of charge 0..1 (from the robot's BatteryState)
string task_id                     # empty when idle
uint32 task_round                  # auction round the task was won in: a re-auctioned task (higher round) beats an older claim
bool loaded                        # a box is on the deck (between pickup and drop): its task can't be re-auctioned
string mission_id
uint32 last_node                   # lane-graph node most recently reached
uint32[] route                     # remaining route (lane-graph node ids)
uint32[] reserved_nodes            # nodes this robot holds (traffic control): peers must not enter them
float32 wait_s                     # seconds spent waiting for the current reservation (ageing priority)
uint32 goal_node                   # node the current leg ends at (or where it stands when idle): chargers are
                                   # claimed through it, so two robots never head for the same one
