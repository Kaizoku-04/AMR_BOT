# Heartbeat of one arm station (/swarm/stations, 5 Hz), published by its station_agent (environment side).
# The handshake is state, not events: the robot waiting at the bay leaves once it sees its task in served_task.
uint8 IDLE=0        # ready for the next robot
uint8 BUSY=1        # transferring a box between a pallet and the robot at the bay
uint8 FAULT=2       # out of service (arm fault, e-stop): no new work there; an empty robot at the bay gives up
uint8 STARVED=3     # receiving: no box staged (next pallet due); outbound: no free pallet slot (full pallet awaiting pickup)

builtin_interfaces/Time stamp
string station_id
string bay_node                # lane-graph node the robot stands on for a transfer
uint8 state
string claimed_by              # robot being served (BUSY)
string task_id                 # task of the transfer in progress
string served_task             # task of the last completed transfer
uint32 boxes_available         # receiving: boxes on the staged pallets; outbound: free pallet slots
int32[] pallets                # boxes on each staged pallet pose (layout order)
uint32 capacity                # boxes per full pallet
uint32 transfers               # completed transfers since start
float32 cycle_s                # nominal transfer time
