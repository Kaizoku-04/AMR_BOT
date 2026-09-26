# A box changing hands (/swarm/transfers), published when the transfer starts, for observers: the simulator
# animates it, the WMS stand-in counts it. Arm transfers come from the station_agent, bin transfers from the robot.
uint8 PALLET_TO_ROBOT=0        # receiving arm depalletizes onto the AMR's conveyor deck
uint8 ROBOT_TO_BIN=1           # the AMR's deck conveys the box into a storage bin
uint8 BIN_TO_ROBOT=2           # ... and out of one
uint8 ROBOT_TO_PALLET=3        # outbound arm palletizes from the AMR's deck

builtin_interfaces/Time stamp
uint8 kind
string task_id
string robot_id
string station_id              # arm kinds
int32 pallet                   # arm kinds: pallet pose index at the station
int32 slot                     # arm kinds: stack slot on that pallet (layer by layer)
string bin_id                  # bin kinds: '<aisle bay node>:W' or ':E'
float32 duration_s
