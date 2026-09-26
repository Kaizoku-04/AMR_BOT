# A unit of work offered on /swarm/tasks by the environment (mission_generator = WMS stand-in).
# Never addressed to a robot: agents bid, and all agents apply the same rule to pick the winner.
uint8 FETCH_FROM_STATION=0     # leg 1 at a station (receiving arm) -> leg 2 to dest bin
uint8 FETCH_FROM_BIN=1         # leg 1 at a bin -> leg 2 to a station (outbound arm)
uint8 GOTO=2                   # plain move to origin_id (tests, repositioning)

builtin_interfaces/Time stamp
string task_id
string mission_id
uint8 type
string origin_id               # lane-graph node name (bay, bin bay, charger ...)
string dest_id                 # lane-graph node name, empty for GOTO
string bin_id                  # storage slot the box goes to / comes from ('<aisle bay node>:W' / ':E'), empty for GOTO
uint8 priority                 # higher = more urgent
uint32 round                   # re-offer counter (the WMS re-offers unclaimed tasks with round+1)
builtin_interfaces/Time deadline
