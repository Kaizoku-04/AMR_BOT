# Per-arm heartbeat (/swarm/stations) from the station side (environment).
uint8 IDLE=0
uint8 BUSY=1
uint8 FAULT=2

builtin_interfaces/Time stamp
string station_id
uint8 state
string claimed_by
uint32 boxes_available         # receiving: boxes left on the staged pallet; outbound: free pallet slots
