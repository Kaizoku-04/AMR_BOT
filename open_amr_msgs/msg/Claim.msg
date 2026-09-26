# Announces the outcome of a deterministic decision (/swarm/claims): every agent evaluates the same bids with
# the same rule; the winner publishes the claim so the environment and late joiners learn it too.
uint8 TASK=0
uint8 DOCK=1                   # charging dock
uint8 STATION=2                # arm bay

uint8 CLAIM=0
uint8 RELEASE=1
uint8 DONE=2                   # task completed
uint8 CANCEL=3                 # TASK: the WMS withdraws an unclaimed task (its bin or bay is blocked by a silent robot)

builtin_interfaces/Time stamp
uint8 resource_type
string resource_id
string robot_id                # the claimant
uint8 action
uint32 round                   # TASK: the auction round the claim belongs to (a re-auctioned task has a higher one)
string by                      # who announced it when not the claimant itself (RELEASE of a silent robot's task)
