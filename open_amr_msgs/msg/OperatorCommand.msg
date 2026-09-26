# An operator's command to the fleet on /swarm/operator (reliable, transient local so agents started later get it).
# Phase 4a: RELEASE_ROBOT. Phase 6 adds pause/resume, out of service, fault acknowledge.
uint8 RELEASE_ROBOT=0    # target (a robot id) is gone: stop treating its last reservations and charger as held, and
                         # count it out of the fleet (quorum). Applies to silence that began before `stamp`; if the
                         # robot is heard again after `stamp` it is back in the fleet.

builtin_interfaces/Time stamp    # sim time
uint8 command
string target
string issued_by                 # operator who issued it (event log)
string reason
