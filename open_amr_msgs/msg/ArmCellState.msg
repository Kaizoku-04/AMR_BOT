# Heartbeat of a palletizing cell's controller (open_amr_arm_cell cell_controller) on /<arm_id>/cell/state, 5 Hz.
# The cell's station_agent reads it (FAULT / STOPPED = out of service); the operations dashboard shows it.
uint8 STARTING=0     # waiting for the arm's driver, controllers and I/O
uint8 READY=1        # idle, can take a transfer
uint8 BUSY=2         # running a transfer
uint8 HOLDING=3      # idle, a pre-picked carton held over the deck zone (depalletizing)
uint8 FAULT=4        # latched fault: a person clears the cell, then calls /<arm_id>/cell/reset
uint8 STOPPED=5      # the arm's safety system stopped it (protective / safeguard / emergency stop)

builtin_interfaces/Time stamp
string arm_id
uint8 state
string phase                   # step of the transfer in progress (BUSY), see ArmTransfer feedback
string fault                   # FAULT: what happened
bool carton_on_tool            # the vacuum switch reads a part
uint8 safety_mode              # ur_dashboard_msgs/SafetyMode (1 = NORMAL)
bool interlock                 # a robot is docked and stopped at the bay (the arm may enter the deck zone)
string robot_at_bay
uint32 cycles                  # transfers completed since start
uint32 faults                  # faults since start
