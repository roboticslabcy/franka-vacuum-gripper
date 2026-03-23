std_msgs/Header header

# Vacuum value is within setpoint area
bool in_control_range

# The part has been detached after a suction cycle
bool part_detached

# Vacuum is over H2 and not yet under H2-h2
bool part_present

# Device status: 0=GREEN, 1=YELLOW, 2=ORANGE, 3=RED
uint8 device_status
uint8 DEVICE_STATUS_GREEN=0
uint8 DEVICE_STATUS_YELLOW=1
uint8 DEVICE_STATUS_ORANGE=2
uint8 DEVICE_STATUS_RED=3

# Current vacuum gripper actual power [%]
uint16 actual_power

# Current system vacuum [mbar]
uint16 vacuum
