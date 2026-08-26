# Operational status of the robot and of the driver supervising it.
# The status values mirror flexiv::rdk::OperationalStatus one-for-one.

# Header
std_msgs/Header header

# Robot operational status, mirrors flexiv::rdk::OperationalStatus
uint8 STATUS_UNKNOWN             = 0
uint8 STATUS_READY               = 1
uint8 STATUS_BOOTING             = 2
uint8 STATUS_ESTOP_NOT_RELEASED  = 3
uint8 STATUS_NOT_ENABLED         = 4
uint8 STATUS_RELEASING_BRAKE     = 5
uint8 STATUS_MINOR_FAULT         = 6
uint8 STATUS_CRITICAL_FAULT      = 7
uint8 STATUS_IN_REDUCED_STATE    = 8
uint8 STATUS_IN_RECOVERY_STATE   = 9
uint8 STATUS_IN_MANUAL_MODE      = 10
uint8 STATUS_IN_AUTO_MODE        = 11

uint8 operational_status

# Human-readable name of the operational status above
string operational_status_name

# State of the driver supervising the robot, independent of the ros2_control lifecycle
uint8 DRIVER_UNINITIALIZED = 0
uint8 DRIVER_READY         = 1
uint8 DRIVER_FAULT         = 2
uint8 DRIVER_RECOVERING    = 3
uint8 DRIVER_LOCKOUT       = 4
uint8 DRIVER_DISCONNECTED  = 5

uint8 driver_state

# Robot condition flags
bool connected
bool fault
bool operational
bool estop_released
bool reduced
bool recovery_state

# Current robot control mode, values defined in Mode.msg
int8 control_mode

# How the current condition should be recovered from, values defined in RecoveryPolicy.msg
uint8 recovery_policy

# Operator-facing explanation of the current condition and what to do about it
string message

# Most recent error and critical events reported by the robot
RobotEvent[] recent_events
