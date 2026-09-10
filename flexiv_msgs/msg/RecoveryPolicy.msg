# How a given robot condition should be recovered from

# Robot is ready, nothing to recover
uint8 NONE = 0

# Transient condition (booting, releasing brakes), resolves on its own
uint8 TRANSIENT = 1

# Recoverable without operator intervention, ErrorRecovery will handle it
uint8 AUTO_RECOVERABLE = 2

# Needs a physical or Flexiv Elements action before recovery can proceed
uint8 WAIT_OPERATOR = 3

# Safety system is engaged, recovery is refused until it is released
uint8 SAFETY_LOCKOUT = 4

# Connection with the robot is lost, the hardware component must be reconfigured
uint8 CONNECTION_LOST = 5

uint8 recovery_policy
