# A single robot event, mirrors flexiv::rdk::RobotEvent

# Event severity level
uint8 LEVEL_UNKNOWN  = 0
uint8 LEVEL_INFO     = 1
uint8 LEVEL_WARNING  = 2
uint8 LEVEL_ERROR    = 3
uint8 LEVEL_CRITICAL = 4

uint8 level

# Unique ID of the event
int32 id

# Brief description of the event
string description

# Consequences caused by the event
string consequences

# Probable causes of the event
string probable_causes

# Recommended actions after the event
string recommended_actions

# Timestamp of the event
builtin_interfaces/Time timestamp
