# A single 2D waypoint in the odometry frame.
uint32 id        # Unique, monotonically increasing identifier
float64 x        # [m]
float64 y        # [m]

# Lifecycle of this waypoint within the mission
uint8 STATUS_PENDING = 0
uint8 STATUS_ACTIVE = 1
uint8 STATUS_REACHED = 2
uint8 STATUS_SKIPPED = 3
uint8 status
