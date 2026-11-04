# Snapshot of the mission, published periodically by mission_manager.
builtin_interfaces/Time stamp

# Mission state: IDLE | RUNNING | PAUSED | COMPLETED | ABORTED
string state
string message            # Human-readable note about the last transition

Waypoint[] waypoints
int32 active_index        # Index into waypoints, -1 when no waypoint is active
uint32 reached_count
float32 progress          # Fraction of waypoints reached, 0.0 .. 1.0

# Robot pose from odometry
bool pose_valid
float64 robot_x
float64 robot_y
float64 robot_yaw         # [rad]
float64 linear_speed      # [m/s]
float64 angular_speed     # [rad/s]

# Live navigation feedback for the active waypoint
float64 distance_to_target   # [m], -1 when not navigating
