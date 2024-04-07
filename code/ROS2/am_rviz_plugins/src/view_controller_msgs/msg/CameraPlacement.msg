# The interpolation mode to use during this step
uint8 interpolation_mode
uint8 LINEAR    = 0 # Positions will be linearly interpolated
uint8 SPHERICAL = 1 # Position and orientation will be interpolated in a spherical sense.

# Sets this as the camera attached (fixed) frame before movement.
# An empty string will leave the attached frame unchanged. 
string target_frame

# When should this pose be reached?
# A negative value will disable the pose command altogether.
builtin_interfaces/Duration time_from_start

# The frame-relative point for the camera.
geometry_msgs/PointStamped eye

# The frame-relative point for the focus (or pivot for an Orbit controller).
geometry_msgs/PointStamped focus

# The frame-relative vector that maps to "up" in the view plane.
# The zero-vector will default to +Z in the view controller's "Target Frame".
geometry_msgs/Vector3Stamped up

# ------------------------------------------------
# Some paramters for interaction control
# ------------------------------------------------
# The interaction style that should be activated when movement is done.
uint8 mouse_interaction_mode
uint8 NO_CHANGE = 0 # Leaves the control style unchanged
uint8 ORBIT = 1 # Activates the Orbit-style controller
uint8 FPS   = 2 # Activates the FPS-style controller

# A flag to enable or disable user interaction 
# (defaults to false so that interaction is enabled)
bool interaction_disabled

# A flag indicating if the camera yaw axis is fixed to +Z of the camera attached_frame
bool allow_free_yaw_axis
