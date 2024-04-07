# This message defines where to move a camera to and at which speeds.  

# The target pose definition:

# The frame-relative point to move the camera to.
geometry_msgs/PointStamped eye

# The frame-relative point for the focus (or pivot for an Orbit controller).
# The camera points into the direction of the focus point at the end of the movement.
geometry_msgs/PointStamped focus

# The frame-relative vector that maps to "up" in the view plane.
# In other words, a vector pointing to the top of the camera, in case you want to perform roll movements.
geometry_msgs/Vector3Stamped up


# Defines how long the transition from the current to the target camera pose should take.
# Movements with a negative transition_duration will be ignored.
builtin_interfaces/Duration transition_duration

# The interpolation speed profile to use during this movement.
uint8 interpolation_speed
uint8 RISING    = 0 # Speed of the camera rises smoothly - resembles the first quarter of a sinus wave.
uint8 DECLINING = 1 # Speed of the camera declines smoothly - resembles the second quarter of a sinus wave.
uint8 FULL      = 2 # Camera is always at full speed - depending on transition_duration.
uint8 WAVE      = 3 # RISING and DECLINING concatenated in one movement.
