# Time stamp
builtin_interfaces/Time stamp

# Joint states
float64[] q_act
float64[] q_dot_act
float64[] q_dotdot_act

# Joint effort
float64[] tau

# Setpoints
float64[] q_set
float64[] q_dot_set
float64[] q_dotdot_set

# Setpoints for Cartesian space
geometry_msgs/Pose x_set
geometry_msgs/Twist x_dot_set
geometry_msgs/Twist x_dotdot_set

# Other setpoints
float64[] q_ns_set