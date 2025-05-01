# ModelInfo.msg
string            model_name

# Physical Info
float64           moi                # moment of inertia [kg·m²]
float64           mass               # total mass [kg]
geometry_msgs/Point  center_of_mass  # CoM in robot frame (x, y, z) [m]

# Limits
float64           max_rpm            # maximum wheel RPM
float64           max_steering_angle # max rear-wheel steering [rad]

# Geometric Info
float64           wheel_base         # distance from rear steering wheel to front axle [m]
float64           wheel_distance     # track width between the two front wheels [m]
float64           wheel_rad          # diamter
float64           vehicle_length     # total length [m]
float64           vehicle_width      # total width [m]
