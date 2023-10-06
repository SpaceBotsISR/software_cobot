# Header
std_msgs/Header header

# Forces and Moments
float64[3] force
float64[3] moment

# Actuation
float64[6] actuation
float64[6] pwm
float64[6] rpm

# Attitude PID gains
float64[3] att_p_gains
float64[3] att_i_gains
float64[3] att_d_gains

# Position PID gains
float64[3] pos_p_gains
float64[3] pos_i_gains
float64[3] pos_d_gains

# Current and Desired Orientation and Position
float64[3] current_rpy
float64[3] desired_rpy
float64[3] desired_position
float64[3] desired_velocity
float64[3] desired_omega

# Error and Integral Error
float64[3] error_rpy
float64[3] err_int

# Time Step
float64 time_step

# Quaternion Error
float64[3] quat_error1