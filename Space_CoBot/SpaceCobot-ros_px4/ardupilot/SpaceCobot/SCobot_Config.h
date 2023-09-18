// User specific config file.  Any items listed in config.h can be overridden here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no longer
// valid! You should switch to using a HAL_BOARD flag in your local config.mk.
#define GPS_USE     0
#define N_MOTORS    6
#define MICRO 10^6

///STORED IN PARAM_MAP:

#define ERLE_UID    "ERLE_UID"
#define MAX_LIN_VEL "MAX_LIN_VEL"
#define MAX_ANG_VEL "MAX_ANG_VEL"
#define MIN_LIN_VEL "MIN_LIN_VEL"
#define MIN_ANG_VEL "MIN_ANG_VEL"
#define MAX_X       "MAX_X"
#define MAX_Y       "MAX_Y"
#define MAX_Z       "MAX_Z"
#define MIN_X       "MIN_X"
#define MIN_Y       "MIN_Y"
#define MIN_Z       "MIN_Z"
#define PWM_MAX     "PWM_MAX"
#define PWM_MIN     "PWM_MIN"
#define ARMED       "ARMED"
#define KXP_z       "KXP_z"
#define KXP_y       "KXP_y"
#define KXP_x       "KXP_x"
#define KXD_z       "KXD_z"
#define KXD_y       "KXD_y"
#define KXD_x       "KXD_x"
#define Kang_z      "Kang_z"
#define Kang_x      "Kang_x"
#define Kang_y      "Kang_y"
#define Komg_y      "Komg_y"
#define Komg_x      "Komg_x"
#define Komg_z      "Komg_z"
#define Ki_x        "Ki_x"
#define Ki_y        "Ki_y"
#define Ki_z        "Ki_z"
#define WINDUP_LIMIT_UP "WINDUP_LIMIT_UP"
#define WINDUP_LIMIT_LOW "WINDUP_LIMIT_LOW"
#define ROS_DEBUG   "ROS_DEBUG"
#define CSV_DEBUG   "CSV_DEBUG"
#define MAG_DEC     "MAG_DEC"
#define COMPASS_OFS_X "COMPASS_OFS_X"
#define COMPASS_OFS_Y "COMPASS_OFS_Y"
#define COMPASS_OFS_Z "COMPASS_OFS_Z"
#define MASS        "MASS"


#define ERLE_UID_VALUE      1001 //erle user id
#define MAX_LIN_VEL_VALUE   1
#define MAX_ANG_VEL_VALUE   2*3.14
#define MIN_LIN_VEL_VALUE   -1.0
#define MIN_ANG_VEL_VALUE   -2*3.14
#define MAX_X_VALUE         1000.0
#define MAX_Y_VALUE         1000.0
#define MAX_Z_VALUE         1000.0
#define MIN_X_VALUE         -1000.0
#define MIN_Y_VALUE         -1000.0
#define MIN_Z_VALUE         -1000.0
#define PWM_MAX_VALUE       1600.0
#define PWM_MIN_VALUE       1350.0
#define ARMED_VALUE         0.0 //We start with disarmed motors
#define KXP_z_VALUE         20.00000
#define KXP_y_VALUE         20.00000
#define KXP_x_VALUE         20.00000
#define KXD_z_VALUE         22.00000
#define KXD_y_VALUE         22.00000
#define KXD_x_VALUE         22.00000
///Attitude proportional gains
#define Kang_x_VALUE        0.30
#define Kang_y_VALUE        0.27
#define Kang_z_VALUE        0.30
///Attitude derivative gains
#define Komg_x_VALUE        0.20
#define Komg_y_VALUE        0.16
#define Komg_z_VALUE        0.17
///Attitude integral gains
#define Ki_x_VALUE          0
#define Ki_y_VALUE          0
#define Ki_z_VALUE          0
///Windup
#define WINDUP_LIMIT_UP_VALUE     0.1
#define WINDUP_LIMIT_LOW_VALUE    -0.1

#define ROS_DEBUG_VALUE     1.0
#define CSV_DEBUG_VALUE     1.0
#define MAG_DEC_VALUE       -2.316667 //This is the magnetic declination in Lisbon
///Extracted from ArduCopter compass calibration:
#define COMPASS_OFS_X_VALUE -166.0
#define COMPASS_OFS_Y_VALUE 387.0
#define COMPASS_OFS_Z_VALUE 79.0
#define MASS_VALUE          2