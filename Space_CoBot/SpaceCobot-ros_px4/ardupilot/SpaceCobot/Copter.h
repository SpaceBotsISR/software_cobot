//
// Created by Filipe Rosa on 12/12/2017.
//
////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include <cmath>
#include <stdio.h>
#include <stdarg.h>
#include <map>
#include <string>

#include <AP_HAL/AP_HAL.h>

// Common dependencies
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Menu/AP_Menu.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>

//Aplication Dependencies - Parameters.h is forcing all these imports
// -> Possible solution is to rewrite Parameters.h to create only needed params
#include <AC_PID/AC_PID.h>             // PID library
#include <AC_PID/AC_PI_2D.h>           // PID library (2-axis)
#include <AC_PID/AC_P.h>               // P library
#include <AP_Button/AP_Button.h>
#include <AP_Stats/AP_Stats.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_Proximity/AP_Proximity.h>

// Configuration
#include "defines.h"
#include "config.h"

// Local modules
#include "Parameters.h"

#include "GCS_Mavlink.h"

#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_RangeFinder/RangeFinder.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_AHRS/AP_AHRS_NavEKF.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialNav/AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <Eigen/Dense>
#include "SCobot_PipeMsgs.h"

struct CompareStrings
{
    bool operator()(const char* lhs, const char* rhs) const {
        return std::strcmp(lhs, rhs) < 0;
    }
};

class Copter : public AP_HAL::HAL::Callbacks{

public:
    friend class GCS_MAVLINK_Copter;
    friend class Parameters;
    friend class ParametersG2;

    Copter(void);
    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

    void mavlink_delay_cb(void);

private:
    //Named Pipe Variables
    int attPipe_fd;
    int actPipe_fd;
    int debugPipe_fd;
    int motionPipe_fd;
    int posePipe_fd;
    int velPipe_fd;
    int paramPipe_fd;
    const char *attPipe_path="/home/erle/imu_fifo";
    const char *actPipe_path="/home/erle/act_fifo";
    const char *debugPipe_path="/home/erle/debug_fifo";
    const char *motionPipe_path="/home/erle/motion_fifo"; //carries current position and velocity from simulation
    const char *posePipe_path="/home/erle/pose_fifo"; //carries target attitude and position from trajectory planner
    const char *velPipe_path="/home/erle/vel_fifo";//carries targe angular and linear velocities from the trajectory planner
    const char *paramPipe_path="/home/erle/param_fifo";
    int debugMsgId;

    Eigen::Vector3d err_int;

    //File descriptor to write debug data to file
    int debugINS_fd;

    std::map<const char*,float,CompareStrings> param_map;
    ParamMsg lastROSParam;

    MotionMsg currentMotion; //Current position and velocity of the vehicle
    MotionMsg prevMotion;

    PoseMsg targetPose;
    PoseMsg prevTargetPose;

    VelocityMsg targetVel;
    VelocityMsg prevTargetVel;

    // key aircraft parameters passed to multiple libraries
    AP_Vehicle::MultiCopter aparm;

    // cliSerial isn't strictly necessary - it is an alias for hal.console. It may
    // be deprecated in favor of hal.console in later releases.
    AP_HAL::BetterStream* cliSerial;

    // Global parameters are all contained within the 'g' class.
    Parameters g;
    ParametersG2 g2;

    // main loop scheduler
    AP_Scheduler scheduler;

    // used to detect MAVLink acks from GCS to stop compassmot
    uint8_t command_ack_counter;

    // primary input control channels
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_yaw;

    // Dataflash
    DataFlash_Class DataFlash;

    AP_GPS gps;

    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;
    AP_SerialManager serial_manager;

    RangeFinder rangefinder {serial_manager, ROTATION_PITCH_270};
    struct {
        bool enabled:1;
        bool alt_healthy:1; // true if we can trust the altitude from the rangefinder
        int16_t alt_cm;     // tilt compensated altitude (in cm) from rangefinder
        uint32_t last_healthy_ms;
        LowPassFilterFloat alt_cm_filt; // altitude filter
        int8_t glitch_count;
    } rangefinder_state = { false, false, 0, 0 };

    // Inertial Navigation EKF
    NavEKF2 EKF2{&ahrs, barometer, rangefinder};
    NavEKF3 EKF3{&ahrs, barometer, rangefinder};
    AP_AHRS_NavEKF ahrs{ins, barometer, gps, rangefinder, EKF2, EKF3, AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
    //AP_AHRS_DCM ahrs{ins,barometer,gps};

    // GCS selection

    static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;

    GCS_MAVLINK_Copter gcs_chan[MAVLINK_COMM_NUM_BUFFERS];
    GCS _gcs; // avoid using this; use gcs()
    GCS &gcs() { return _gcs; }

    //USED BY MAVLINK
    // Documentation of GLobals:
    union {
        struct {
            uint8_t unused1                 : 1; // 0
            uint8_t simple_mode             : 2; // 1,2     // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
            uint8_t pre_arm_rc_check        : 1; // 3       // true if rc input pre-arm checks have been completed successfully
            uint8_t pre_arm_check           : 1; // 4       // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
            uint8_t auto_armed              : 1; // 5       // stops auto missions from beginning until throttle is raised
            uint8_t logging_started         : 1; // 6       // true if dataflash logging has started
            uint8_t land_complete           : 1; // 7       // true if we have detected a landing
            uint8_t new_radio_frame         : 1; // 8       // Set true if we have new PWM data to act on from the Radio
            uint8_t usb_connected           : 1; // 9       // true if APM is powered from USB connection
            uint8_t rc_receiver_present     : 1; // 10      // true if we have an rc receiver present (i.e. if we've ever received an update
            uint8_t compass_mot             : 1; // 11      // true if we are currently performing compassmot calibration
            uint8_t motor_test              : 1; // 12      // true if we are currently performing the motors test
            uint8_t initialised             : 1; // 13      // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
            uint8_t land_complete_maybe     : 1; // 14      // true if we may have landed (less strict version of land_complete)
            uint8_t throttle_zero           : 1; // 15      // true if the throttle stick is at zero, debounced, determines if pilot intends shut-down when not using motor interlock
            uint8_t system_time_set         : 1; // 16      // true if the system time has been set from the GPS
            uint8_t gps_base_pos_set        : 1; // 17      // true when the gps base position has been set (used for RTK gps only)
            enum HomeState home_state       : 2; // 18,19   // home status (unset, set, locked)
            uint8_t using_interlock         : 1; // 20      // aux switch motor interlock function is in use
            uint8_t motor_emergency_stop    : 1; // 21      // motor estop switch, shuts off motors when enabled
            uint8_t land_repo_active        : 1; // 22      // true if the pilot is overriding the landing position
            uint8_t motor_interlock_switch  : 1; // 23      // true if pilot is requesting motor interlock enable
            uint8_t in_arming_delay         : 1; // 24      // true while we are armed but waiting to spin motors
        };
        uint32_t value;
    } ap;

    RCMapper rcmap;

    // board specific config
    AP_BoardConfig BoardConfig;

    // sensor health for logging
    struct {
        uint8_t baro        : 1;    // true if baro is healthy
        uint8_t compass     : 1;    // true if compass is healthy
        uint8_t primary_gps;        // primary gps index
    } sensor_health;

    // Battery Sensors
    AP_BattMonitor battery;

    // Variables for extended status MAVLink messages
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // IMU variables
    // Integration time (in seconds) for the gyros (DCM algorithm)
    // Updated with the fast loop
    float G_Dt;

    // Inertial Navigation
    AP_InertialNav_NavEKF inertial_nav;

    // System Timers
    // --------------
    // Time in microseconds of main control loop
    uint32_t fast_loopTimer;
    // Counter of main loop executions.  Used for performance monitoring and failsafe processing
    uint16_t mainLoop_count;

    // use this to prevent recursion during sensor init
    bool in_mavlink_delay;

    // true if we are out of time in our event timeslice
    bool gcs_out_of_time;


    // Top-level logic
    // setup the var_info table
    AP_Param param_loader;

    // 3D Location vectors
    // Current location of the copter (altitude is relative to home)
    Location_Class current_loc;

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    void compass_accumulate(void);
    void compass_cal_update(void);
    void barometer_accumulate(void);
    void accel_cal_update(void);
    void fast_loop();
    void produceAttMessage(AttitudeMsg*, Quaternion*);
    void produceActMsg(ActuationMsg*, Eigen::VectorXd);
    int setupCommunicationPipes(void);
    void sendRosDebugMsg(const char *, const char *); //string messages only
    int setupPipeProducer(const char* pipe_path, int* pipe_fd, bool waitForListener);
    int setupPipeConsumer(const char* pipe_path, int* pipe_fd, bool waitForProducer);
    int getTargetPose();
    int getCurrentMotion();
    int getTargetVel();
    int getROSParams(uint32_t timeout);
    float getParam(const char* param_name);
    void setDefaultParams(void);
    void printAllParams(void);

    //Sensor and estimator data write to file
    void debugINS(bool init);
    void debugAHRS(bool init);
    void debugMAG(bool init);
    void debugEKF(bool init);


    ParamMsg getEmptyParamMsg();
    bool validTargetVel(VelocityMsg* newVel);
    bool validTargetPose(PoseMsg* pose);
    bool validMotion(MotionMsg* m);

    void ten_hz_logging_loop();
    void update_batt_compass(void);
    void twentyfive_hz_logging();
    void three_hz_loop();
    void one_hz_loop();

    void update_GPS(void);
    void read_AHRS(void);

    void init_ardupilot(void);
    void print_blanks(int16_t num);
    void print_divider(void);
    void init_compass(void);
    bool calibrate_gyros(void);
    void startup_INS_ground(void);
    void init_barometer(bool);


    void gcs_send_heartbeat(void);
    void gcs_send_deferred(void);
    void send_heartbeat(mavlink_channel_t chan);
    void send_attitude(mavlink_channel_t chan);
    void send_location(mavlink_channel_t chan);
    void gcs_send_message(enum ap_message id);
    void gcs_data_stream_send(void);
    void gcs_check_input(void);
    void gcs_send_text(MAV_SEVERITY severity, const char *str);
    void send_extended_status1(mavlink_channel_t chan);
    void update_sensor_status_flags(void);
    void send_hwstatus(mavlink_channel_t chan);
    void send_rangefinder(mavlink_channel_t chan);
    void gcs_send_text_fmt(MAV_SEVERITY severity, const char *fmt, ...);
    void load_parameters(void);
    void dataflash_periodic(void);
    void stats_update(void);

    //radio in/out - used because of HAL dependence
    void default_dead_zones(void);
    void init_rc_in(void);
    void init_rc_out(void);
    void enable_motor_output(void);
    void read_radio(void);
    void set_throttle_and_failsafe(uint16_t throttle_pwm);
    void set_throttle_zero_flag(int16_t throttle_control);
    void radio_passthrough_to_motors(void);


//    void Log_Write_Attitude();
//    void Log_Write_MotBatt();
//    void Log_Sensor_Health();

};

extern const AP_HAL::HAL& hal;
extern Copter copter;

using AP_HAL::millis;
using AP_HAL::micros;


