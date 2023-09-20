//
// Simple test for the AP_AHRS interface
//

#include <AP_ADC/AP_ADC.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>
#include <AC_AttitudeControl/AC_AttitudeControl_SCobot.h>
#include "SCobot_Actuation.h"
#include "AMatrix.h"

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// INS and Baro declaration
AP_InertialSensor ins;

Compass compass;

AP_GPS gps;
AP_Baro barometer;
AP_SerialManager serial_manager;

class SpaceCobot {
public:
    RangeFinder sonar {serial_manager, ROTATION_PITCH_270};
    AP_AHRS_NavEKF ahrs{ins, barometer, gps, sonar, EKF2, EKF3,
                        AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
    NavEKF2 EKF2{&ahrs, barometer, sonar};
    NavEKF3 EKF3{&ahrs, barometer, sonar};
};

static SpaceCobot vehicle;

// choose which AHRS system to use
//AP_AHRS_DCM  ahrs(ins, baro, gps);
AP_AHRS_NavEKF ahrs(vehicle.ahrs);

void setup(void)
{
    AP_BoardConfig{}.init();

    ins.init(100);
    ahrs.init();
    serial_manager.init();

    //Initialize motors
    for (uint8_t i = 0; i< 6; i++) {
        hal.rcout->enable_ch(i);
        hal.console->printf("Enable channel:%d\n", i);

    }

    if( compass.init() ) {
        hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
    //################# MY CUSTOM CODE ##########################
    //gps.init(nullptr, serial_manager);
    int noGps = vehicle.EKF2.setInhibitGPS();
    if(noGps>0) hal.console->printf("GPS Inhibited\n");
    else hal.console->printf("GPS not inhibited\n");
    //###########################################################
}

void loop(void)
{
    static uint16_t counter;
    static uint32_t last_t, last_print, last_compass;
    uint32_t now = AP_HAL::micros();
    //float heading = 0;

    if (last_t == 0) {
        last_t = now;
        return;
    }
    last_t = now;

    /*if (now - last_compass > 100 * 1000UL &&
        compass.read()) {
        heading = compass.calculate_heading(ahrs.get_rotation_body_to_ned());
        // read compass at 10Hz
        last_compass = now;
    }*/

    ahrs.update();
    counter++;

    if (now - last_print >= 100000 /* 100ms : 10hz */) {
        /*Vector3f drift  = ahrs.get_gyro_drift();
        hal.console->printf(
                "r:%4.1f  p:%4.1f y:%4.1f "
                    "drift=(%5.1f %5.1f %5.1f) hdg=%.1f rate=%.1f\n",
                (double)ToDeg(ahrs.roll),
                (double)ToDeg(ahrs.pitch),
                (double)ToDeg(ahrs.yaw),
                (double)ToDeg(drift.x),
                (double)ToDeg(drift.y),
                (double)ToDeg(drift.z),
                (double)(compass.use_for_yaw() ? ToDeg(heading) : 0.0f),
                (double)((1.0e6f * counter) / (now-last_print)));*/

        //################# MY CUSTOM CODE ##########################
        Quaternion q = Quaternion();
        ahrs.get_NavEKF2().getQuaternion(1,q);
        //hal.console->printf("Attitude in quaternion q=[%f, %f, %f, %f]\n", q.q1, q.q2, q.q3, q.q4);
        Eigen::RowVector4d q_current = Eigen::RowVector4d(q.q1, q.q2, q.q3, q.q4);
        hal.console->printf("Quaternion q_current=[ %f, %f, %f, %f ]\n",q_current(0),q_current(1),q_current(2),q_current(3));
        Vector3f euler_att;
        ahrs.get_NavEKF2().getEulerAngles(-1,euler_att);
        hal.console->printf("Euler angles = [%f, %f, %f]\n", euler_att.x, euler_att.y, euler_att.z);

        Eigen::RowVector4d q_des = Eigen::RowVector4d(1.0,0.0,0.0,0.0);

        Eigen::Vector3d omega_current = Eigen::Vector3d(ahrs.get_gyro().x, ahrs.get_gyro().y, ahrs.get_gyro().z);
        hal.console->printf("Omega current=[ %f, %f, %f ]\n", omega_current(0), omega_current(1), omega_current(2) );
        hal.console->printf("acceleration=[ %f, %f, %f ]\n", ahrs.get_accel_ef().x, ahrs.get_accel_ef().y, ahrs.get_accel_ef().z );
        Eigen::Vector3d omega_des = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d omegaD_des = Eigen::Vector3d(0,0,0);

        Eigen::Vector3d M = attitudeController(q_current,q_des,omega_current,omega_des,omegaD_des);

        hal.console->printf("Result torquesM'=[ %f, %f, %f ] \n\n", M(0), M(1), M(2));

        //#####################ACTUATE MOTORS########################

        Eigen::Vector3d force = Eigen::Vector3d(0,0,0);
        Eigen::VectorXd u = getActuationVector(force,M);
        hal.console->printf("  u=[ %f,%f,%f,%f,%f,%f ]\n", u(0), u(1), u(2), u(3), u(4), u(5));
        hal.console->printf("pwm=[");

        //Iterate through all 6 motors
        for (uint8_t j = 0; j < 6; j++) {
            int16_t rpm = thrustToRPM( u(j) );   //calculate rpm for motor j
            uint16_t pwm = convertToPWM(rpm);       //calculate pwm for motor j
            hal.rcout->write(j, pwm);               //deliver to the motor j
            hal.console->printf(" %d   ,", pwm);
        }
        hal.console->printf("]\n\n");

        //###########################################################
        last_print = now;
        counter = 0;
    }
}

GCS _gcs;

AP_HAL_MAIN();
