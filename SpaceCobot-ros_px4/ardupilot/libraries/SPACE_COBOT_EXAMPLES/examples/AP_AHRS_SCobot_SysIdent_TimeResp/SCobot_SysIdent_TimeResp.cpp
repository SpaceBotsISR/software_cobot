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

double init_time;// = AP_HAL::millis();
static uint16_t counter;
static uint16_t motor_init;

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

    //init_time = hal.util->get_system_clock_ms();

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
    init_time = AP_HAL::millis();
    motor_init = init_time;
    counter = 0;
    //gps.init(nullptr, serial_manager);
    int noGps = vehicle.EKF2.setInhibitGPS();
    if(noGps>0) hal.console->printf("GPS Inhibited\n");
    else hal.console->printf("GPS not inhibited\n");
    //###########################################################
}

void loop(void)
{
    //static uint32_t last_t, last_print, last_compass;
    uint32_t now = AP_HAL::millis();
    //float heading = 0;

    /*if (now - last_compass > 100 * 1000UL &&
        compass.read()) {
        heading = compass.calculate_heading(ahrs.get_rotation_body_to_ned());
        // read compass at 10Hz
        last_compass = now;
    }*/

    ahrs.update();

    if (counter>0 && (now - motor_init >= 2000) /* 100ms : 10hz */) {//WAIT 2 SECONDS from motor initialization
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

        //#####################SYSTEM IDENTIFICATION########################
        hal.console->printf("\n");
        Vector3f euler_att;
        ahrs.get_NavEKF2().getEulerAngles(-1,euler_att);
        hal.console->printf("Euler angles=[ %f, %f, %f ]\n", euler_att.x, euler_att.y, euler_att.z);
        Eigen::Vector3d omega_current = Eigen::Vector3d(ahrs.get_gyro().x, ahrs.get_gyro().y, ahrs.get_gyro().z);
        hal.console->printf("Omega vel=[ %f, %f, %f ]\n", omega_current(0), omega_current(1), omega_current(2) );
        hal.console->printf("acceleration=[ %f, %f, %f ]\n", ahrs.get_accel_ef().x, ahrs.get_accel_ef().y, ahrs.get_accel_ef().z );

        double max_torque = 0.2;

        double torque = max_torque;//Fixed TORQUE
        Eigen::Vector3d M = Eigen::Vector3d(torque,0,0);

        hal.console->printf("time t= %f\n", t, f);
        hal.console->printf("Result torquesM'=[ %f, %f, %f ] \n", M(0), M(1), M(2));

        //#####################ACTUATE MOTORS########################

        Eigen::Vector3d force = Eigen::Vector3d(0,0,0);
        Eigen::VectorXd u = getActuationVector(force,M);
        //hal.console->printf("  u=[ %f,%f,%f,%f,%f,%f ]\n", u(0), u(1), u(2), u(3), u(4), u(5));
        hal.console->printf("pwm=[");

        //Iterate through all 6 motors
        for (uint8_t j = 0; j < 6; j++) {
            int16_t rpm = thrustToRPM( u(j) );   //calculate rpm for motor j
            uint16_t pwm = convertToPWM(rpm);       //calculate pwm for motor j
            hal.rcout->write(j, pwm);               //deliver to the motor j
            hal.console->printf(" %d   ", pwm);
        }
        hal.console->printf("]\n");

        //###########################################################

        //last_print = now;
    }
    else{
        if(counter==0){
            hal.console->printf("Initializing motors ");
            motor_init = now;
        }
        hal.console->printf(".");
        //Iterate through all 6 motors
        for (uint8_t j = 0; j < 6; j++) {
            hal.rcout->write(j, 1500);               //deliver to the motor j
        }
        counter++;
    }
}

GCS _gcs;

AP_HAL_MAIN();
