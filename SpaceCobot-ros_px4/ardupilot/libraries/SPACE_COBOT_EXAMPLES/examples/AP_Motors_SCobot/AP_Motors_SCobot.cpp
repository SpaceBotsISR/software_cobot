/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

// Libraries
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include <AP_Motors/AP_Motors.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <DataFlash/DataFlash.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ADC/AP_ADC.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Baro/AP_Baro.h>
#include <Filter/Filter.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Scheduler/AP_Scheduler.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// declare functions
void setup();
void loop();
void motor_order_test();
void stability_test();
void update_motors();

#define HELI_TEST       0   // set to 1 to test helicopters
#define NUM_OUTPUTS     6   // set to 4 for quadcopter, 6 for hexacopter, 8 for octacopter and heli

//RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);
//RC_Channel rc7(6), rsc(8), h1(0), h2(1), h3(2), h4(3);

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(400);
AP_MotorsMatrix   motors(400);
//AP_MotorsHeli_Single motors(rc7, rsc, h1, h2, h3, h4, 400);
//AP_MotorsSingle motors(400);
//AP_MotorsCoax motors(400);

// setup
void setup()
{
    hal.console->printf("AP_Motors library test Space CoBot version \n");

    // motor initialisation
    motors.set_update_rate(400);
    hal.console->printf("step 1\n");
    motors.init(AP_Motors::MOTOR_FRAME_HEXA, AP_Motors::MOTOR_FRAME_TYPE_X);
    hal.console->printf("step 2\n");
    motors.enable();
    hal.console->printf("step 3\n");
    motors.output_min();
    hal.console->printf("step 4\n");

    /*// setup radio
	rc3.set_radio_min(1000);
    rc3.set_radio_max(2000);

    // set rc channel ranges
    rc1.set_angle(4500);
    rc2.set_angle(4500);
    rc3.set_range(1000);
    rc4.set_angle(4500);*/

    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int16_t value;

    // display help
    hal.console->printf("Press 't' to run motor orders test, 's' to run stability patch test.  Be careful the motors will spin!\n");

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    // test motors
    if (value == 't' || value == 'T') {
        motor_order_test();
    }
}

void motor_order_test()
{
    hal.console->printf("testing motor order\n");
    motors.armed(true);
    for (int8_t i=1; i <= AP_MOTORS_MAX_NUM_MOTORS; i+=2 ) { //CW MOTORS
        hal.console->printf("Motor %d\n",(int)i);
        motors.output_test(i, 1515);//PWM
        hal.scheduler->delay(300);
        motors.output_test(i, 1500);
        hal.scheduler->delay(2000);
    }
    for (int8_t i=2; i <= AP_MOTORS_MAX_NUM_MOTORS; i+=2) { //CCW MOTORS
        hal.console->printf("Motor %d\n",(int)i);
        motors.output_test(i, 1455);//PWM
        hal.scheduler->delay(300);
        motors.output_test(i, 1500);
        hal.scheduler->delay(2000);
    }
    motors.armed(false);
    hal.console->printf("finished test.\n");

}

void update_motors()
{
    // call update motors 1000 times to get any ramp limiting complete
    for (uint16_t i=0; i<4000; i++) {
        motors.output();
    }
}

AP_HAL_MAIN();
