/*
  simple test of RC output interface
 */

#include <AP_HAL/AP_HAL.h>
#include "AMatrix.h"
#include "SCobot_Actuation.h"
#include <AP_Math/definitions.h>

void setup();
void loop();
bool boot = true;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static int8_t delta = 1;
int8_t counter = 0;

void setup (void)
{
    hal.console->printf("Starting AP_HAL::RCOut_SCobot test\n");
    for (uint8_t i = 0; i< 6; i++) {
        hal.rcout->enable_ch(i);
        hal.console->printf("Enable channel:%d\n", i);

    }
    //sleep(2000);
}

void loop (void)
{
    //waving();
    if(boot){
        for (uint8_t i = 0; i< 6; i++) {
            hal.rcout->write(i, 1500);

        }
        hal.scheduler->delay(2000);
        boot = !boot;
    }
    else{
        Eigen::Vector3f force;
        force << 0,0,0;
        Eigen::Vector3f torque;
        torque << 0,0,0.2;
        Eigen::VectorXf u = getActuationVector(force,torque);
        hal.console->printf("  u=[%f,%f,%f,%f,%f,%f]\n", u(0), u(1), u(2), u(3), u(4), u(5));
        hal.console->printf("pwm=[");

        //Iterate through all 6 motors
        for (uint8_t j = 0; j < 6; j++) {
            int16_t rpm = thrustToRPM( u(j) );   //calculate rpm for motor j
            uint16_t pwm = convertToPWM(rpm);       //calculate pwm for motor j
            hal.rcout->write(j, pwm);               //deliver to the motor j
            hal.console->printf(" %d   ,", pwm);
        }
        hal.console->printf("]\n\n");

        hal.scheduler->delay(1000);
    }


}



AP_HAL_MAIN();
