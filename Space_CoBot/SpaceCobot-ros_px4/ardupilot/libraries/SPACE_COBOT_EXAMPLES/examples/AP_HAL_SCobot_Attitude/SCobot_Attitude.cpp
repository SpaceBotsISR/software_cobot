/*
  simple test of RC output interface
 */

#include <AP_HAL/AP_HAL.h>
#include "AMatrix.h"
#include <AP_Math/definitions.h>
#include <AP_NavEKF2/AP_NavEKF2_core.h>

void setup();
void loop();
bool boot = true;


const AP_HAL::HAL& hal = AP_HAL::get_HAL();
uint16_t pwm = 1500;
static int8_t delta = 1;
static int16_t inputRPM[] = {424,
                             1752,
                             2752,
                             3792,
                             4780,
                             4780,
                             3792,
                             2752,
                             1752,
                             424};
int8_t counter = 0;
Quaternion att;

//declare functions
void waving();

void setup (void)
{
    hal.console->printf("Starting AP_HAL::RCOut_SCobot test\n");
    for (uint8_t i = 0; i< 6; i++) {
        hal.rcout->enable_ch(1);
        hal.console->printf("Enable channel:%d\n", i);

    }
    att = Quaternion();
    //sleep(2000);
}

void loop (void)
{
    //waving();
    if(boot){
        //for (uint8_t i = 0; i< 6; i++) {
            hal.rcout->write(1, 1500);

        //}
        hal.scheduler->delay(1000);
        boot = !boot;
    }
    else{
        Quaternion* q = &att;
        NavEKF2_core::getQuaternion(q);
        hal.console->printf("Att quaternion = [%f,%f,%f,%f]:\n",att.q1,att.q2,att.q3,att.q4);
        hal.scheduler->delay(1000);
    }


}

AP_HAL_MAIN();
