/*
  simple test of RC output interface
 */

#include <AP_HAL/AP_HAL.h>
#include "AMatrix.h"
#include <AP_Math/definitions.h>

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

//declare functions
void waving();
uint16_t convertToPWM(int16_t rpm);

void setup (void)
{
    hal.console->printf("Starting AP_HAL::RCOut_SCobot test\n");
    for (uint8_t i = 0; i< 6; i++) {
        hal.rcout->enable_ch(1);
        hal.console->printf("Enable channel:%d\n", i);

    }
    //sleep(2000);
}

void loop (void)
{
    //waving();
    if(boot){
        //for (uint8_t i = 0; i< 6; i++) {
            hal.rcout->write(1, pwm);

        //}
        hal.scheduler->delay(1000);
        boot = !boot;
    }
    else{

        if(counter < (sizeof(inputRPM)/sizeof(inputRPM[0])) ){
            hal.console->printf("Requested %d RPM\n", inputRPM[counter]);
            pwm = convertToPWM(inputRPM[counter]);
            hal.console->printf("Calculated pwm: %d \n\n", pwm);
            counter++;

        }

        hal.rcout->write(0, 1500); //ONLY SENDING TO MOTOR 1
        hal.scheduler->delay(3000);
    }
    Eigen::Vector3f force;
    force << 1,2,3;
    Eigen::Vector3f torque;
    torque << 4,5,6;
    Eigen::VectorXf u = getActuationVector(force,torque);
    hal.console->printf("Actuation vector u=[");
    int i=0;
    while(i<6){
        hal.console->printf(" %f ", u(i));
        i++;
    }
    hal.console->printf("]\n");

}

uint16_t convertToPWM(int16_t rpm){
    uint16_t mypwm;
    rpm /= 4/3; // 4/3 = (14.8 / 11.1) CONVERT FROM RPM ON 4S TO 3S
    if(rpm>0){
        mypwm = (uint16_t) (0.000000214802726 * pow(rpm, 2) + 0.0121696724019783 * rpm + 1499.25490902604);
    }
    else{
        mypwm = (uint16_t) (-0.000000193881142 * pow(rpm, 2) + 0.0124845157903363 * rpm + 1467.04520972071);
    }
    if(mypwm > 1250 && mypwm < 1750) return mypwm;
    return 1500;
}

void waving()
{
    if(boot){
        for (uint8_t i = 0; i< 6; i++) {
            hal.rcout->write(i, pwm);

        }
        hal.scheduler->delay(1000);
        boot = !boot;
    }
    else{
        for (uint8_t i=0; i < 6; i++) {
            hal.rcout->write(i, pwm);

            pwm += delta;
            if (delta > 0 && pwm >= 1700) {
                delta = -1;
                hal.console->printf("waving\n");
            } else if (delta < 0 && pwm <= 1300) {
                delta = 1;
                hal.console->printf("waving\n");
            }
        }
        hal.scheduler->delay(50);
    }
}

AP_HAL_MAIN();
