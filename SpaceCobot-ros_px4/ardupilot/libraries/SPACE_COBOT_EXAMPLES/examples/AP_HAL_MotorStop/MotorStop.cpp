/*
  simple test of RC output interface
 */

#include <AP_HAL/AP_HAL.h>

void setup();
void loop();
bool boot = true;
int8_t wait = 50;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup (void)
{
    hal.console->printf("Starting AP_HAL::Motors_Stop\n");
    for (uint8_t i = 0; i< 6; i++) {
        hal.rcout->enable_ch(1);
        hal.console->printf("Enable channel:%d\n", i);

    }
    //sleep(2000);
}

static uint16_t pwm = 1500;
static int8_t delta = 1;

void stop()
{
    for (uint8_t i=0; i < 6; i++) {
           hal.rcout->write(i, 1500);
    }
    hal.scheduler->delay(wait);
    
}

void loop (void)
{   
    stop();
}

AP_HAL_MAIN();
