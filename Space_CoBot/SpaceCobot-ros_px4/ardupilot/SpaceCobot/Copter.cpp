#include "Copter.h"
#include "config.h"
#include "version.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Copter class
 */
Copter::Copter(void) :
        DataFlash{FIRMWARE_STRING},
//        G_Dt(MAIN_LOOP_SECONDS),
        inertial_nav(ahrs),
        fast_loopTimer(0),
        mainLoop_count(0)
//        in_mavlink_delay(false),
//        gcs_out_of_time(false),
//        param_loader(var_info)
{
    memset(&current_loc, 0, sizeof(current_loc));

    // init sensor error logging flags
    sensor_health.baro = true;
    sensor_health.compass = true;
}

Copter copter;

