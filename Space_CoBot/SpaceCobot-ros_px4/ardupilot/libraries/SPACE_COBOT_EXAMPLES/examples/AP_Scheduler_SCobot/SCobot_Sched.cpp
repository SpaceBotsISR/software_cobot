//
// Simple test for the AP_Scheduler interface
//

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
//#include <AP_Stats/AP_Stats.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

// Common dependencies
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Menu/AP_Menu.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>

#include "SpaceCobot.h"

// Local modules
//#include "Parameters.h"


const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// INS and Baro declaration
AP_InertialSensor ins;
Compass compass;
AP_GPS gps;
AP_Baro barometer;
AP_SerialManager serial_manager;


class SCobot_Sched {
public:
    //friend class Parameters;
    //friend class ParametersG2;
    void setup();
    void loop();

private:

    AP_Scheduler scheduler;

    // Global parameters are all contained within the 'g' class.
    //Parameters g;
    //ParametersG2 g2;

    uint32_t ins_counter;
    static const AP_Scheduler::Task scheduler_tasks[];
    Quaternion attQ;

    void ins_update(void);
    void one_hz_print(void);
    void five_second_call(void);
    void readAttitude(void);
};

static SCobot_Sched sched;

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(SCobot_Sched, &sched, func, _interval_ticks, _max_time_micros)

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task SCobot_Sched::scheduler_tasks[] = {
    SCHED_TASK(ins_update,             50,   1000),
    SCHED_TASK(one_hz_print,            1,   1000),
    SCHED_TASK(five_second_call,      0.2,   1800),
    //SCHED_TASK(stats_update,           1,    100),
};

void SCobot_Sched::setup(void)
{

    AP_BoardConfig{}.init();

    ins.init(scheduler.get_loop_rate_hz());

    SpaceCobot::ahrs.init();

    // initialise the scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

void SCobot_Sched::loop(void)
{
    // wait for an INS sample
    ins.wait_for_sample();

    readAttitude();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all tasks that fit in 20ms
    scheduler.run(20000);
}

/*
  update inertial sensor, reading data 
 */
void SCobot_Sched::ins_update(void)
{
    ins_counter++;
    ins.update();
}

/*
  print something once a second
 */
void SCobot_Sched::one_hz_print(void)
{
    //hal.console->printf("one_hz: t=%lu \n", (unsigned long)AP_HAL::millis());
    Vector3f accel_reading = ins.get_accel();
    hal.console->printf("accel_x=%f accel_y=%f accel_z=%f\n", accel_reading.x, accel_reading.y, accel_reading.z );
    Vector3f gyro_reading = ins.get_gyro();
    hal.console->printf("gyro_x=%f gyro_y=%f gyro_z=%f\n", gyro_reading.x, gyro_reading.y, gyro_reading.z );
}

/*
  print something every 5 seconds
 */
void SCobot_Sched::five_second_call(void)
{
    hal.console->printf("five_seconds: t=%lu ins_counter=%u\n", (unsigned long)AP_HAL::millis(), ins_counter);
}



/*
  update AP_Stats

void Copter::stats_update(void)
{
    g2.stats.update();
}*/


/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup(void)
{
    sched.setup();
}
void loop(void)
{
    sched.loop();
}
AP_HAL_MAIN();
