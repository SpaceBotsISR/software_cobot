//
// Simple test for the AP_AHRS interface
//

#include <AP_ADC/AP_ADC.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

//My includes for FIFO use
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//ROS communication structs
struct AttitudeMsg{
    int id;
    //int time;
    struct Quaternion{
        float w;
        float x;
        float y;
        float z;
    }quaternion;
};

struct StringMsg{
    int id;
    char topic[20];
    char message[512];
};


void setup();
void loop();

//My functions
void setDebugPipe();
void setAttitudePipe();
void produceAttMessage(AttitudeMsg*, Quaternion*);
void sendRosDebugMsg(const char *, const char *); //string messages only

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

//ROS communication variables
int attitude_pipe;
int debug_pipe;
int debugMsgId;
const char *imu_fifo="/home/erle/imu_fifo";
const char *debug_fifo="/home/erle/debug_fifo";

uint16_t counter;

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

    attitude_pipe = 0;
    debug_pipe = 0;
    counter = 0;

    setDebugPipe();
    setAttitudePipe();
    //###########################################################
}

void loop(void)
{
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
        //ahrs.get_secondary_quaternion(q);
        //hal.console->printf("Attitude in secondary quaternion q=[%f, %f, %f, %f]\n", q.q1, q.q2, q.q3, q.q4);
        //vehicle.EKF2.getQuaternion(1,q);
        ahrs.get_NavEKF2().getQuaternion(1,q);
        hal.console->printf("Attitude in quaternion q=[%f, %f, %f, %f]\n", q.q1, q.q2, q.q3, q.q4);

        if(attitude_pipe){
            //Send attitude quaternion to simulator
            AttitudeMsg msg;
            produceAttMessage(&msg,&q);
            if( write( attitude_pipe, &msg , sizeof(AttitudeMsg) ) < 0 ) {
                hal.console->printf("Error sending attitude message\n");
            }
        }


        //###########################################################
        last_print = now;
    }
}

GCS _gcs;

AP_HAL_MAIN();

void setDebugPipe(){

    remove(debug_fifo);
    struct stat status;

    if( mkfifo(debug_fifo, 0666 ) < 0 ) {
        hal.console->printf("Error: mkfifo(debug_fifo): ");
        return;
    }

    int readfd = open(debug_fifo, O_RDONLY | O_NONBLOCK);
    if(readfd==-1)
    {
        hal.console->printf("readfd: open(debug_fifo)");
        exit(EXIT_FAILURE);
    }

    if(fstat(readfd, &status)==-1)
    {
        hal.console->printf("fstat");
        close(readfd);
        exit(EXIT_FAILURE);
    }

    debug_pipe = open(debug_fifo, O_WRONLY | O_NONBLOCK);
    if(-1==debug_pipe)
    {
        hal.console->printf("writefd: open()");
        close(readfd);
        exit(EXIT_FAILURE);
    }
    hal.console->printf("Setup debug pipe.\n");
}

void setAttitudePipe(){

    remove(imu_fifo);
    struct stat status;

    if( mkfifo(imu_fifo, 0666 ) < 0 ) {
        hal.console->printf("Error: mkfifo(attitude_pipe): ");
        return;
    }

    int readfd = open(imu_fifo, O_RDONLY | O_NONBLOCK);
    if(readfd==-1)
    {
        hal.console->printf("readfd: open(attitude_pipe)");
        exit(EXIT_FAILURE);
    }

    if(fstat(readfd, &status)==-1)
    {
        hal.console->printf("fstat");
        close(readfd);
        exit(EXIT_FAILURE);
    }

    attitude_pipe = open(imu_fifo, O_WRONLY | O_NONBLOCK);
    if(-1==debug_pipe)
    {
        hal.console->printf("writefd: open()");
        close(readfd);
        exit(EXIT_FAILURE);
    }
    hal.console->printf("Setup attitude pipe.\n");
}

void produceAttMessage(AttitudeMsg* msg, Quaternion* q){
    msg->id = counter;
    //msg->time = AP_HAL::millis();
    msg->quaternion.w = q->q1;
    msg->quaternion.x = q->q2;
    msg->quaternion.y = q->q3;
    msg->quaternion.z = q->q4;
}


void sendRosDebugMsg(const char *topic, const char *text) {
    StringMsg debug_msg;
    debug_msg.id = debugMsgId++;
    strcpy(debug_msg.topic,topic);
    strcpy(debug_msg.message, text);

    if(write( debug_pipe, &debug_msg , sizeof(StringMsg)) < 0 ){
        //write failed
    }
}
