//
// Created by Filipe Rosa on 12/12/2017.
//

#include "Copter.h"
#include "AC_AttitudeControl_SCobot.h"
#include "SCobot_Actuation.h"
#include "PC_PositionControl_SCobot.h"
#include "AMatrix.h"

//My includes for FIFO use
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//needed to write debug data to file
#include <iostream>
#include <fstream>

#include <AP_NavEKF/AP_Nav_Common.h>

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Copter, &copter, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
//        SCHED_TASK(three_hz_loop,          3,     75),
//        SCHED_TASK(compass_accumulate,   100,    100),
//        SCHED_TASK(update_GPS,            50,    200), //Obtain position from simulator
//        SCHED_TASK(barometer_accumulate,  50,     90),
//        SCHED_TASK(update_notify,         50,     90),
        SCHED_TASK(one_hz_loop,            1,    100),
//        SCHED_TASK(ekf_check,             10,     75),
        SCHED_TASK(update_batt_compass,   10,    120),
//        SCHED_TASK(ten_hz_logging_loop,   10,    350),
//        SCHED_TASK(twentyfive_hz_logging, 25,    110),
//        SCHED_TASK(perf_update,           0.1,    75),
//        SCHED_TASK(rpm_update,            10,    200),
//        SCHED_TASK(compass_cal_update,   100,    100),
//        SCHED_TASK(accel_cal_update,      10,    100),
//        SCHED_TASK(gcs_check_input,      400,    180),
//        SCHED_TASK(gcs_send_heartbeat,     1,    110),
//        SCHED_TASK(gcs_send_deferred,     50,    550),
//        SCHED_TASK(gcs_data_stream_send,  50,    550),
//        SCHED_TASK(dataflash_periodic,    400,    300),
//        SCHED_TASK(stats_update,           1,    100),
}; ///ATTENTION: Build fails if scheduler is left empty

void Copter::setup() {
    cliSerial = hal.console;

    // setup storage layout for copter
    StorageManager::set_layout_copter();


    ///My custom part below - initialization

    err_int = Eigen::Vector3d(0.0,0.0,0.0);

    ///We start at z=1 because that is the initial height on simulation
    currentMotion = {};
    currentMotion.pos = {0,0,1};
    currentMotion.id = -1;
    prevMotion = currentMotion;

    targetPose.position = {0,0,1};
    targetPose.att_quaternion = {1.0,0.0,0.0,0.0};
    targetPose.id = -1;
    prevTargetPose=targetPose;

    targetVel = {};
    targetVel.id = -1;
    prevTargetVel = targetVel;

    attPipe_fd = 0;
    actPipe_fd = 0;
    posePipe_fd = 0;
    motionPipe_fd = 0;
    velPipe_fd = 0;
    paramPipe_fd = 0;

    debugINS_fd = 0;
    debugINS(true);
    debugMAG(true);
    debugAHRS(true);
    debugEKF(true);

    lastROSParam = {};
    lastROSParam.f_value = -1.0;

    setDefaultParams();
    setupCommunicationPipes();
    //printAllParams();

    //barometer.set_hil_mode();

    ///Sensor initialization
    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    // setup initial performance counters
    //perf_info_reset();
    fast_loopTimer = AP_HAL::micros();

    /*
    int noGps = ahrs.setInhibitGPS();
    if(noGps>0) cliSerial->printf("AHRS GPS Inhibited\n");
    else cliSerial->printf("AHRS GPS not inhibited\n");

    noGps = EKF2.setInhibitGPS();
    if(noGps>0) cliSerial->printf("EKF2 GPS Inhibited\n");
    else cliSerial->printf("EKF2 GPS not inhibited\n");

    noGps = EKF3.setInhibitGPS();
    if(noGps>0) cliSerial->printf("EKF3 GPS Inhibited\n");
    else cliSerial->printf("EKF3 GPS not inhibited\n");
    */

}

void Copter::loop() {
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    //perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop - CONTROL LOOP
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    char time_print[512];
    sprintf(time_print,"Time available %d",time_available);
    sendRosDebugMsg("/scobot/debug",time_print);
    scheduler.run(time_available > MAIN_LOOP_MICROS ? 0u : time_available);

}

void Copter::fast_loop() {
    // update INS immediately to get current gyro data populated
    ins.update();
    debugINS(false); //Add a param for activating/deactivating debug from ROS
    compass_accumulate();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();
    debugAHRS(false);
    debugEKF(false);

    Quaternion q = Quaternion();
    //ahrs.get_NavEKF2().getQuaternion(1,q); //EKF attitude - ORIGINALLY USED THIS ONE
    ahrs.get_secondary_quaternion(q);//DCM attitude

    Quaternion x_180_rotation = Quaternion(0,1,0,0);
    q = q.operator*(x_180_rotation); //We get it in IMU frame so we rotate it to the body frame
    q =x_180_rotation.operator*(q);
    q.normalize();

    Eigen::RowVector4d q_current = Eigen::RowVector4d(q.q1, q.q2, q.q3, q.q4);

    //q.rotation_matrix();

    if(attPipe_fd){
        //Send attitude quaternion to simulator
        AttitudeMsg msg={};
        produceAttMessage(&msg,&q);
        if( write( attPipe_fd, &msg , sizeof(AttitudeMsg) ) < 0 ) {
            sendRosDebugMsg("/scobot/debug","Error sending attitude message");
        }
    }
    ///#########################POSITION CONTROL##################################
    Eigen::Vector3d force = Eigen::Vector3d(0,0,0);

    getCurrentMotion();
    getTargetPose();
    getTargetVel();
    //if(getCurrentMotion() && getTargetPose() && getTargetVel()){
        Eigen::Vector3d p_curr = Eigen::Vector3d(currentMotion.pos.x,currentMotion.pos.y,currentMotion.pos.z);
        Eigen::Vector3d p_des = Eigen::Vector3d(targetPose.position.x,targetPose.position.y,targetPose.position.z);
        Eigen::Vector3d v_curr = Eigen::Vector3d(currentMotion.vel.x,currentMotion.vel.y,currentMotion.vel.z);
        Eigen::Vector3d v_des = Eigen::Vector3d(targetVel.v.x, targetVel.v.y, targetVel.v.z);
        Eigen::Matrix3d rotation_matrix = getRotationMatrix(q_current);

        ///Proportional position gain                     axis:     x               y               z
        Eigen::Matrix3d KXp = Eigen::DiagonalMatrix<double, 3>(getParam(KXP_x),getParam(KXP_y),getParam(KXP_z));

        ///Derivate position gain                         axis:     x               y               z
        Eigen::Matrix3d KXd = Eigen::DiagonalMatrix<double, 3>(getParam(KXD_x),getParam(KXD_y),getParam(KXD_z));

        force = positionController(getParam(MASS),p_curr,p_des,v_curr,v_des,rotation_matrix,KXp,KXd);

        char str[512];
        sprintf(str,"p=[%f,%f,%f];p_des=[%f,%f,%f];v_curr=[%f,%f,%f];v_des=[%f,%f,%f];F=[%f,%f,%f]",
                p_curr[0],p_curr[1],p_curr[2],
                p_des[0],p_des[1],p_des[2],
                v_curr[0],v_curr[1],v_curr[2],
                v_des[0],v_des[1],v_des[2],
                force[0],force[1],force[2]
        );
        //sprintf(str,"F=[%f,%f,%f]",force[0],force[1],force(2));
        sendRosDebugMsg("/scobot/pc_debug",str);
    //}
    //else sendRosDebugMsg("/scobot/pc_debug","No valid data yet");


    ///#########################ATTITUDE CONTROL##################################
    //hal.console->printf("Attitude in quaternion q=[%f, %f, %f, %f]\n", q.q1, q.q2, q.q3, q.q4);

    //hal.console->printf("Quaternion q_current=[ %f, %f, %f, %f ]\n",q_current(0),q_current(1),q_current(2),q_current(3));
    Vector3f euler_att;
    ahrs.get_NavEKF2().getEulerAngles(-1,euler_att);
    //cliSerial->printf("Euler angles = [%f, %f, %f]\n", euler_att.x, euler_att.y, euler_att.z);

    Eigen::RowVector4d q_des = Eigen::RowVector4d(targetPose.att_quaternion.w,targetPose.att_quaternion.x,targetPose.att_quaternion.y,targetPose.att_quaternion.z);
    /*
    Quaternion q_des_body_frame;
    q_des_body_frame.q1 = targetPose.att_quaternion.w;
    q_des_body_frame.q2 = targetPose.att_quaternion.x;
    q_des_body_frame.q3 = targetPose.att_quaternion.y;
    q_des_body_frame.q4 = targetPose.att_quaternion.z;

    Quaternion q_des_IMU_frame = x_180_rotation.operator*(q_des_body_frame);
    Eigen::RowVector4d q_des = Eigen::RowVector4d(q_des_IMU_frame.q1,q_des_IMU_frame.q2,q_des_IMU_frame.q3,q_des_IMU_frame.q4);
     */

    Eigen::Vector3d omega_current_IMU_frame = Eigen::Vector3d(ahrs.get_gyro().x, ahrs.get_gyro().y, ahrs.get_gyro().z);
    Eigen::Matrix3d rot_matrix_180 = Eigen::DiagonalMatrix<double, 3>(1,-1,-1);
    Eigen::Vector3d omega_current = rot_matrix_180*omega_current_IMU_frame;

    //hal.console->printf("Omega current=[ %f, %f, %f ]\n", omega_current(0), o  mega_current(1), omega_current(2) );
    //hal.console->printf("acceleration=[ %f, %f, %f ]\n", ahrs.get_accel_ef().x, ahrs.get_accel_ef().y, ahrs.get_accel_ef().z );
    Eigen::Vector3d omega_des = Eigen::Vector3d(targetVel.w.x, targetVel.w.y, targetVel.w.z);
    Eigen::Vector3d omegaD_des = Eigen::Vector3d(0,0,0);

    ///Proportional gain                               axis:        x               y               z
    Eigen::Matrix3d K_ang = Eigen::DiagonalMatrix<double, 3>(getParam(Kang_x),getParam(Kang_y),getParam(Kang_z));
    ///Derivate gain                                    axis:       x               y               z
    Eigen::Matrix3d K_omg = Eigen::DiagonalMatrix<double, 3>(getParam(Komg_x),getParam(Komg_y),getParam(Komg_z));
    ///Integral gain                                    axis:       x               y               z
    Eigen::Matrix3d Ki = Eigen::DiagonalMatrix<double, 3>(getParam(Ki_x),getParam(Ki_y),getParam(Ki_z));

    float time_step = 0.02; //TODO: Move this to a ROS PARAM and pass it as a function argument
    Eigen::Vector3d M = attitudeController(q_current,q_des,omega_current,omega_des,omegaD_des,K_ang,K_omg,&err_int,Ki,time_step,getParam(WINDUP_LIMIT_UP),getParam(WINDUP_LIMIT_LOW));

    char ac_print[512];
    sprintf(str,"Torque=[%f,%f,%f] q_des=[w= %f, %f, %f, %f] err_int=[%f,%f,%f]",
            M(0), M(1), M(2), q_des(0),
            q_des(1), q_des(2), q_des(3),
            err_int.x(),err_int.y(),err_int.z()
    );
    sendRosDebugMsg("/scobot/ac_debug",str);
    //cliSerial->printf("Result torquesM'=[ %f, %f, %f ] \n\n", M(0), M(1), M(2));

    ///#####################ACTUATE MOTORS########################

    Eigen::VectorXd u = getActuationVector(force,M);

    if(actPipe_fd){
        //Send attitude quaternion to simulator
        ActuationMsg msg={};
        produceActMsg(&msg,u);
        if( write( actPipe_fd, &msg , sizeof(ActuationMsg) ) < 0 ) {
            sendRosDebugMsg("/scobot/debug","Error sending actuation message");
        }
    }

    //hal.console->printf("  u=[ %f,%f,%f,%f,%f,%f ]\n", u(0), u(1), u(2), u(3), u(4), u(5));
    //cliSerial->printf("pwm=[");
    char pwm_print[200];
    strcpy(pwm_print,"pwm=[ ");
    char taskTime_print[200];
    strcpy(taskTime_print,"write_micros=[");
    int exec_time;
    //Iterate through all 6 motors - THIS ITERATION IS TAKING MORE TIME THAN THE LOOP TIME AVAILABLE
    for (uint8_t j = 0; j < N_MOTORS; j++) {
        int16_t rpm = thrustToRPM( u(j) );   //calculate rpm for motor j
        uint16_t pwm = convertToPWM(rpm);       //calculate pwm for motor j

        ///Safe range of PWM - if outside STOP MOTOR
        if(pwm < getParam(PWM_MIN))  pwm = getParam(PWM_MIN);
        else if(pwm > getParam(PWM_MAX)) pwm = getParam(PWM_MAX);
        if( getParam(ARMED)==0.0 ) pwm = 1500; // idle PWM
        exec_time = micros();
        hal.rcout->write(j, pwm);               //deliver to the motor j
        exec_time = micros() - exec_time;

        sprintf(pwm_print,"%s %d,",pwm_print,pwm);
        sprintf(taskTime_print,"%s %d,",taskTime_print,exec_time);

        //cliSerial->printf(" %d   ,", pwm);
    }
    strcat(pwm_print," ]  ");
    strcat(pwm_print,taskTime_print);
    strcat(pwm_print," ]");


    sendRosDebugMsg("/scobot/pwm",pwm_print);
    //cliSerial->printf("]\n\n");

}

void Copter::produceAttMessage(AttitudeMsg* msg, Quaternion* q){
    msg->id = mainLoop_count;
    //msg->time = AP_HAL::millis();
    msg->quaternion.w = q->q1;
    msg->quaternion.x = q->q2;
    msg->quaternion.y = q->q3;
    msg->quaternion.z = q->q4;
}

void Copter::produceActMsg(ActuationMsg * msg, Eigen::VectorXd u) {
    msg->id = mainLoop_count;
    msg->U.u1 = u(0);
    msg->U.u2 = u(1);
    msg->U.u3 = u(2);
    msg->U.u4 = u(3);
    msg->U.u5 = u(4);
    msg->U.u6 = u(5);
}

int Copter::setupCommunicationPipes() {

    //Send debug msgs to ROS
    setupPipeProducer(debugPipe_path,&debugPipe_fd,false);

    //ROS parameters pipe
    setupPipeConsumer(paramPipe_path, &paramPipe_fd, false);
    cliSerial->printf("Start the SCobot_Params ROS node\n");
    sleep(3); //give a few seconds to launch the node
    int np = getROSParams(micros()+3*MICRO);
    printAllParams();
    cliSerial->printf("Loaded %d parameters from ROS.\n",np);

    //publish IMU data to ROS
    setupPipeProducer(attPipe_path, &attPipe_fd, true);

    //publish actuation vector
    setupPipeProducer(actPipe_path, &actPipe_fd, true);

    //read robot current positions and linear velocity
    setupPipeConsumer(motionPipe_path, &motionPipe_fd, false);

    //read target pose
    setupPipeConsumer(posePipe_path, &posePipe_fd, false);

    //read target velocities
    setupPipeConsumer(velPipe_path, &velPipe_fd, false);

    cliSerial->printf("All pipe connections set.\n");
    return 1;
}

void Copter::sendRosDebugMsg(const char *topic, const char *text) {
    if(debugPipe_fd==0 || getParam(ROS_DEBUG)==0.0) return;
    StringMsg debug_msg = {};
    debug_msg.id = debugMsgId++;
    strcpy(debug_msg.topic,topic);
    strcpy(debug_msg.message, text);

    if(write( debugPipe_fd, &debug_msg , sizeof(StringMsg)) < 0 ){
        //write failed -> do nothing
    }
}

/**
 * Setup a linux fifo producer
 *
 * @param pipe_path - path to the fifo
 * @param pipe_fd - pointer to variable where the fifo file descriptor will be stored
 * @param waitForListener - true if open is a a blocking operation (waits for other process open())
 *                        - false if non-blocking -> we'll write to the pipe regardless of anyone being reading
 * @return
 */
int Copter::setupPipeProducer(const char *pipe_path, int * pipe_fd, bool waitForListener) {
    remove(pipe_path);
    struct stat status;

    if( mkfifo(pipe_path, 0666 ) < 0 ) {
        cliSerial->printf("Error: mkfifo(%s): ",pipe_path);
        return 0;
    }

    int readfd = open(pipe_path, O_RDONLY | O_NONBLOCK);
    if(readfd==-1)
    {
        cliSerial->printf("readfd: open(debugPipe_path)");
        exit(EXIT_FAILURE);
    }

    if(fstat(readfd, &status)==-1)
    {
        cliSerial->printf("fstat");
        close(readfd);
        exit(EXIT_FAILURE);
    }

    if(waitForListener){ //Wait for ROS node to open pipe. Mind that if it closes we terminate execution.
        close(readfd);
        cliSerial->printf("Trying to connect to ROS Node\n");
        *pipe_fd = open(pipe_path, O_WRONLY);
    }
    else *pipe_fd = open(pipe_path, O_WRONLY | O_NONBLOCK); //Here we open without blocking


    if(*pipe_fd==-1)
    {
        cliSerial->printf("writefd: open(%s)",pipe_path);
        close(readfd);
        exit(EXIT_FAILURE);
    }
    cliSerial->printf("[PRODUCER] Setup %s pipe.\n",pipe_path);
    return 1;
}

/**
 * Setup a linux fifo listener
 *
 * @param pipe_path - path to the fifo
 * @param pipe_fd - pointer to variable where the fifo file descriptor will be stored
 * @param waitForProducer - true if open is a a blocking operation (waits for other process open())
 *                        - false if non-blocking -> reads will fail until someone writes to the pipe
 * @return
 */
int Copter::setupPipeConsumer(const char* pipe_path, int *pipe_fd, bool waitForProducer){
    remove(pipe_path);
    struct stat status;

    if( mkfifo(pipe_path, 0666 ) < 0 ) {
        perror("Error: mkfifo( ): ");
        return 0;
    }
    //Change ownwership of the fifo because we are executing as root
    chown(pipe_path,ERLE_UID_VALUE,ERLE_UID_VALUE);

    if(waitForProducer){
        printf("Trying to connect to ROS node.\n");
        *pipe_fd = open(pipe_path, O_RDONLY);
    }
    else *pipe_fd = open(pipe_path, O_RDONLY | O_NONBLOCK);

    if(*pipe_fd==-1)
    {
        perror("readfd: open()");
        exit(EXIT_FAILURE);
    }

    if(fstat(*pipe_fd, &status)==-1)
    {
        perror("fstat");
        close(*pipe_fd);
        exit(EXIT_FAILURE);
    }

    cliSerial->printf("[CONSUMER] Setup %s pipe.\n",pipe_path);
    return 1;
}


void Copter::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

/*
  update AP_Stats
 */
void Copter::stats_update(void)
{
    g2.stats.update();
}

/*
  if the compass is enabled then try to accumulate a reading
 */
void Copter::compass_accumulate(void) {
    compass.accumulate();
    debugMAG(false);
}

void Copter::compass_cal_update() {

}

void Copter::barometer_accumulate() {
    //cliSerial->printf("\n.\n");
    //barometer.accumulate();
}

void Copter::accel_cal_update(){

}

void Copter::one_hz_loop() {

    //Check for ROS param updates for 100 milliseconds
    getROSParams(micros()+100);
}

void Copter::ten_hz_logging_loop() {}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(){

    battery.read();

    // update compass with current value
    if (battery.has_current()) {
        compass.set_current(battery.current_amps());
    }
    //compass.learn_offsets();
    compass.read();
}

void Copter::three_hz_loop() {
    cliSerial->printf("Millis: %d\n",millis());

    Location newLoc;
    ahrs.get_position(newLoc);
    cliSerial->printf("Pos: [%d, %d, %d]    ",newLoc.lat,newLoc.lng,newLoc.alt);
    ahrs.get_location(newLoc);
    cliSerial->printf("Loc: [%d, %d, %d]    ",newLoc.lat,newLoc.lng,newLoc.alt);
    newLoc = gps.location();
    cliSerial->printf("GPS: [%d, %d, %d]\n",newLoc.lat,newLoc.lng,newLoc.alt);
}

void Copter::twentyfive_hz_logging() {}

void Copter::update_GPS() {
    //Todo: READ location from the simulator through the pos_pipe
    Location loc;
    loc.lat = 0;//rand();//x
    loc.lng = 0;//rand();//y
    loc.alt = 0;//rand();//z
    Vector3f vel = Vector3f();
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;
    gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,millis(),loc, vel, 10, 0);
    barometer.setHIL(loc.alt);


}

void Copter::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
//#if HIL_MODE != HIL_MODE_DISABLED
//    // update hil before ahrs update
//    gcs_check_input();
//#endif

    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
}

/**
 * Read from pipe the target position and attitude sent by the trajectory planner
 * @return 1: success  0: read error  -1: invalid data
 */
int Copter::getTargetPose(){
    if(!posePipe_fd) return 0;

    PoseMsg newPose = {};
    newPose.id = -1;
    if( (read(posePipe_fd , &newPose, sizeof(PoseMsg))) < 0){
        //sendRosDebugMsg("/scobot/fifo","Erro: read(posePipe_fd): ");
        return 0;
    }

    if(newPose.id==-1) return 0; // no data was read
    if(!validTargetPose(&newPose)) return -1;

    prevTargetPose = targetPose;
    targetPose = newPose;

    char str[512];
    sprintf(str,"p_ref=[%f,%f,%f] q_ref=[%f,%f,%f,%f]",
            newPose.position.x,
            newPose.position.y,
            newPose.position.z,
            newPose.att_quaternion.w,
            newPose.att_quaternion.x,
            newPose.att_quaternion.y,
            newPose.att_quaternion.z);
    sendRosDebugMsg("/scobot/fifo2",str);
    return 1;
}

/**
 * read from pipe the current position and and linear velocity sent by the simulator
 * @return 1: success  0: read error  -1: invalid data
 */
int Copter::getCurrentMotion(){

    if(motionPipe_fd==0) return 0;

    MotionMsg newMotion = {};
    newMotion.id = -1;
    if( (read( motionPipe_fd , &newMotion, sizeof(MotionMsg))) < 0){
        //sendRosDebugMsg("/scobot/fifo","Erro: read(motionPipe_fd): ");
        return 0;
    }

    if(newMotion.id==-1) return 0; //no data was read
    if(!validMotion(&newMotion)) return -1;

    prevMotion = currentMotion;
    currentMotion = newMotion;

    char str[512];
    sprintf(str,"p=[%f,%f,%f] v=[%f,%f,%f]",
            newMotion.pos.x,
            newMotion.pos.y,
            newMotion.pos.z,
            newMotion.vel.x,
            newMotion.vel.y,
            newMotion.vel.z);
    sendRosDebugMsg("/scobot/fifo1",str);
    return 1;
}

/**
 * Read from pipe the target linear and angular velocities sent by the trajectory planner
 *
 * @return 1: success  0: read error  -1: invalid data
 */
int Copter::getTargetVel(){
    if(velPipe_fd==0) return 0;

    VelocityMsg newVel = {};
    newVel.id = -1;
    if( (read(velPipe_fd,&newVel, sizeof(VelocityMsg))) < 0)
        return 0;

    if(newVel.id==-1) return 0; //no data read
    if(!validTargetVel(&newVel)) return -1;

    prevTargetVel = targetVel;
    targetVel = newVel;

    char str[512]="";
    sprintf(str,"v_ref=[%f,%f,%f] w_ref=[%f,%f,%f]",
            newVel.v.x,
            newVel.v.y,
            newVel.v.z,
            newVel.w.x,
            newVel.w.y,
            newVel.w.z
    );
    sendRosDebugMsg("/scobot/fifo3",str);
    return 1;

}

/**
 * Check the pipe for ROS parameters and update value in params table
 * @return number of retrieved params
 */
int Copter::getROSParams(uint32_t timeout){

    int counter = 0;
    ParamMsg lastItem = {};

    while (true) {
        ParamMsg msg = {};
        if( (read(paramPipe_fd, &msg, sizeof(ParamMsg))) < 0){

            //We read at a much higher rate than the node writes to the pipe.
            //NONBLOCK pipe reader won't wait for producer so plenty of reads will fail.
            //Tolerate failures until timeout is reached.

            //set timeout
            if(micros()>timeout) break;
        }
        if(strcmp(lastItem.name,msg.name)!=0 || lastItem.f_value != msg.f_value){
            cliSerial->printf("param: %s value: %f",msg.name, msg.f_value);
            if( param_map.find(msg.name)!=param_map.end() ){//received parameter is valid
                param_map[msg.name] = msg.f_value;
                cliSerial->printf("   LOADED\n");
                counter++;
            }
            else cliSerial->printf("   NOT FOUND\n");

        }
        lastItem = msg;
    }
    return counter;
}

ParamMsg Copter::getEmptyParamMsg(){
    ParamMsg msg;
    strcpy(msg.name, "");
    msg.f_value = 0.0;
    return msg;
}

bool Copter::validTargetVel(VelocityMsg* newVel){
    //Return if newVel has a nonsense id
    if(newVel->id>(prevTargetVel.id+1000000)) return false;

    //validate linear velocity
    if(newVel->v.x > getParam(MAX_LIN_VEL) || newVel->v.y > getParam(MAX_LIN_VEL) || newVel->v.z > getParam(MAX_LIN_VEL))
        return false;
    if(newVel->v.x < getParam(MIN_LIN_VEL) || newVel->v.y < getParam(MIN_LIN_VEL) || newVel->v.z < getParam(MIN_LIN_VEL))
        return false;

    //validate angular velocity
    if(newVel->w.x > getParam(MAX_ANG_VEL) || newVel->w.y > getParam(MAX_ANG_VEL) || newVel->w.z > getParam(MAX_ANG_VEL))
        return false;
    if(newVel->w.x < getParam(MIN_ANG_VEL) || newVel->w.y < getParam(MIN_ANG_VEL) || newVel->w.z < getParam(MIN_ANG_VEL))
        return false;
    return true;
}

bool Copter::validTargetPose(PoseMsg* pose){

    //validate position coordinates
    if(pose->position.x > getParam(MAX_X) || pose->position.y > getParam(MAX_Y) || pose->position.z > getParam(MAX_Z) )
        return false;
    if(pose->position.x < getParam(MIN_X) || pose->position.y < getParam(MIN_Y) || pose->position.z < getParam(MIN_Z) )
        return false;

    //validate quaternion
    if(pose->att_quaternion.w < -1 || pose->att_quaternion.w > 1)
        return false;
    if(pose->att_quaternion.x < -1 || pose->att_quaternion.x > 1)
        return false;
    if(pose->att_quaternion.y < -1 || pose->att_quaternion.y > 1)
        return false;
    if(pose->att_quaternion.z < -1 || pose->att_quaternion.z > 1)
        return false;

    return true;
}

bool Copter::validMotion(MotionMsg* m){

    //validate position coordinates
    if( m->pos.x > getParam(MAX_X) || m->pos.y > getParam(MAX_Y) || m->pos.z > getParam(MAX_Z) )
        return false;
    if( m->pos.x < getParam(MIN_X) || m->pos.y < getParam(MIN_Y) || m->pos.z < getParam(MIN_Z) )
        return false;

    //validate linear velocity
    if(m->vel.x > getParam(MAX_LIN_VEL) || m->vel.y > getParam(MAX_LIN_VEL) || m->vel.z > getParam(MAX_LIN_VEL))
        return false;
    if(m->vel.x < getParam(MIN_LIN_VEL) || m->vel.y < getParam(MIN_LIN_VEL) || m->vel.z < getParam(MIN_LIN_VEL))
        return false;

    return true;
}

float Copter::getParam(const char* param_name){
    return param_map.find(param_name)->second;
}


void Copter::setDefaultParams(){

    param_map[ERLE_UID      ] = ERLE_UID_VALUE   ;
    param_map[MAX_LIN_VEL   ] = MAX_LIN_VEL_VALUE;
    param_map[MAX_ANG_VEL   ] = MAX_ANG_VEL_VALUE;
    param_map[MIN_LIN_VEL   ] = MIN_LIN_VEL_VALUE;
    param_map[MIN_ANG_VEL   ] = MIN_ANG_VEL_VALUE;
    param_map[MAX_X         ] = MAX_X_VALUE      ;
    param_map[MAX_Y         ] = MAX_Y_VALUE      ;
    param_map[MAX_Z         ] = MAX_Z_VALUE      ;
    param_map[MIN_X         ] = MIN_X_VALUE      ;
    param_map[MIN_Y         ] = MIN_Y_VALUE      ;
    param_map[MIN_Z         ] = MIN_Z_VALUE      ;
    param_map[PWM_MAX       ] = PWM_MAX_VALUE    ;
    param_map[PWM_MIN       ] = PWM_MIN_VALUE    ;
    param_map[ARMED         ] = ARMED_VALUE      ;
    param_map[KXP_z         ] = KXP_z_VALUE      ;
    param_map[KXP_y         ] = KXP_y_VALUE      ;
    param_map[KXP_x         ] = KXP_x_VALUE      ;
    param_map[KXD_z         ] = KXD_z_VALUE      ;
    param_map[KXD_y         ] = KXD_y_VALUE      ;
    param_map[KXD_x         ] = KXD_x_VALUE      ;
    param_map[Kang_z        ] = Kang_z_VALUE     ;
    param_map[Kang_x        ] = Kang_x_VALUE     ;
    param_map[Kang_y        ] = Kang_y_VALUE     ;
    param_map[Komg_y        ] = Komg_y_VALUE     ;
    param_map[Komg_x        ] = Komg_x_VALUE     ;
    param_map[Komg_z        ] = Komg_z_VALUE     ;
    param_map[Ki_x          ] = Ki_x_VALUE       ;
    param_map[Ki_y          ] = Ki_y_VALUE       ;
    param_map[Ki_z          ] = Ki_z_VALUE       ;
    param_map[ROS_DEBUG     ] = ROS_DEBUG_VALUE  ;
    param_map[CSV_DEBUG     ] = CSV_DEBUG_VALUE  ;
    param_map[MAG_DEC       ] = MAG_DEC_VALUE    ;
    param_map[COMPASS_OFS_X ] = COMPASS_OFS_X_VALUE  ;
    param_map[COMPASS_OFS_Y ] = COMPASS_OFS_Y_VALUE  ;
    param_map[COMPASS_OFS_Z ] = COMPASS_OFS_Z_VALUE  ;
    param_map[MASS          ] = MASS_VALUE       ;
    param_map[WINDUP_LIMIT_UP] = WINDUP_LIMIT_UP_VALUE;
    param_map[WINDUP_LIMIT_LOW] = WINDUP_LIMIT_LOW_VALUE;

}

void Copter::printAllParams() {
    cliSerial->printf("\n################# SPACE COBOT PARAMETERS #################\n");
    for (std::map<const char *,float>::iterator it = param_map.begin() ; it != param_map.end(); ++it){
        cliSerial->printf("param: %s   value: %f\n",it->first, it->second );
    }
    cliSerial->printf("##########################################################\n\n");

}

AP_HAL_MAIN_CALLBACKS(&copter);

