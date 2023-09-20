//
// Created by Filipe Rosa on 14/12/2017.
//
#include "Copter.h"
#include "version.h"
#include <unistd.h>

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/
static void mavlink_delay_cb_static()
{
    copter.mavlink_delay_cb();
}

void Copter::init_ardupilot()
{
    // initialise serial port
    serial_manager.init_console();

    // init vehicle capabilties
    //init_capabilities();

    cliSerial->printf("\n\nInit " FIRMWARE_STRING
    "\n\nFree RAM: %u\n",
            (unsigned)hal.util->available_memory());

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    //report_version();
    cliSerial->printf("FW Ver: %s\n",THISFIRMWARE);
    print_divider();
    print_blanks(2);

    // load parameters from EEPROM
    load_parameters();

    // initialise stats module
    g2.stats.init();

    gcs().set_dataflash(&DataFlash);

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;

    // initialise serial ports
    serial_manager.init();

    // setup first port early to allow BoardConfig to report errors
    gcs_chan[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    BoardConfig.init();

    // initialise notify system
    //notify.init(true);
    //notify_flight_mode(control_mode);

    // initialise battery monitor
    battery.init();

    // Init RSSI
    //rssi.init();

    barometer.init();

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    //ap.usb_connected = true;
    //check_usb_mux();

    //setup telem slots with serial ports
    for (uint8_t i = 1; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        gcs_chan[i].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, i);
    }

//#if LOGGING_ENABLED == ENABLED
//    log_init();
//#endif

    init_rc_in();               // sets up rc channels from radio

    // allocate the motors class
    //allocate_motors();

    init_rc_out();              // sets up motors and output to escs

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    //hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    //gps.init(&DataFlash, serial_manager);

    //if(g.compass_enabled)
    init_compass();


    // init Location class
    Location_Class::set_ahrs(&ahrs);


//#ifdef USERHOOK_INIT
//    USERHOOK_INIT
//#endif

//#if CLI_ENABLED == ENABLED
//    if (g.cli_enabled) {
//        const char *msg = "\nPress ENTER 3 times to start interactive setup\n";
//        cliSerial->printf("%s\n", msg);
//        if (gcs_chan[1].initialised && (gcs_chan[1].get_uart() != nullptr)) {
//            gcs_chan[1].get_uart()->printf("%s\n", msg);
//        }
//        if (num_gcs > 2 && gcs_chan[2].initialised && (gcs_chan[2].get_uart() != nullptr)) {
//            gcs_chan[2].get_uart()->printf("%s\n", msg);
//        }
//    }
//#endif // CLI_ENABLED
//
//#if HIL_MODE != HIL_MODE_DISABLED
//    while (barometer.get_last_update() == 0) {
//        // the barometer begins updating when we get the first
//        // HIL_STATE message
//        gcs_send_text(MAV_SEVERITY_WARNING, "Waiting for first HIL_STATE message");
//        delay(1000);
//    }
//
//    // set INS to HIL mode
//    ins.set_hil_mode();
//#endif

    // read Baro pressure at ground
    //-----------------------------
    init_barometer(false);

    // initialise rangefinder
    //init_rangefinder();


    // initialise the flight mode and aux switch
    // ---------------------------
    //reset_control_switch();
    //init_aux_switches();

    startup_INS_ground();

    // set landed flags
    //set_land_complete(true);
    //set_land_complete_maybe(true);

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    //failsafe_enable();

    //ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
    ins.set_dataflash(&DataFlash);

    // enable output to motors
    //arming.pre_arm_rc_checks(true);
    //Initialize motors
    for (uint8_t i = 0; i< N_MOTORS; i++) {
        hal.rcout->enable_ch(i);
        hal.rcout->write(i,1500);
        cliSerial->printf("Enable channel:%d\n", i);
    }
    sleep(2); //Wait for motors to initialize


    cliSerial->printf("\nReady to FLY \n");

    // flag that initialisation has completed
    ap.initialised = true;
}

void Copter::print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->printf("\n");
    }
}

void Copter::print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        cliSerial->printf("-");
    }
    cliSerial->printf("\n");
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Copter::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    //ins.set_board_orientation(ROTATION_ROLL_180);
    //compass.set_board_orientation(ROTATION_ROLL_180);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// calibrate gyros - returns true if successfully calibrated
bool Copter::calibrate_gyros()
{
    // gyro offset calibration
    copter.ins.init_gyro();

    // reset ahrs gyro bias
    if (copter.ins.gyro_calibrated_ok_all()) {
        copter.ahrs.reset_gyro_drift();
        return true;
    }

    return false;
}

// initialise compass
void Copter::init_compass()
{
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->printf("COMPASS INIT ERROR\n");
        //Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ///We programmatically set compass ofsets unlike with ArduCopter which requires calibration on a GCS
    //Todo: Allow compass offset updates mid execution if motors not ARMED
    Vector3f offsets;
    offsets.x = getParam(COMPASS_OFS_X);
    offsets.y = getParam(COMPASS_OFS_Y);
    offsets.z = getParam(COMPASS_OFS_Z);
    compass.set_and_save_offsets(0,offsets);
    compass.set_declination(ToRad(getParam(MAG_DEC)),true);

    cliSerial->printf("Compass declination=%f offsets x=%f, y=%f, z=%f\n",ToDeg(compass.get_declination()),compass.get_offsets().x,compass.get_offsets().y,compass.get_offsets().z);

    ahrs.set_compass(&compass);
}

void Copter::init_barometer(bool full_calibration)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Calibrating barometer");
    if (full_calibration) {
        barometer.calibrate();
    }else{
        barometer.update_calibration();
    }
    gcs_send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");
}

// update error mask of sensors and subsystems. The mask
// uses the MAV_SYS_STATUS_* values from mavlink. If a bit is set
// then it indicates that the sensor or subsystem is present but
// not functioning correctly.
void Copter::update_sensor_status_flags(void)
{
    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (copter.DataFlash.logging_present()) { // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
/*#if PROXIMITY_ENABLED == ENABLED
    if (copter.g2.proximity.get_status() > AP_Proximity::Proximity_NotConnected) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
#endif*/
    if (copter.battery.healthy()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }


    // all present sensors enabled by default except altitude and position control and motors which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS &
                                                         ~MAV_SYS_STATUS_LOGGING &
                                                         ~MAV_SYS_STATUS_SENSOR_BATTERY);

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    /*if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }*/

    if (copter.DataFlash.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }

    if (g.fs_batt_voltage > 0 || g.fs_batt_mah > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }



    // default to all healthy
    control_sensors_health = control_sensors_present;

    if (!barometer.all_healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (!g.compass_enabled || !compass.healthy() || !ahrs.use_compass()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() == AP_GPS::NO_GPS) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_GPS;
    }
//    if (!ap.rc_receiver_present || failsafe.radio) {
//        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
//    }

    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (copter.DataFlash.logging_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_LOGGING;
    }

#if RANGEFINDER_ENABLED == ENABLED
    if (rangefinder_state.enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (rangefinder.has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

    if (!ap.initialised || ins.calibrating()) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

//    if (copter.failsafe.battery) {
//        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_BATTERY;                                                                    }
}


