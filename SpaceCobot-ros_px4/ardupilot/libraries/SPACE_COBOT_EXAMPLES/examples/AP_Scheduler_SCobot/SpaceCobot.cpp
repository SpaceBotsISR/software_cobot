//
// Created by Filipe Rosa on 11/09/2017.
//

#include "SpaceCobot.h"

void SpaceCobot::SpaceCobot(void){
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
    //###########################################################
}

Quaternion SpaceCobot::getAttitude() {
    ahrs.update();
    attQ = Quaternion();
    return ahrs.get_secondary_quaternion(attQ);
}