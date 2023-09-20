//
// Created by Filipe Rosa on 08/04/2018.
//
//Functions to write data to csv files
//
#include "Copter.h"

#include <iostream>
#include <fstream>

void Copter::debugINS(bool init) {
    //Open the file
    if(init){
        std::ofstream myfile;
        myfile.open ("ins.csv");
        myfile << "time,gyro_x,gyro_y,gyro_z,gyro_offset_x,gyro_offset_y,gyro_offset_z,accel_x,accel_y,accel_z,accel_offset_x,accel_offset_y,accel_offset_z\n";
        myfile.close();
        return;
    }
    if(getParam(CSV_DEBUG)==0.0) return;

    std::ofstream myfile;
    myfile.open ("ins.csv",std::ios_base::app);

    myfile << (int) millis() << " ,"
           << ins.get_gyro()[0] << " ," << ins.get_gyro()[1] << " ," << ins.get_gyro()[2] << " ,"
           << ins.get_gyro_offsets()[0] << " ," << ins.get_gyro_offsets()[1] << " ," << ins.get_gyro_offsets()[2] << " ,"
           << ins.get_accel()[0] << " ," << ins.get_accel()[1] << " ," << ins.get_accel()[2] << " ,"
           << ins.get_accel_offsets()[0] << " ," << ins.get_accel_offsets()[1] << " ," << ins.get_accel_offsets()[2] << "\n";
    myfile.close();

}

void Copter::debugMAG(bool init) {
    if(init){
        std::ofstream myfile;
        myfile.open ("mag.csv");
        myfile << "time,field_x,field_y,field_z,offset_x,offset_y,offset_z,heading_a_x,"
               << "heading_a_y,heading_a_z,heading_b_x,heading_b_y,heading_b_z,heading_c_x,heading_c_y,heading_c_z,"
               << "heading_roll,heading_pitch,heading_yaw,"
               << "configured,healthy,is_calibrating,compass_cal_requires_reboot,bad_ekf_mag_variance\n";
        myfile.close();
        return;
    }
    if(getParam(CSV_DEBUG)==0.0) return;

    std::ofstream myfile;
    myfile.open ("mag.csv",std::ios_base::app);

    Matrix3f heading;
    compass.calculate_heading(heading);
    float roll, pitch, yaw;
    heading.to_euler(&roll,&pitch,&yaw);

    myfile << (int) millis() << " ,"
           << compass.get_field().x << " ," << compass.get_field().y << " ," << compass.get_field().z << " ,"
           << compass.get_offsets().x << " ," << compass.get_offsets().y << " ," << compass.get_offsets().z << " ,"
           << heading.a.x << " ," << heading.a.y << " ," << heading.a.z << " ," << heading.b.x << " ," << heading.b.y << " ," << heading.b.z << " ," << heading.c.x << " ," << heading.c.y << " ," << heading.c.z << " ,"
           << roll << " ," << pitch << " ," << yaw << " ,"
           << compass.configured() << " ," << compass.healthy() << " ," << compass.is_calibrating() << " ," << compass.compass_cal_requires_reboot() << " ,";

    // check EKF compass variance is below failsafe threshold
    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    ahrs.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);
    if (mag_variance.length() >= copter.g.fs_ekf_thresh){
        //gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: EKF compass variance");
        myfile << 1;
    }
    else myfile << 0;


    myfile << "\n";
    myfile.close();
}

void Copter::debugAHRS(bool init){
    if(init){
        std::ofstream myfile;
        myfile.open ("ahrs.csv");
        myfile << "time,healthy,ekf_type,initialised,have_inertial_nav,"
               << "mag_NED_x,mag_NED_y,mag_NED_z,"
               << "gyro_drift_x,gyro_drift_y,gyro_drift_z,"
               << "pos_lat, pos_lng, pos_alt,"
               << "vel_variance,pos_variance,hgt_variance,tas_variance,offset_x,offset_y,"
               << "ahrs_roll,ahrs_pitch,ahrs_yaw,"
               << "DCM_roll,DCM_pitch,DCM_yam\n";
        myfile.close();
        return;
    }
    if(getParam(CSV_DEBUG)==0.0) return;

    Vector3f mag_NED;
    ahrs.get_mag_field_NED(mag_NED);

    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    ahrs.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);

    //nav_filter_status stat;
    //ahrs.get_filter_status(stat);
    //stat.flags.attitude;
    //compass.start_calibration_all()
    Location pos;
    ahrs.get_position(pos);

    Vector3f euler_DCM;
    ahrs.get_secondary_attitude(euler_DCM);


    std::ofstream myfile;
    myfile.open ("ahrs.csv",std::ios_base::app);
    myfile << millis() << " ,"
           << ahrs.healthy() << " ," << ahrs.get_ekf_type() << " ," << ahrs.initialised() << " ," << ahrs.have_inertial_nav() << " ,"
           << mag_NED.x << " ," << mag_NED.y << " ," << mag_NED.z << " ,"
           << pos.lat << " ," << pos.lng << " ," << pos.alt << " ,"
           << ahrs.get_gyro_drift().x << " ," << ahrs.get_gyro_drift().y << " ," << ahrs.get_gyro_drift().z << " ,"
           << vel_variance << " ," << pos_variance << " ," << hgt_variance << " ," << tas_variance << " ," << offset.x << " ," << offset.y << " ,"
           << ahrs.roll << " ," << ahrs.pitch << " ," << ahrs.yaw << " ,"
           << euler_DCM[0] << " ," << euler_DCM[1] << " ," << euler_DCM[2] << "\n";
    myfile.close();
}

void Copter::debugEKF(bool init) {
    if(init){
        std::ofstream myfile;
        myfile.open ("ekf2.csv");
        myfile << "time,healthy,faults,pos_N,pos_E,pos_D,"
               << "PosDownDerivative,accel_N,accel_E,accel_D,"
               << "last_calc_Lat,last_calc_Lng,last_calc_Hgt,"
               << "euler_x,euler_y,euler_z,"
               << "velInnov_x,velInnov_y,velInnov_z,"
               << "posInnov_x,posInnov_y,posInnov_z,"
               << "magInnov_x,magInnov_y,magInnov_z,"
               << "tasInnov, yawInnov,"
               << "error_x,error_y,error_z,"
               << "vel_variance,pos_variance,hgt_variance,tas_variance,"
               << "mag_variance_x,mag_variance_y,mag_variance_z,"
               << "offset_x,offset_y,"
               << "\n";
        myfile.close();
        return;
    }
    if(getParam(CSV_DEBUG)==0.0) return;

    uint16_t faults;
    ahrs.get_NavEKF2().getFilterFaults(-1, faults);

    Vector2f posNE;
    ahrs.get_NavEKF2().getPosNE(-1,posNE);
    float pos_D;
    ahrs.get_NavEKF2().getPosD(-1,pos_D);

    Vector3f accelNED;
    ahrs.get_NavEKF2().getAccelNED(accelNED);

    Location last_calc;
    ahrs.get_NavEKF2().getLLH(last_calc);

    Vector3f euler;
    ahrs.get_NavEKF2().getEulerAngles(-1,euler);

    Vector3f velInnov, posInnov, magInnov;
    float tasInnov, yawInnov;
    ahrs.get_NavEKF2().getInnovations(-1,velInnov, posInnov, magInnov,tasInnov, yawInnov);

    Vector3f error;
    ahrs.get_NavEKF2().getOutputTrackingError(-1,error);

    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    ahrs.get_NavEKF2().getVariances(-1,vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);

    std::ofstream myfile;
    myfile.open ("ekf2.csv",std::ios_base::app);
    myfile << millis() << " ,"
           << ahrs.get_NavEKF2().healthy() << " ," << faults << " ," << posNE[0] << " ," << posNE[1] << " ," << pos_D << " ,"
           << ahrs.get_NavEKF2().getPosDownDerivative(-1) << " ," << accelNED[0] << " ," << accelNED[1] << " ," << accelNED[2] << " ,"
           << last_calc.lat << " ," << last_calc.lng << " ," << last_calc.alt << " ,"
           << euler.x << " ," << euler.y << " ," << euler.z << " ,"
           << velInnov.x << " ," << velInnov.y << " ," << velInnov.z << " ,"
           << posInnov.x << " ," << posInnov.y << " ," << posInnov.z << " ,"
           << magInnov.x << " ," << magInnov.y << " ," << magInnov.z << " ,"
           << tasInnov << " ," << yawInnov << " ,"
           << error.x << " ," << error.y << " ," << error.z << " ,"
           << vel_variance << " ," << pos_variance << " ," << hgt_variance << " ," << tas_variance << " ,"
           << mag_variance.x << " ," << mag_variance.y << " ," << mag_variance.z << " ,"
           << offset.x << " ," << offset.y << " ,"
           << "\n";
    myfile.close();
}