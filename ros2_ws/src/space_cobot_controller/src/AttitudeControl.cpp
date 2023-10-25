//
// Created by socrob on 07-02-2019.
//

#include "AttitideControl.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/src/Core/Matrix.h>

//using namespace Eigen;

Eigen::Matrix3d getRotationMatrix(Eigen::RowVector4d q_current)
{
    Eigen::Matrix3d R;
    R <<   powf(q_current(0),2.0)+powf(q_current(1),2.0) - powf(q_current(2),2.0) - powf(q_current(3),2.0), 2*(q_current(1)*q_current(2) - q_current(0)*q_current(3)), 2*(q_current(1)*q_current(3)+ q_current(0)*q_current(2)),
            2*(q_current(1)*q_current(2) + q_current(0)*q_current(3)), powf(q_current(0),2.0)-powf(q_current(1),2.0) + powf(q_current(2),2.0) - powf(q_current(3),2.0), 2*(q_current(2)*q_current(3) - q_current(0)*q_current(1)),
            2*(q_current(1)*q_current(3) - q_current(0)*q_current(2)), 2*(q_current(2)*q_current(3) + q_current(0)*q_current(1)), powf(q_current(0),2.0)-powf(q_current(1),2.0) - powf(q_current(2),2.0) + powf(q_current(3),2.0);
    return R;
}

Eigen::Matrix3d getDesiredRotationMatrix(Eigen::RowVector4d q_des){
   Eigen::Matrix3d Rd;
    Rd <<   powf(q_des(0),2)+powf(q_des(1),2) - powf(q_des(2),2) - powf(q_des(3),2),    2*(q_des(1)*q_des(2) - q_des(0)*q_des(3)),                                   2*(q_des(1)*q_des(3)+ q_des(0)*q_des(2)),
            2*(q_des(1)*q_des(2) + q_des(0)*q_des(3)),                                  powf(q_des(0),2)-powf(q_des(1),2) + powf(q_des(2),2) - powf(q_des(3),2),     2*(q_des(2)*q_des(3) - q_des(0)*q_des(1)),
            2*(q_des(1)*q_des(3) - q_des(0)*q_des(2)),                                  2*(q_des(2)*q_des(3) + q_des(0)*q_des(1)),                                   powf(q_des(0),2)-powf(q_des(1),2) - powf(q_des(2),2) + powf(q_des(3),2);


    return Rd;
}

Eigen::Vector3d invskew(Eigen::Matrix3d mat){
    Eigen::Vector3d vec;
    vec << mat(2,1), mat(0,2), mat(1,0);
    return vec;
}

Eigen::Matrix3d skew(Eigen::Vector3d vec){
    Eigen::Matrix3d mat;
    mat <<  0,       -vec(2), vec(1),
            vec(2),  0,       -vec(0),
            -vec(1), vec(0),  0;
    return mat;
}

float trace(Eigen::Matrix3d mat){
    return mat(0,0) + mat(1,1) + mat(2,2);
}

Eigen::Vector3d attitudeController(Eigen::RowVector4d q_current, Eigen::RowVector4d q_des, Eigen::Vector3d omega_current, Eigen::Vector3d omega_des, Eigen::Vector3d omegaD_des){
    //K Constants, may have to be tuned

    ///CALIBRATION:
    ///se funciona no x manter posição 1,1 da matrix a 0.1 e depsois ajustar 2,2 e 3,3 para valores que funcionem
    ///
    /// Começar a k_ang de y e z a 0.01, subir em potências de 10 até obter uma velocidade de convergência que pareça razoavel.
    /// Quando convergir ele vai oscilar e nesse momento começar a subir o ganho derivativo (K_omg)
    ///
    /// K_omg -> Para calibração dos restantes eixos colocar a zero.


    ///Proportional gain                               axis:  x   y   z
    Eigen::Matrix3d K_ang = Eigen::DiagonalMatrix<double, 3>(0.1,0.1,0.1);

    ///Derivate gain                                    axis:   x      y      z
    Eigen::Matrix3d K_omg = Eigen::DiagonalMatrix<double, 3>(0.1510,0.1510,0.1510);

    //The inertia matrix is considered diagonal here, with I = [I(1,1), 0 , 0; 0, I(2,2), 0 ; 0, 0, I(3,3)];
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    //Rotation matrices
    Eigen::Matrix3d R = getRotationMatrix(q_current);
    Eigen::Matrix3d Rt = R.transpose();

    Eigen::Matrix3d R_d = getDesiredRotationMatrix(q_des);
    Eigen::Matrix3d R_dt = R_d.transpose();


    //terms of the control equation
    //angular and angular velocity errors
    Eigen::Vector3d err_ang = (1/(2*sqrt(1+trace(R_dt*R))))*invskew(R_dt*R - Rt*R_d);
    Eigen::Vector3d err_omg = omega_current - (Rt*R_d*omega_des);

    //other terms
    Eigen::Vector3d term3 = skew(Rt*R_d*omega_des)*I*(Rt*R_d*omega_des);
    Eigen::Vector3d term4 = I*(Rt*R_d*omegaD_des);

    //controller equation
    Eigen::Vector3d M = -K_ang*err_ang - K_omg*err_omg + term3 +term4;

    return M;
}

//Currently using this one
Eigen::Vector3d attitudeController(Eigen::RowVector4d q_current, Eigen::RowVector4d q_des, Eigen::Vector3d omega_current,
                                   Eigen::Vector3d omega_des, Eigen::Vector3d omegaD_des, Eigen::Matrix3d K_ang, Eigen::Matrix3d K_omg,
                                   Eigen::Vector3d &err_int, Eigen::Matrix3d K_i, float time_step,
                            float wind_up_limit_up, float wind_up_limit_low){
    //K Constants, may have to be tuned

    ///CALIBRATION:
    ///se funciona no x manter posição 1,1 da matrix a 0.1 e depsois ajustar 2,2 e 3,3 para valores que funcionem
    ///
    /// Começar a k_ang de y e z a 0.01, subir em potências de 10 até obter uma velocidade de convergência que pareça razoavel.
    /// Quando convergir ele vai oscilar e nesse momento começar a subir o ganho derivativo (K_omg)
    ///
    /// K_omg -> Para calibração dos restantes eixos colocar a zero.

    //The inertia matrix is considered diagonal here, with I = [I(1,1), 0 , 0; 0, I(2,2), 0 ; 0, 0, I(3,3)];
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    //Rotation matrices
    Eigen::Matrix3d R = getRotationMatrix(q_current);
    Eigen::Matrix3d Rt = R.transpose();

    Eigen::Matrix3d R_d = getDesiredRotationMatrix(q_des);
    Eigen::Matrix3d R_dt = R_d.transpose();


    //terms of the control equation
    //angular and angular velocity errors
    Eigen::Vector3d err_ang = (1/(2*sqrt(1+trace(R_dt*R))))*invskew(R_dt*R - Rt*R_d);
    Eigen::Vector3d err_omg = omega_current - (Rt*R_d*omega_des);

    //other terms
    Eigen::Vector3d term3 = skew(Rt*R_d*omega_des)*I*(Rt*R_d*omega_des);
    Eigen::Vector3d term4 = I*(Rt*R_d*omegaD_des);

    //Integral part
    err_int = (time_step * err_ang) + (err_int);
    ///x
    if( err_int[0] > wind_up_limit_up ) err_int[0] = wind_up_limit_up;
    else if( err_int[0] < wind_up_limit_low) err_int[0] = wind_up_limit_low;
    ///y
    if( err_int[1] > wind_up_limit_up ) err_int[1] = wind_up_limit_up;
    else if( err_int[1] < wind_up_limit_low) err_int[1] = wind_up_limit_low;
    ///z
    if( err_int[2] > wind_up_limit_up ) err_int[2] = wind_up_limit_up;
    else if( err_int[2] < wind_up_limit_low) err_int[2] = wind_up_limit_low;

    //controller equation
    Eigen::Vector3d M = -K_i*(err_int) + -K_ang*err_ang - K_omg*err_omg + term3 +term4;

    return M;
}

Eigen::Vector3d attitudeControllerDebug(Eigen::RowVector4d q_current, Eigen::RowVector4d q_des, Eigen::Vector3d omega_current, Eigen::Vector3d omega_des, Eigen::Vector3d omegaD_des){
    //K Constants, may have to be tuned
    ///Proportional gain                               axis:  x   y   z
    Eigen::Matrix3d K_ang = Eigen::DiagonalMatrix<double, 3>(0.1,0.1,0.1);

    ///Derivate gain                                    axis:   x      y      z
    Eigen::Matrix3d K_omg = Eigen::DiagonalMatrix<double, 3>(0.1510,0.1510,0.1510);

    //The inertia matrix is considered diagonar here, with I = [I(1,1), 0 , 0; 0, I(2,2), 0 ; 0, 0, I(3,3)];
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    //Rotation matrices
    Eigen::Matrix3d R = getRotationMatrix(q_current);
    Eigen::Matrix3d Rt = R.transpose();

    Eigen::Matrix3d R_d = getDesiredRotationMatrix(q_des);
    Eigen::Matrix3d R_dt = R_d.transpose();


    //terms of the control equation
    //angular and angular velocity errors
    Eigen::Vector3d err_ang = (1/(2*sqrt(1+trace(R_dt*R))))*invskew(R_dt*R - Rt*R_d);
    Eigen::Vector3d err_omg = omega_current - (Rt*R_d*omega_des);

    //other terms
    Eigen::Vector3d term3 = skew(Rt*R_d*omega_des)*I*(Rt*R_d*omega_des);
    Eigen::Vector3d term4 = I*(Rt*R_d*omegaD_des);

    //controller equation
    Eigen::Vector3d M = -K_ang*err_ang - K_omg*err_omg + term3 +term4;

    return M;
}
