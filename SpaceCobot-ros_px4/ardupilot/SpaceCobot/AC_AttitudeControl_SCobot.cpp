//
// Created by Filipe Rosa on 12/10/2017.
//

#include "AC_AttitudeControl_SCobot.h"

using namespace Eigen;

Matrix3d getRotationMatrix(RowVector4d q_current)
{
    Matrix3d R;
    R <<   powf(q_current(0),2.0)+powf(q_current(1),2.0) - powf(q_current(2),2.0) - powf(q_current(3),2.0), 2*(q_current(1)*q_current(2) - q_current(0)*q_current(3)), 2*(q_current(1)*q_current(3)+ q_current(0)*q_current(2)),
            2*(q_current(1)*q_current(2) + q_current(0)*q_current(3)), powf(q_current(0),2.0)-powf(q_current(1),2.0) + powf(q_current(2),2.0) - powf(q_current(3),2.0), 2*(q_current(2)*q_current(3) - q_current(0)*q_current(1)),
            2*(q_current(1)*q_current(3) - q_current(0)*q_current(2)), 2*(q_current(2)*q_current(3) + q_current(0)*q_current(1)), powf(q_current(0),2.0)-powf(q_current(1),2.0) - powf(q_current(2),2.0) + powf(q_current(3),2.0);
    return R;
}

Matrix3d getDesiredRotationMatrix(RowVector4d q_des){
    Matrix3d Rd;
    Rd <<   powf(q_des(0),2)+powf(q_des(1),2) - powf(q_des(2),2) - powf(q_des(3),2),    2*(q_des(1)*q_des(2) - q_des(0)*q_des(3)),                                   2*(q_des(1)*q_des(3)+ q_des(0)*q_des(2)),
            2*(q_des(1)*q_des(2) + q_des(0)*q_des(3)),                                  powf(q_des(0),2)-powf(q_des(1),2) + powf(q_des(2),2) - powf(q_des(3),2),     2*(q_des(2)*q_des(3) - q_des(0)*q_des(1)),
            2*(q_des(1)*q_des(3) - q_des(0)*q_des(2)),                                  2*(q_des(2)*q_des(3) + q_des(0)*q_des(1)),                                   powf(q_des(0),2)-powf(q_des(1),2) - powf(q_des(2),2) + powf(q_des(3),2);


    return Rd;
}

Vector3d invskew(Matrix3d mat){
    Vector3d vec;
    vec << mat(2,1), mat(0,2), mat(1,0);
    return vec;
}

Matrix3d skew(Vector3d vec){
    Matrix3d mat;
    mat <<  0,       -vec(2), vec(1),
            vec(2),  0,       -vec(0),
            -vec(1), vec(0),  0;
    return mat;
}

float trace(Matrix3d mat){
    return mat(0,0) + mat(1,1) + mat(2,2);
}

Vector3d attitudeController(RowVector4d q_current, RowVector4d q_des, Vector3d omega_current, Vector3d omega_des, Vector3d omegaD_des){
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

    //The inertia matrix is considered diagonar here, with I = [I(1,1), 0 , 0; 0, I(2,2), 0 ; 0, 0, I(3,3)];
    Matrix3d I = Matrix3d::Identity();

    //Rotation matrices
    Matrix3d R = getRotationMatrix(q_current);
    Matrix3d Rt = R.transpose();

    Matrix3d R_d = getDesiredRotationMatrix(q_des);
    Matrix3d R_dt = R_d.transpose();


    //terms of the control equation
    //angular and angular velocity errors
    Vector3d err_ang = (1/(2*sqrt(1+trace(R_dt*R))))*invskew(R_dt*R - Rt*R_d);
    Vector3d err_omg = omega_current - (Rt*R_d*omega_des);

    //other terms
    Vector3d term3 = skew(Rt*R_d*omega_des)*I*(Rt*R_d*omega_des);
    Vector3d term4 = I*(Rt*R_d*omegaD_des);

    //controller equation
    Vector3d M = -K_ang*err_ang - K_omg*err_omg + term3 +term4;

    return M;
}

Vector3d attitudeController(RowVector4d q_current, RowVector4d q_des, Vector3d omega_current,
                            Vector3d omega_des, Vector3d omegaD_des, Matrix3d K_ang, Matrix3d K_omg,
                            Vector3d* err_int, Matrix3d K_i, float time_step,
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
    Matrix3d I = Matrix3d::Identity();

    //Rotation matrices
    Matrix3d R = getRotationMatrix(q_current);
    Matrix3d Rt = R.transpose();

    Matrix3d R_d = getDesiredRotationMatrix(q_des);
    Matrix3d R_dt = R_d.transpose();


    //terms of the control equation
    //angular and angular velocity errors
    Vector3d err_ang = (1/(2*sqrt(1+trace(R_dt*R))))*invskew(R_dt*R - Rt*R_d);
    Vector3d err_omg = omega_current - (Rt*R_d*omega_des);

    //other terms
    Vector3d term3 = skew(Rt*R_d*omega_des)*I*(Rt*R_d*omega_des);
    Vector3d term4 = I*(Rt*R_d*omegaD_des);

    //Integral part
    *err_int = (time_step * err_ang) + (*err_int);
    ///x
    if( err_int->x() > wind_up_limit_up ) err_int->x() = wind_up_limit_up;
    else if( err_int->x() < wind_up_limit_low) err_int->x() = wind_up_limit_low;
    ///y
    if( err_int->y() > wind_up_limit_up ) err_int->y() = wind_up_limit_up;
    else if( err_int->y() < wind_up_limit_low) err_int->y() = wind_up_limit_low;
    ///z
    if( err_int->z() > wind_up_limit_up ) err_int->z() = wind_up_limit_up;
    else if( err_int->z() < wind_up_limit_low) err_int->z() = wind_up_limit_low;

    //controller equation
    Vector3d M = -K_i*(*err_int) + -K_ang*err_ang - K_omg*err_omg + term3 +term4;

    return M;
}

Vector3d attitudeControllerDebug(RowVector4d q_current, RowVector4d q_des, Vector3d omega_current, Vector3d omega_des, Vector3d omegaD_des){
    //K Constants, may have to be tuned
    ///Proportional gain                               axis:  x   y   z
    Eigen::Matrix3d K_ang = Eigen::DiagonalMatrix<double, 3>(0.1,0.1,0.1);

    ///Derivate gain                                    axis:   x      y      z
    Eigen::Matrix3d K_omg = Eigen::DiagonalMatrix<double, 3>(0.1510,0.1510,0.1510);

    //The inertia matrix is considered diagonar here, with I = [I(1,1), 0 , 0; 0, I(2,2), 0 ; 0, 0, I(3,3)];
    Matrix3d I = Matrix3d::Identity();

    //Rotation matrices
    Matrix3d R = getRotationMatrix(q_current);
    Matrix3d Rt = R.transpose();

    Matrix3d R_d = getDesiredRotationMatrix(q_des);
    Matrix3d R_dt = R_d.transpose();


    //terms of the control equation
    //angular and angular velocity errors
    Vector3d err_ang = (1/(2*sqrt(1+trace(R_dt*R))))*invskew(R_dt*R - Rt*R_d);
    Vector3d err_omg = omega_current - (Rt*R_d*omega_des);

    //other terms
    Vector3d term3 = skew(Rt*R_d*omega_des)*I*(Rt*R_d*omega_des);
    Vector3d term4 = I*(Rt*R_d*omegaD_des);

    //controller equation
    Vector3d M = -K_ang*err_ang - K_omg*err_omg + term3 +term4;


    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    hal.console->printf("#################DEBUG#################\n");
    hal.console->printf("R =\n");
    hal.console->printf("%.10e  %.10e  %.10e\n", R(0,0), R(0,1), R(0,2) );
    hal.console->printf("%.10e  %.10e  %.10e\n", R(1,0), R(1,1), R(1,2) );
    hal.console->printf("%.10e  %.10e  %.10e\n", R(2,0), R(2,1), R(2,2) );
    hal.console->printf("\n\n");
    hal.console->printf("Rt =\n");
    hal.console->printf("%.10e  %.10e  %.10e\n", Rt(0,0), Rt(0,1), Rt(0,2) );
    hal.console->printf("%.10e  %.10e  %.10e\n", Rt(1,0), Rt(1,1), Rt(1,2) );
    hal.console->printf("%.10e  %.10e  %.10e\n", Rt(2,0), Rt(2,1), Rt(2,2) );
    hal.console->printf("\n\n");
    hal.console->printf("R_d =\n");
    hal.console->printf("%.10e  %.10e  %.10e\n", R_d(0,0), R_d(0,1), R_d(0,2) );
    hal.console->printf("%.10e  %.10e  %.10e\n", R_d(1,0), R_d(1,1), R_d(1,2) );
    hal.console->printf("%.10e  %.10e  %.10e\n", R_d(2,0), R_d(2,1), R_d(2,2) );
    hal.console->printf("\n\n");
    hal.console->printf("R_dt =\n");
    hal.console->printf("%.10e  %.10e  %.10e\n", R_dt(0,0), R_dt(0,1), R_dt(0,2) );
    hal.console->printf("%.10e  %.10e  %.10e\n", R_dt(1,0), R_dt(1,1), R_dt(1,2) );
    hal.console->printf("%.10e  %.10e  %.10e\n", R_dt(2,0), R_dt(2,1), R_dt(2,2) );
    hal.console->printf("\n\n");
    hal.console->printf("err_ang = [%.10e  %.10e  %.10e]'\n", err_ang(0), err_ang(1), err_ang(2) );
    hal.console->printf("err_omg = [%.10e  %.10e  %.10e]'\n", err_omg(0), err_omg(1), err_omg(2) );
    hal.console->printf("term3 = [%.10e  %.10e  %.10e]'\n", term3(0), term3(1), term3(2) );
    hal.console->printf("term4 = [%.10e  %.10e  %.10e]'\n", term4(0), term4(1), term4(2) );
    hal.console->printf("\n\n");

    return M;
}