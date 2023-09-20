//
// Created by socrob on 09-02-2019.
//



#include "PositionControl.h"


Eigen::Vector3d positionController(Eigen::Vector3d p_curr, Eigen::Vector3d p_des, Eigen::Vector3d v_curr, Eigen::Vector3d v_des, Eigen::Matrix3d rotation_matrix){

    ///Proportional position gain                         axis:  x  y  z
    Eigen::Matrix3d KXp = Eigen::DiagonalMatrix<double, 3>(20,20,20);

    ///Derivate position gain                             axis:  x  y  z
    Eigen::Matrix3d KXd = Eigen::DiagonalMatrix<double, 3>(22,22,22);

    Eigen::Vector3d posError = p_des - p_curr;
    Eigen::Vector3d velError = v_des - v_curr;

    Eigen::Vector3d force = rotation_matrix.transpose()*(KXp*posError+KXd*velError);

    return force;
}

Eigen::Vector3d positionController(Eigen::Vector3d p_curr, Eigen::Vector3d p_des, Eigen::Vector3d v_curr, Eigen::Vector3d v_des, Eigen::Matrix3d rotation_matrix, Eigen::Matrix3d KXp, Eigen::Matrix3d KXd){

    Eigen::Vector3d posError = p_des - p_curr;
    Eigen::Vector3d velError = v_des - v_curr;

    Eigen::Vector3d force = rotation_matrix.transpose()*(KXp*posError+KXd*velError);

    return force;
}

//Currently using this one
Eigen::Vector3d positionController(std::vector<double> p_curr, std::vector<double> p_des, std::vector<double>  v_curr, std::vector<double>  v_des, Eigen::Matrix3d rotation_matrix, Eigen::Matrix3d KXp, Eigen::Matrix3d KXd){


    std::vector<double> position_error(3), velocity_error(3);
    for(int i=0; i<3; i++){

        position_error[i] = p_des[i] - p_curr[i];
        velocity_error[i] = v_des[i] - v_curr[i];

    }

    Eigen::Vector3d posError(position_error[0], position_error[1], position_error[2]), velError(velocity_error[0], velocity_error[1], velocity_error[2]);
    Eigen::Vector3d force = rotation_matrix.transpose()*(KXp*posError+KXd*velError);

    return force;
}
