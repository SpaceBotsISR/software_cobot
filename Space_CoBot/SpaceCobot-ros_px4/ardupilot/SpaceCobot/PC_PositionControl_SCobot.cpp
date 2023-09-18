//
// Created by Filipe Rosa on 26/02/2018.
//

#include "PC_PositionControl_SCobot.h"


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

Eigen::Vector3d positionController(float mass, Eigen::Vector3d p_curr, Eigen::Vector3d p_des, Eigen::Vector3d v_curr, Eigen::Vector3d v_des, Eigen::Matrix3d rotation_matrix, Eigen::Matrix3d KXp, Eigen::Matrix3d KXd){

    Eigen::Vector3d posError = p_des - p_curr;
    Eigen::Vector3d velError = v_des - v_curr;

    Eigen::Vector3d force = mass*rotation_matrix.transpose()*(KXp*posError+KXd*velError);

    return force;
}