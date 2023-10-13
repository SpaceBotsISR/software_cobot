//
// Created by socrob on 07-02-2019.
//

#ifndef SPACE_COBOT_CONTROLLER_POSITIONCONTROL_H
#define SPACE_COBOT_CONTROLLER_POSITIONCONTROL_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

// using namespace Eigen;

/// Proportional position gain                         axis:  x  y  z
const Eigen::Matrix3d KXp = Eigen::DiagonalMatrix<double, 3>(20, 20, 20);

/// Derivate position gain                             axis:  x  y  z
const Eigen::Matrix3d KXd = Eigen::DiagonalMatrix<double, 3>(22, 22, 22);

/**
 * Calculates the force to apply in the vehicle
 *
 * @param p_curr - current position 3x1
 * @param p_des - desired position 3x1
 * @param v_curr - current linear velocity 3x1
 * @param v_des - desired linear velocity 3x1
 * @param rotation_matrix - extracted from current attitude quaternion 3x3
 * @return F - force to apply in the vehicle 3x1
 */
Eigen::Vector3d positionController(Eigen::Vector3d p_curr, Eigen::Vector3d p_des, Eigen::Vector3d v_curr, Eigen::Vector3d v_des, Eigen::Matrix3d rotation_matrix);

/**
 * Same as previous function but receives PD gains as arguments
 * @param KXp - proportional gain
 * @param KXd - derivative gain
 * @return
 */
Eigen::Vector3d positionController(Eigen::Vector3d p_curr, Eigen::Vector3d p_des, Eigen::Vector3d v_curr, Eigen::Vector3d v_des, Eigen::Matrix3d rotation_matrix, Eigen::Matrix3d KXp, Eigen::Matrix3d KXd);

Eigen::Vector3d positionController(std::vector<double> p_curr, std::vector<double> p_des, std::vector<double> v_curr, std::vector<double> v_des, Eigen::Matrix3d rotation_matrix, Eigen::Matrix3d KXp, Eigen::Matrix3d KXd);

#endif // SPACE_COBOT_CONTROLLER_POSITIONCONTROL_H
