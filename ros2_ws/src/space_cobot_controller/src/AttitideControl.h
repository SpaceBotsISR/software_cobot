//
// Created by socrob on 07-02-2019.
//

#ifndef SPACE_COBOT_CONTROLLER_ATTITIDECONTROL_H
#define SPACE_COBOT_CONTROLLER_ATTITIDECONTROL_H

#include <eigen3/Eigen/Dense>

/**
 * Get rotation matrix from quaternion
 *
 * @param q_current - current attitude in quaternion form
 * @return R - Rotation matrix
 */
Eigen::Matrix3d getRotationMatrix(Eigen::RowVector4d q_current);

/**
 *
 * @param q_des
 * @return
 */
Eigen::Matrix3d getDesiredRotationMatrix(Eigen::RowVector4d q_des);

/**
 *
 * @param mat
 * @return
 */
Eigen::Vector3d invskew(Eigen::Matrix3d mat);

/**
 *
 * @param vec
 * @return
 */
Eigen::Matrix3d skew(Eigen::Vector3d vec);

/**
 *
 * @param mat
 * @return
 */
float trace(Eigen::Matrix3d mat);

/**
 * Calculate attitude controller equation
 *
 * @param q_current - current attitude quaternion
 * @param q_des - desired attitude
 * @param omega_current - current angular velocity
 * @param omega_des - desired angular velocity
 * @param omegaD_des - desired angular acceleration
 * @return M - Vector with torques to be aplied
 */
Eigen::Vector3d attitudeController(Eigen::RowVector4d q_current, Eigen::RowVector4d q_des, Eigen::Vector3d omega_current, Eigen::Vector3d omega_des, Eigen::Vector3d omegaD_des);

/**
 * Same as previous function but receives PDs as arguments
 * @param K_ang - proportional gain
 * @param K_omg - derivative gain
 * @return
 */
Eigen::Vector3d attitudeController(Eigen::RowVector4d q_current, Eigen::RowVector4d q_des, Eigen::Vector3d omega_current,
                                   Eigen::Vector3d omega_des, Eigen::Vector3d omegaD_des, Eigen::Matrix3d K_ang, Eigen::Matrix3d K_omg,
                                   Eigen::Vector3d &err_int, Eigen::Matrix3d K_i, float time_step,
                                   float wind_up_limit_up, float wind_up_limit_low);
/**
 * Calculate attitude controller equation and print mid calcs for debug
 *
 * @param q_current - current attitude quaternion
 * @param q_des - desired attitude
 * @param omega_current - current angular velocity
 * @param omega_des - desired angular velocity
 * @param omegaD_des - desired angular acceleration
 * @return M - Vector with torques to be aplied
 */
Eigen::Vector3d attitudeControllerDebug(Eigen::RowVector4d q_current, Eigen::RowVector4d q_des, Eigen::Vector3d omega_current, Eigen::Vector3d omega_des, Eigen::Vector3d omegaD_des);

#endif // SPACE_COBOT_CONTROLLER_ATTITIDECONTROL_H
