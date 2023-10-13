//
// Created by socrob on 07-02-2019.
//

#ifndef SPACE_COBOT_CONTROLLER_AMATRIX_H
#define SPACE_COBOT_CONTROLLER_AMATRIX_H

#include <stdio.h>
#include <eigen3/Eigen/Dense>
//#include "AP_Math/AP_Math.h"        // ArduPilot Mega Vector/Matrix math Library

/**
 Function which converts robot Force and Torque into the amount of rotation of each motor

 @param force 3x1 Force vector
 @param torque 3x1 Torque vector
 @return U - 6x1 actuation vector where sqrt(|ui|)*sign(ui) corresponds to rev/s (revolutions per second)
 */
Eigen::VectorXd getActuationVector(Eigen::Vector3d force, Eigen::Vector3d torque);

#endif //SPACE_COBOT_CONTROLLER_AMATRIX_H
