//
//  A_Matrix.hpp
//  RPMConversion
//
//  Created by Filipe Rosa on 29/06/2017.
//  Copyright © 2017 Space CoBot. All rights reserved.
//

#ifndef AMatrix_hpp
#define AMatrix_hpp

#include <stdio.h>
#include <Eigen/Dense>
//#include "AP_Math/AP_Math.h"        // ArduPilot Mega Vector/Matrix math Library

/**
 Function chich converts robot Force and Torque into the amount of rotation of each motor

 @param force 3x1 Force vector
 @param torque 3x1 Torque vector
 @return U - 6x1 actuation vector where sqrt(|ui|)*sign(ui) corresponds to rev/s (revolutions per second)
 */
Eigen::VectorXf getActuationVector(Eigen::Vector3f force, Eigen::Vector3f torque);

#endif /* AMatrix_hpp */
