//
// Created by socrob on 07-02-2019.
//

#ifndef SPACE_COBOT_CONTROLLER_ACTUATION_H
#define SPACE_COBOT_CONTROLLER_ACTUATION_H

#define LAMBDA 9.43e-9

/**
 *
 * @param rpm - desired rotations per minute
 * @return pwm which drives the motor at the givem rpm
 *
 */
//typedefs defined by me... make sure these are correct...
typedef unsigned short int uint16_t;
typedef short int int16_t;
typedef float float_t;

uint16_t convertToPWM(int16_t rpm);

/**
 *
 * @param ui - position of the actuation vector, comes if the form: +/-(revolutions/s)^2
 * @return RPM correspondent to the given u
 */
int16_t actuationToRPM(float_t ui);

/**
 *
 * Converts thrust into RPM
 *
 * LAMBDA = relation between force and RPM according to the motor data about HQ Propeller
 * @param ui - thrust of the motor in Newtons
 * @return rpm
 */
float_t thrustToRPM(float_t ui);

#endif //SPACE_COBOT_CONTROLLER_ACTUATION_H
