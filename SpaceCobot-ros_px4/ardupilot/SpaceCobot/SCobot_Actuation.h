//
// Created by Filipe Rosa on 18/07/2017.
//
//Functions to translate Space CoBot actuation vector into PWM values to deliver to the motors
//
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#ifndef ARDUPILOT_SCOBOT_ACTUATION_H
#define ARDUPILOT_SCOBOT_ACTUATION_H
#define LAMBDA 9.43e-9

/**
 *
 * @param rpm - desired rotations per minute
 * @return pwm wich drives the motor at the givem rpm
 */
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

#endif //ARDUPILOT_SCOBOT_ACTUATION_H
