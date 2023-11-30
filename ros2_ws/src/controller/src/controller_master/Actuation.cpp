//
// Created by socrob on 09-02-2019.
//

#include "controller_master/Actuation.hpp"
#include <cmath>

uint16_t convertToPWM(int16_t rpm)
{
    uint16_t mypwm;
    rpm /= 4 / 3; // 4/3 = (14.8 / 11.1) CONVERT FROM RPM ON 4S TO 3S
    if (rpm > 0)
    {
        // mypwm = (uint16_t) (0.000000214802726 * pow(rpm, 2) + 0.0121696724019783 * rpm + 1499.25490902604); //quadratic regression curve
        mypwm = (uint16_t)-0.0000000000110106978797619 * pow(rpm, 3) + 0.000000448602698785971 * pow(rpm, 2) + 0.0108341387639997 * rpm + 1500.87746062623; // cubic regression curve
    }
    else
    {
        // mypwm = (uint16_t) (-0.000000193881142 * pow(rpm, 2) + 0.0124845157903363 * rpm + 1467.04520972071); //quadratic regression curve
        mypwm = (uint16_t)-0.00000000000645913910427410 * pow(rpm, 3) - 0.000000338986537473653 * pow(rpm, 2) + 0.0115794605304886 * rpm + 1465.70343578529; // cubic regression curve
    }

    // if(mypwm > 1250 && mypwm < 1750) return mypwm;
    // return 1500;
    return mypwm;
}

int16_t actuationToRPM(float_t ui)
{
    // ui data comes in revolutions per second squared
    if (ui < 0)
        return roundf(-sqrtf(fabsf(ui)) * 60); // negative RPM
    return roundf(sqrtf(fabsf(ui)) * 60);      // positive RPM
}

// made some changes here -- changed the value of abs to become absolute values
float_t thrustToRPM(float_t ui)
{
    if (ui < 0)
        return (float_t)-sqrt(std::abs(ui) / LAMBDA);
    return (float_t)sqrt(std::abs(ui) / LAMBDA);
}