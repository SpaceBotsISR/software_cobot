// RCControl.hpp
#ifndef __RC_CONTROL_HPP__
#define __RC_CONTROL_HPP__
// RCControl.hpp
#pragma once

#include <iostream>
#include <memory>
#include <algorithm>
#include <mavros_msgs/msg/rc_in.hpp>

class RCControl
{
public:
    RCControl(unsigned int roll_channel, unsigned int pitch_channel, unsigned int yaw_channel);

    void update(const std::shared_ptr<mavros_msgs::msg::RCIn> rc_in);

    void set_roll(double roll);
    void set_pitch(double pitch);
    void set_yaw(double yaw);

    unsigned int get_roll_channel() const;
    unsigned int get_pitch_channel() const;
    unsigned int get_yaw_channel() const;

    double get_roll() const;
    double get_pitch() const;
    double get_yaw() const;

    void print_angles() const;

private:
    unsigned int roll_channel;
    unsigned int pitch_channel;
    unsigned int yaw_channel;

    double roll_angle;
    double pitch_angle;
    double yaw_angle;
};

#endif