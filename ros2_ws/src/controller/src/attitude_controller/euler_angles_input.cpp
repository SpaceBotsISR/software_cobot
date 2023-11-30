// RCControl.cpp
#include "attitude_controller/euler_angles_input.hpp"

// Constructor
RCControl::RCControl(unsigned int roll_channel, unsigned int pitch_channel, unsigned int yaw_channel)
    : roll_channel(roll_channel), pitch_channel(pitch_channel), yaw_channel(yaw_channel),
      roll_angle(0.0), pitch_angle(0.0), yaw_angle(0.0) {}

// Method to update angles from RC message
void RCControl::update(const std::shared_ptr<mavros_msgs::msg::RCIn> rc_in)
{
    if (rc_in->channels.size() > std::max({roll_channel, pitch_channel, yaw_channel}))
    {
        set_roll(rc_in->channels[roll_channel]);
        set_pitch(rc_in->channels[pitch_channel]);
        set_yaw(rc_in->channels[yaw_channel]);
    }
    else
    {
        std::cerr << "Invalid RCIn message format." << std::endl;
    }
}

// Setter methods
void RCControl::set_roll(double roll)
{
    roll_angle = roll; // Update the auxiliary variable
}

void RCControl::set_pitch(double pitch)
{
    pitch_angle = pitch; // Update the auxiliary variable
}

void RCControl::set_yaw(double yaw)
{
    yaw_angle = yaw; // Update the auxiliary variable
}

// Getter methods
unsigned int RCControl::get_roll_channel() const
{
    return roll_channel;
}

unsigned int RCControl::get_pitch_channel() const
{
    return pitch_channel;
}

unsigned int RCControl::get_yaw_channel() const
{
    return yaw_channel;
}

double RCControl::get_roll() const
{
    return roll_angle;
}

double RCControl::get_pitch() const
{
    return pitch_angle;
}

double RCControl::get_yaw() const
{
    return yaw_angle;
}

// Method to print angles
void RCControl::print_angles() const
{
    std::cout << "Roll: " << get_roll() << ", Pitch: " << get_pitch() << ", Yaw: " << get_yaw() << std::endl;
}