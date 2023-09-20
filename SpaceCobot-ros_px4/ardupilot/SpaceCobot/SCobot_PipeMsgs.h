//
// Created by Filipe Rosa on 27/02/2018.
//

#ifndef ARDUPILOT_SCOBOT_PIPEMSGS_H
#define ARDUPILOT_SCOBOT_PIPEMSGS_H

struct AttitudeMsg{
    int id;
    //int time;
    struct Quaternion{
        float w;
        float x;
        float y;
        float z;
    }quaternion;
};

struct ActuationMsg{
    int id;
    struct ActuationVector{
        float u1;
        float u2;
        float u3;
        float u4;
        float u5;
        float u6;
    }U;
};

struct StringMsg{
    int id;
    char topic[40];
    char message[512];
};

struct MotionMsg{
    int id;
    struct Position{
        float x;
        float y;
        float z;
    }pos;
    struct Velocity{
        float x;
        float y;
        float z;
    }vel;
};

struct PoseMsg{
    int id;
    struct Point{
        float x;
        float y;
        float z;
    }position;
    struct Quaternion{
        float w;
        float x;
        float y;
        float z;
    }att_quaternion;
};

struct VelocityMsg{
    int id;
    struct linear{
        float x;
        float y;
        float z;
    }v;
    struct angular{
        float x;
        float y;
        float z;
    }w;
};

struct ParamMsg{
    char name[40];
//    int type;
    float f_value;
//    double d_value;
//    int i_value;
//    bool b_value;
//    char s_value[40];
};

#endif //ARDUPILOT_SCOBOT_PIPEMSGS_H
