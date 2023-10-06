#include <iostream>
#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>

//including files for actuator control by ROS
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/RCIn.h>

#include "space_cobot_interface/pwm_values.h"
#include <tf/transform_broadcaster.h>


#include "AMatrix.h"
#include "Actuation.h"


#define NUM_MOTORS 6
#define pi 3.1415926535897932384


///Defining the flight modes here
//TODO(ALG): The different flight_modes should be #1 BOARD, #2 MANUAL_POSITION, #3 MANUAL_ATTITUDE, #4
enum FlightModes { BOARD, MANUAL_ATTITUDE_INCREMENTAL, OPEN_LOOP_TORQUE, OPEN_LOOP_FORCE};  //1099 is BOARD, 1500 is MANUAL_INCREMENTAL and 1901 is OPEN_LOOP_TORQUE,



using namespace std;

///Inputs from the transmitter-- default neutral value being 1499.. These are used by manual_control, torque_control, force_control
int roll_input=1499, pitch_input=1499, yaw_input=1499;

/// Declaring flight_mode switch for enabling manual control
int pwm_flight_mode= 1099;

///Declaring value for kill switch to set the value of err_int and des_orientation_rpy when kill is deployed such that it doesnt bounce back when kill is switched off
///Kill-OFF is 1099
int kill_switch=1099;

Eigen::Quaterniond orientation_current(1, 0, 0, 0); /// =identity

///The Eigen::Quaternion is (w, x, y, z)
Eigen::Quaterniond orientation_desired(1, 0, 0, 0);
vector<double> orientation_desired_rpy(3), angular_velocity_desired(3), position_desired(3), velocity_desired(3);


///defining current matrix for rc control
Eigen::Matrix3d rc_Rd;
///rc flag for locking the current orientation for manual control when first switched.
int rc_flag = 0;


/// Declaring MAVROS State
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void rc_controlCallback(const mavros_msgs::RCIn::ConstPtr &msg) {
    pwm_flight_mode = msg->channels[5];


    roll_input= msg->channels[0];
    pitch_input= msg->channels[1];
    yaw_input= msg->channels[3];

    kill_switch= msg->channels[4];

}

void manual_control(){
    double rc_x_change = ((roll_input- 1497)*0.00225*(pi/180));
    double rc_y_change = ((pitch_input-1498)*0.00225)*(pi/180);
    double rc_z_change = ((yaw_input - 1499)*0.00225)*(pi/180);
    //cout<<"CHEcKPOINT 1"<<endl;

    Eigen::Matrix3d Ri;
    ///cv version of Ri and angle change for rodriguez formula
    cv::Mat cvRI(3, 3, CV_32FC1);
    cv::Mat cvChange(1, 3, CV_32FC1);
    //cout<<"CHECKPOINT 2 "<<endl;

    ///Assigning angle change values to cvChange for rodriguez formula
    cvChange.ptr< float >(0)[0]= rc_x_change;
    cvChange.ptr< float >(0)[1]= rc_y_change;
    cvChange.ptr< float >(0)[2]= rc_z_change;

    //cout<<"The CV matrix that goes into the Rodriguez formula"<<endl;
    //cout<<cvChange<<endl;

    //cout<<"CHECKPOINT 2.5"<<endl;

    ///The main Rodriguez Formula
    cv::Rodrigues(cvChange, cvRI);

    //cout<<"CHECKPOINT 3"<<endl;
    //cout<<"3X3 matrix after the Rodriguez Formula"<<endl;
    //cout<<cvRI<<endl;

    ///Need to convert cvRI to Ri i.e the eigen matrix for easier multiplication
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++){
            Ri(i,j)= cvRI.ptr< float >(i)[j];
        }
    //cout<<"CHECKPOINT 4"<<endl;
    //cout<<"This is Ri-- the change matrix in Eigen form-----should be same as the 3x3 matrix after the rodriguez formula: "<<endl;
    //cout<<Ri<<endl;

    if(rc_flag != 1){  ///rc_flag more like rc_lock for orientation----- so that the rc_Rd is given the value of current orientation only when its flicked to manual mode-- after that it takes the value of the ri*rd not current orientation
        tf::Quaternion q_temp_cur;
        q_temp_cur.setX(orientation_current.x());
        q_temp_cur.setY(orientation_current.y());
        q_temp_cur.setZ(orientation_current.z());
        q_temp_cur.setW(orientation_current.w());

        tf::Matrix3x3 rc33(q_temp_cur); // transform the current quat to 3X3 --> use a tf function to change this

        //cout<<"Locked Initial Current Orientation: "<<endl;
        //cout<<rc33.getRow(0)<<endl;
        //cout<<rc33.getRow(1)<<endl;
        //cout<<rc33.getRow(2)<<endl;

        ///changing the current orientation to 3x3 matrix Eigen
        rc_Rd(0,0)= rc33.getRow(0).x();
        rc_Rd(0,1)= rc33.getRow(0).y();
        rc_Rd(0,2)= rc33.getRow(0).z();
        rc_Rd(1,0)= rc33.getRow(1).x();
        rc_Rd(1,1)= rc33.getRow(1).y();
        rc_Rd(1,2)= rc33.getRow(1).z();
        rc_Rd(2,0)= rc33.getRow(2).x();
        rc_Rd(2,1)= rc33.getRow(2).y();
        rc_Rd(2,2)= rc33.getRow(2).z();

    }
    //cout<<"This is either the current orientation (when flicked for the first time) or changed orientation after applying orientations "<<endl;
    //cout<<rc_Rd<<endl;

    //cout<<"CHECKPOINT 5"<<endl;

    rc_flag=1;
    ///This is causing all the problems -----> continuously multiplying and going to zero....
    rc_Rd=Ri*rc_Rd;
    //cout<<"The des_orientation 3x3 matrix---currently has no changes and IS EQUAL TO Ri"<<endl;
    //cout<<rc_Rd<<endl;

    ///Now convert the New rotation into orientation desired quaternion
    //tf::Quaternion quatdes();

    Eigen::Quaterniond quatdes(rc_Rd);
    quatdes.normalize();
    orientation_desired.x()= quatdes.x();
    orientation_desired.y()= quatdes.y();
    orientation_desired.z()= quatdes.z();
    orientation_desired.w()= quatdes.w();

    //cout<<"Desired Orientation after changing from rc_Rd: "<<orientation_des.x()<<" "<<orientation_des.y()<<" "<<orientation_des.z()<<" "<<orientation_des.w()<<endl;


}




///GLOBAL VARIABLE FOR THE PWM VALUES .....................................
//TODO(ALG): Check the safety of this thang!!!

vector<double> actuator_pwm_values(6);

void pwmValuesCallback(const space_cobot_interface::pwm_values::ConstPtr& msg ){
    for(int i = 0; i<6; i++)
        actuator_pwm_values[i] = msg->PWM[i];

}
void setFlightMode(FlightModes& flight_mode){




    if(pwm_flight_mode == 1099)
        flight_mode = BOARD;

    if(pwm_flight_mode == 1500)
        flight_mode = MANUAL_ATTITUDE_INCREMENTAL;

    if(pwm_flight_mode == 1901)
        flight_mode = OPEN_LOOP_TORQUE;


}

void torque_control(Eigen::Vector3d &M) {

    double x_change= (roll_input - 1499)*0.0000025;
    double y_change= (pitch_input- 1499)*0.0000025;
    double z_change= (yaw_input- 1499)*0.0000025;

    M[0]+=x_change;
    M[1]+=y_change;
    M[2]+=z_change;

}

void quats2rpy(Eigen::Quaterniond temp_orientation_desired, vector<double>& temp_desired_orientation_rpy ){
    ///Converting the orientations back to quaternions
    tf::Quaternion q_temp_des, q_temp_cur;

    q_temp_des.setX(temp_orientation_desired.x());
    q_temp_des.setY(temp_orientation_desired.y());
    q_temp_des.setZ(temp_orientation_desired.z());
    q_temp_des.setW(temp_orientation_desired.w());


    tf::Matrix3x3 mat2(q_temp_des);
    tfScalar roll_des=0, pitch_des=0, yaw_des=0;
    mat2.getEulerYPR(yaw_des, pitch_des, roll_des);

    ///The yaw, pitch, roll we get from the above function is in radians, so we need to convert it to degrees before putting it in the ros message
    temp_desired_orientation_rpy[0]= roll_des*(180/pi);
    temp_desired_orientation_rpy[1]= pitch_des*(180/pi);
    temp_desired_orientation_rpy[2]= yaw_des*(180/pi);


}


///bool values so that notification messages are printed only once
bool isaid_board = false;
bool isaid_manual = false;
bool isaid_openloop = false;

int main(int argc, char** argv) {

    ros::init(argc, argv, "space_cobot_interface");
    ros::NodeHandle nh;

    //Publishing and Subscribing

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher actuator_controls_pub = nh.advertise<mavros_msgs::ActuatorControl>
            ("/mavros/actuator_control", 1000);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber fmode= nh.subscribe("/mavros/rc/in", 1000, rc_controlCallback);

    ros::Subscriber pwm_values = nh.subscribe("important_values", 1000, pwmValuesCallback);

    ros::Rate rate(20);

    /// wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        cout<<"Connecting to FCU....."<<endl;
        rate.sleep();
    }
    if(current_state.connected)
        cout<<"Connection to FCU established!"<<endl;
    else {
        cout<<"Connection to FCU FAILED!"<<endl;
        return 0;
    }
    ///send a few setpoints before starting in order to arm it
    //%%%%%%%%%%%%%%%%%%%%%%% --------NEED TO CHECK THIS to make sure if this doesnt cause any sudden motion or tells the cobot to go to the desired setpoint... %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //UPDATE on the above comment: NO such thing was observed but still keeping the comment if any future issues arise

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


    ///Initializing the rosparams
    nh.param<vector<double> >("desired_orientation_rpy", orientation_desired_rpy);
    nh.param<vector<double> >("desired_angular_velocity", angular_velocity_desired);
    ///commenting position and velocity rosparam for now for safety
    nh.param<vector<double>>("desired_position", position_desired, {0.0, 0.0, 0.0});
    nh.param<vector<double>>("desired_velocity", velocity_desired, {0.0, 0.0, 0.0});





    while(ros::ok()) {


        ///Getting all the rosparams
        ///getting all the rosparams
        nh.getParam("desired_position", position_desired);
        nh.getParam("desired_velocity", velocity_desired);
        nh.getParam("desired_orientation_rpy", orientation_desired_rpy);
        nh.getParam("desired_angular_velocity", angular_velocity_desired);


        //TODO(ALG): Looks alright but maybe try to simplify it.
        /// Making sure the mode set is always OFFBOARD and its always armed
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        mavros_msgs::ActuatorControl actuator_control_msg;

        //TODO(ALG): The mode controls here: Depending on which switch is flicked change the mode and have a different file for each mode to enhane code readability

        /**********************************SETTING FLIGHT MODES**************************************************
        *********************************************************************************************************/
        FlightModes  flight_mode;
        ///SETTING THE FLIGHT MODE
        setFlightMode(flight_mode);

        if(flight_mode == FlightModes::BOARD){
            //get motor_actuation_values from the the pwm_values rosmessage of the space_cobot_controller rosnode
            if(!isaid_board){
                cout<<"Board Mode"<<endl;
                isaid_board =true;
                isaid_manual = false;
                isaid_openloop = false;
            }


        }

        else if(flight_mode == FlightModes::MANUAL_ATTITUDE_INCREMENTAL){
            //set the desired rpy values from the transmitter and then get the motor actuation values from the rosmessage of the space_cobot_controller rosnode
            manual_control();
            if(!isaid_manual){
                cout<<"Manual Mode"<<endl;
                isaid_manual = true;
                isaid_board = false;
                isaid_openloop = false;

            }

        }

        ///Setting the desired orientation rosparam

        quats2rpy(orientation_desired, orientation_desired_rpy);
        nh.setParam("desired_orientation_rpy", orientation_desired_rpy);



        if(flight_mode == FlightModes::OPEN_LOOP_TORQUE){
            //set the torque values and get the actuation vector from the in-program Actuation.cpp and set the actutation vector from it
            //Setting the force vector to zero in OPEN_LOOP_TORQUE
            Eigen::Vector3d force(0,0,0);
            Eigen::Vector3d moment(0,0,0);

            /// setting value of torque from transmitter
            torque_control(moment);


            Eigen::VectorXd u = getActuationVector(force, moment);
            double RPM[NUM_MOTORS];

            /// Limiting PWM values to the range 1350-1650
            for(int j=0; j<NUM_MOTORS; j++)
            {
                double rpm = thrustToRPM(u(j));
                double pwm = convertToPWM(rpm);

                /// safe range of PWM-- limiting the values of PWM
                if(pwm<1350)
                    pwm=1350;
                else if (pwm> 1615)
                    pwm= 1615;

                /// Saving these values to the array
                RPM[j]= rpm;
                actuator_pwm_values[j]= pwm;

            }

            if(!isaid_openloop){
                cout<<"Open Loop: Torque"<<endl;
                isaid_openloop = true;
                isaid_board =false;
                isaid_manual = false;
            }



        }




        //Send to the controller
        actuator_control_msg.header.stamp = ros::Time::now();
        actuator_control_msg.group_mix = 0;

        /// running a loop to send the PWM values to motor
        for(int i=0; i< NUM_MOTORS; i++){
            /// Converting it to the system pixhawk understands---as 0 in input gives 1500 in output.
            actuator_control_msg.controls[i] = (actuator_pwm_values[i] - 1500)/500;
        }

        ///publishing the actuator pwms (assigned above) to the topic "mavros_msgs::ActuatorControl" controls the motors
        actuator_controls_pub.publish(actuator_control_msg);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
