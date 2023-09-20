//
// Created by socrob on 05-03-2019.
//

#include <iostream>
#include "ros/ros.h"

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "Actuation.h"
#include "space_cobot_interface/pwm_values.h"
#include "mocap_interface/mocap_msg.h"


#define POSITION 0

/****************************************************Assumptions******************************************************************
#1  acadaVariables.x[0] = position.x  .................................................. acadoVariables.x[3] = velocity.x
    acadoVariables.x[6] = orientation.x, acadoVariables.x[9] = orientation.w............acadoVariables.x[10] = angular_velocity.x

#2 The reference values are given to the OnlineData. i.e. acadoVariables.od[]  The acadoVariables.y and acadoVariables.yN is kept at 0.

Contradictory so check.
#3 The acadoVariables.od[6] = orientation_des.w and acadoVariables.od[9] = orientation_des.z
**********************************************************************************************************************************/



/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */ ////Irrelevant to this one
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */  ///Doesnt matter when using a while(ros::ok())
#define VERBOSE     1        /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

#define pi 3.1415926535897932384

//TODO(Master): add a callback function for getting angular velocity values



using namespace std;
bool current_pose_updated = false;

//Global variable for desired position, orientation and velocity and angular velocity
vector<double> orientation_desired_rpy(3), acceleration_desired(3), angular_velocity_desired(3), position_desired(3), velocity_desired(3);

//Desired orientation quaternion
Eigen::Quaterniond orientation_desired(1, 0, 0, 0);
Eigen::Quaterniond orientation_current(1,0,0,0);


/*void vrpnCallback(const geometry_msgs::PoseStamped::ConstPtr& msg ){
    acadoVariables.x0[0] = msg->pose.position.x;
    acadoVariables.x0[1] = msg->pose.position.y;
    acadoVariables.x0[2] = msg->pose.position.z;

    acadoVariables.x0[6] = msg->pose.orientation.w;
    acadoVariables.x0[7] = msg->pose.orientation.x;
    acadoVariables.x0[8] = msg->pose.orientation.y;
    acadoVariables.x0[9] = msg->pose.orientation.z;


    orientation_current.x() = msg->pose.orientation.x;
    orientation_current.y() = msg->pose.orientation.y;
    orientation_current.z() = msg->pose.orientation.z;
    orientation_current.w() = msg->pose.orientation.w;

    //cout<<"This should be good coz its the pose orientation: "<<acadoVariables.x[3]<<" "<<acadoVariables.x[4]<<" "<<acadoVariables.x[5]<<" "<<acadoVariables.x[6]<<endl;

    //cout<<"The acado variables should be changed"<<endl;

    current_pose_updated = true;


}*/
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

    ///Since the position is being obtained from the IMU, the position and velocity are being set to 0. With mocap the position will be set to the real pose.
    /*acadoVariables.x0[0] = 0.0;
    acadoVariables.x0[1] = 0.0;
    acadoVariables.x0[2] = 0.0;

    acadoVariables.x0[3] = 0.0;
    acadoVariables.x0[4] = 0.0;
    acadoVariables.x0[5] = 0.0;*/

    acadoVariables.x0[6] = msg->orientation.w;
    acadoVariables.x0[7] = msg->orientation.x;
    acadoVariables.x0[8] = msg->orientation.y;
    acadoVariables.x0[9] = msg->orientation.z;
	//cout<<"This should not print: "<<endl;

    acadoVariables.x0[10] = msg->angular_velocity.x;
    acadoVariables.x0[11] = msg->angular_velocity.y;
    acadoVariables.x0[12] = msg->angular_velocity.z;

    orientation_current.x() = msg->orientation.x;
    orientation_current.y() = msg->orientation.y;
    orientation_current.z() = msg->orientation.z;
    orientation_current.w() = msg->orientation.w;


    current_pose_updated = true;


}

void mocapCallback(const mocap_interface::mocap_msg &msg){

	/*acadoVariables.x0[6] = msg.pose.pose.orientation.w;
	acadoVariables.x0[7] = msg.pose.pose.orientation.x;
	acadoVariables.x0[8] = msg.pose.pose.orientation.y;
	acadoVariables.x0[9] = msg.pose.pose.orientation.z;

	orientation_current.x() = msg.pose.pose.orientation.x;
	 orientation_current.y() = msg.pose.pose.orientation.y;
	 orientation_current.z() = msg.pose.pose.orientation.z;
 	orientation_current.w() = msg.pose.pose.orientation.w;*/





    if(POSITION){
        acadoVariables.x0[0] = msg.pose.pose.position.x;
        acadoVariables.x0[1] = msg.pose.pose.position.y;
        acadoVariables.x0[2] = msg.pose.pose.position.z;

        acadoVariables.x0[3] = msg.velocity.twist.linear.x;
        acadoVariables.x0[4] = msg.velocity.twist.linear.y;
        acadoVariables.x0[5] = msg.velocity.twist.linear.z;

    }


    else{

        acadoVariables.x0[0] = 0;
        acadoVariables.x0[1] = 0;
        acadoVariables.x0[2] = 0;

        acadoVariables.x0[3] = 0;
        acadoVariables.x0[4] = 0;
        acadoVariables.x0[5] = 0;

    }

}








vector<double> toRad(vector<double> deg){
    vector<double> rad(3);

    for(int i=0; i<3; i++) {
        rad[i]= deg[i]*(pi/180);
    }
    return rad;



}

void getTargetValues_old(){ //old one

    vector<double> orientation_desired_rpy_rad(3);
    orientation_desired_rpy_rad = toRad(orientation_desired_rpy);

    /// converting the target orientation in rpy to quaternion and assigning it
    tf::Quaternion q_temp = tf::createQuaternionFromRPY(orientation_desired_rpy_rad[0], orientation_desired_rpy_rad[1], orientation_desired_rpy_rad[2]);
    q_temp.normalize();

    orientation_desired.x()= q_temp.x();
    orientation_desired.y()= q_temp.y();
    orientation_desired.z()= q_temp.z();
    orientation_desired.w()= q_temp.w();


    //assigning the desired values to the acadoVariables

    //TODO(ALG): repeat the mapping for whole of the horizon.
    //desried/reference values
    double repeating_vector[12];

    //Position
    repeating_vector[0] = position_desired[0];
    repeating_vector[1] = position_desired[1];
    repeating_vector[2] = position_desired[2];

    //Velocity  //TODO(Potential source of error).. giving velocity = 0.1m/s.
    repeating_vector[3] = 0.0;
    repeating_vector[4] = 0.0;
    repeating_vector[5] = 0.0;

    //Orientation //rpy
    repeating_vector[6] = orientation_desired_rpy[0];
    repeating_vector[7] = orientation_desired_rpy[1];
    repeating_vector[8] = orientation_desired_rpy[2];

    //Angular Velocity //TODO(Potential source of error).. giving angular velocity = 0.01m/s.
    repeating_vector[9] = 0.00;
    repeating_vector[10] = 0.00;
    repeating_vector[11] = 0.00;


    ///Now putting this repeating vector into the whole of acadVariables.y[]
    for(int i =0; i < N; i++ ){
        for(int j=0; j<NY; j++){
            acadoVariables.y[i*NY + j] = repeating_vector[j];
        }
    }

    for(int i = 0; i<NYN; i++) acadoVariables.yN[i] = repeating_vector[i];

	cout<<"#222"<<endl;

}

void getTargetValues(){


    vector<double> orientation_desired_rpy_rad(3);
    orientation_desired_rpy_rad = toRad(orientation_desired_rpy);

    /// converting the target orientation in rpy to quaternion and assigning it
    tf::Quaternion q_temp = tf::createQuaternionFromRPY(orientation_desired_rpy_rad[0], orientation_desired_rpy_rad[1], orientation_desired_rpy_rad[2]);
   

    q_temp.normalize();
    
    orientation_desired.x()= q_temp.x();
    orientation_desired.y()= q_temp.y();
    orientation_desired.z()= q_temp.z();
    orientation_desired.w()= q_temp.w();


    //assigning the desired values to the acadoVariables

    //repeatimg the mapping for whole of the horizon.
    //desried/reference values
    double repeating_vector[13];

    //Position
    repeating_vector[0] = position_desired[0];
    repeating_vector[1] = position_desired[1];
    repeating_vector[2] = position_desired[2];

    //Velocity  //TODO(Potential source of error).. giving velocity = 0.1m/s.
    repeating_vector[3] = 0.0;
    repeating_vector[4] = 0.0;
    repeating_vector[5] = 0.0;

    //Orientation //rpy
    repeating_vector[6] = orientation_desired.w();
    repeating_vector[7] = orientation_desired.x();
    repeating_vector[8] = orientation_desired.y();
    repeating_vector[9] = orientation_desired.z();

    //Angular Velocity //TODO(Potential source of error).. giving angular velocity = 0.01m/s.
    repeating_vector[10] = 0.00;
    repeating_vector[11] = 0.00;
    repeating_vector[12] = 0.00;


    ///Now putting this repeating vector into the whole of acadVariables.od[]
    for(int i =0; i < N+1; ++i ){
        for(int j=0; j<NOD; ++j){
            acadoVariables.od [i*NOD + j] = repeating_vector[j];
        
	}
    }



}

void getCurrentValues(){
    //values are assinged when callback function is called. (IMU or vrpn)

    if(current_pose_updated) cout<<"Updated curent pose"<<endl;

    current_pose_updated = false;

}

void get_cost_matrices() {

    int i,j;

    //TODO(ALG): ALG Edit. Need to tune the weights matrix.
    //TODO(ALG): Maybe use a Eigen matrix to individually tune the params.

    for (i = 0; i < NY; ++i) {
        for(j = 0; j < NY; ++j) {
            if(i==j)
                acadoVariables.W[i*NY+j] = 0.1;
            else
                acadoVariables.W[i*NY+j]=0;
        }
    }
 	acadoVariables.W[0] = 0.0001;
	acadoVariables.W[13] = 0.0001;
	acadoVariables.W[26] = 0.0001;
	 acadoVariables.W[39] = 0.5;
	 acadoVariables.W[52] = 0.5;
	 acadoVariables.W[65] = 0.5;


    for (i = 0; i < NYN; ++i) {
        for(j = 0; j < NYN; ++j) {
            if(i==j)
                acadoVariables.WN[i*NYN+j] = 5;
            else
                acadoVariables.WN[i*NYN+j]=0;
        }
    }

	acadoVariables.WN[0] = 35;
	acadoVariables.WN[7] = 35;
	acadoVariables.WN[14] = 35;
	/* acadoVariables.WN[21] = 10;
	 acadoVariables.WN[28] = 10;
 	acadoVariables.WN[35] = 10;*/

}


void initialize_mpc(){
    int i,j;

    /* Initialize the states and controls. */
    for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;  //Changed to 0.0




    for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

    /* Initialize the measurements/reference. */
    for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
    for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;


	//Initialize the online data values
	for(i =0; i<NOD * (N+1); ++i) acadoVariables.od[i] = 0.0;


	


    /* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
    for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.0; //Changed to 0.0 from 0.01


#endif


}

void print_data(){ ///Function to print the important values whenever needed.



    cout<<"Current Position: "<<acadoVariables.x0[0]<<" "<<acadoVariables.x0[1]<<" "<<acadoVariables.x0[2]<<" "<<endl;
    cout<<"Current Orientation "<<acadoVariables.x0[6]<<" "<<acadoVariables.x0[7]<<" "<<acadoVariables.x0[8]<<" "<<acadoVariables.x0[9]<<endl;

    cout<<"Position Desired: "<<acadoVariables.od[0]<<" "<<acadoVariables.od[1]<<" "<<acadoVariables.od[2]<<endl;
    cout<<"Orientation Desired: "<<acadoVariables.od[6]<<" "<<acadoVariables.od[7]<<" "<<acadoVariables.od[8]<<" "<<acadoVariables.od[9]<<endl;





}

void set_y_matrix(){

    int i = 0;


    /* Initialize the measurements/reference. */
    for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
    for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;


}

void quats2rpy(space_cobot_interface::pwm_values &Data){
    ///Converting the orientations back to quaternions
    tf::Quaternion q_temp_des, q_temp_cur;

    q_temp_des.setX(orientation_desired.x());
    q_temp_des.setY(orientation_desired.y());
    q_temp_des.setZ(orientation_desired.z());
    q_temp_des.setW(orientation_desired.w());

    q_temp_cur.setX(orientation_current.x());
    q_temp_cur.setY(orientation_current.y());
    q_temp_cur.setZ(orientation_current.z());
    q_temp_cur.setW(orientation_current.w());



    tf::Matrix3x3 mat1(q_temp_cur);
    tfScalar roll_cur=0, pitch_cur=0, yaw_cur=0;
    mat1.getEulerYPR(yaw_cur, pitch_cur, roll_cur);

    tf::Matrix3x3 mat2(q_temp_des);
    tfScalar roll_des=0, pitch_des=0, yaw_des=0;
    mat2.getEulerYPR(yaw_des, pitch_des, roll_des);

    ///The yaw, pitch, roll we get from the above function is in radians, so we need to convert it to degrees before putting it in the ros message
    Data.Current_rpy[0]= roll_cur*(180/pi);
    Data.Current_rpy[1]= pitch_cur*(180/pi);
    Data.Current_rpy[2]= yaw_cur*(180/pi);

    Data.Desired_rpy[0]= roll_des*(180/pi);
    Data.Desired_rpy[1]= pitch_des*(180/pi);
    Data.Desired_rpy[2]= yaw_des*(180/pi);

    Data.Error_rpy[0] = Data.Desired_rpy[0] - Data.Current_rpy[0];
    Data.Error_rpy[1] = Data.Desired_rpy[1] - Data.Current_rpy[1];
    Data.Error_rpy[2] = Data.Desired_rpy[2] - Data.Current_rpy[2];


}

double wrap(double x) {
    return x-2*pi*floor(x/(2*pi)+0.5);
}

void computeQuatError(Eigen::Vector3d &err_Quat){
    ///In this function, the error between orientation_curr and orientation_des will be calculated and saved in an array for publishing
    Eigen::Quaterniond error_Quaternion;
    error_Quaternion=orientation_desired*(orientation_current.inverse());

    err_Quat[0]= wrap(2*(acos(error_Quaternion.w())));
    err_Quat[1]= 2*0*(asin(error_Quaternion.y()));
    err_Quat[2]= 2*0*(asin(error_Quaternion.z()));
}


void assign_data(space_cobot_interface::pwm_values& data, vector<double> des_position, vector<double> des_velocity, vector<double> des_ang_vel, double PWM[6], double RPM[6], Eigen::Vector3d& error_quaternion){

    quats2rpy(data);

    for(int i=0; i<3; i++){
        data.Desired_position[i] = des_position[i];
        data.Desired_velocity[i] = des_velocity[i];
        data.Desired_omega[i] = des_ang_vel[i];
        data.Quat_error[i] = error_quaternion[i];

    }

    for(int i=0; i<6; i++){
        data.Actuation[i] = acadoVariables.u[i];
        data.PWM[i] = PWM[i];
        data.rpm[i] = RPM[i];
    }

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "mpc_ros");
    ros::NodeHandle nh;


    //ROS publisher and subscriber to get the values of current state and desired state
    //TODO(ALG): Getting the current state from the IMU of pixracer instead of vrpn
    //ros::Subscriber sub_current_state = nh.subscribe("/vrpn_client_node/RigidBody1/pose", 10, vrpnCallback);

    ros::Subscriber sub_current_pose = nh.subscribe("/mavros/imu/data", 10, imuCallback);
    ros::Subscriber sub_current_position = nh.subscribe("/space_cobot/mocap_interface/data", 1000, mocapCallback);

    ros::Publisher pwm_values_pub = nh.advertise<space_cobot_interface::pwm_values>
            ("important_values", 1000);

    /* Some temporary variables. */
    int    i, iter;
    acado_timer t;

    /* Initialize the solver. */
    acado_initializeSolver();

    initialize_mpc();  //program by ALG to initialize the values. Different from the acado_initializeSolver()


    if( VERBOSE ) acado_printHeader();

    /* Prepare first step */
    acado_preparationStep();

    /* Get the time before start of the loop. */
    acado_tic( &t );

    ros::Rate rate(100);



    /// Initialising the rosparams
    nh.param<vector<double>>("desired_position", position_desired);
    nh.param<vector<double>>("desired_orientation_rpy", orientation_desired_rpy);
    nh.param<vector<double>>("desired_velocity", velocity_desired);
    nh.param<vector<double>>("desired_angular_velocity", angular_velocity_desired);


    while(ros::ok())
    {

        ///getting all the rosparams
        nh.getParam("desired_position", position_desired);
        nh.getParam("desired_velocity", velocity_desired);
        nh.getParam("desired_orientation_rpy", orientation_desired_rpy);
        nh.getParam("desired_angular_velocity", angular_velocity_desired);
       


        getCurrentValues();


        getTargetValues(); //this function assigns reference values to online data values.


        get_cost_matrices();



        set_y_matrix();

        /* Perform the feedback step. */
        acado_feedbackStep();

        print_data(); //Function by ALG to print all the ACADO Variables.

        // Conversion of control inputs to PWM values
        // Limiting PWM values to the range
        double RPM[6], PWM[6];
        space_cobot_interface::pwm_values pwm_values_msg;

        for(int j=0; j<6; j++)
        {
            double rpm = thrustToRPM(acadoVariables.u[j]);
            double pwm = convertToPWM(rpm);

            /// safe range of PWM-- limiting the values of PWM
            if(pwm<1350)
                pwm=1350;
            else if (pwm> 1615)
                pwm= 1615;

            /// Saving these values to the array
            RPM[j]= rpm;
            PWM[j]= pwm;

        }

        cout<<"PWM Values: "<<endl;
        for(int it = 0; it<6; it++)
            cout<<PWM[it]<<" ";

        cout<<endl;

        //assign values to the pwm_values_msg
        pwm_values_msg.header.stamp = ros::Time::now();

        Eigen::Vector3d error_quaternion;  //TODO(ALG): PLEASE CHECK HOW THE ERROR IS BEING CALCULATED!!!!!!!!!!
	
	computeQuatError(error_quaternion);

        //function to assign values to the message to be published which is subscribed by interface node.
        assign_data(pwm_values_msg, position_desired, velocity_desired, angular_velocity_desired, PWM, RPM, error_quaternion);

        //Publish the PWM values for the interface to take
        pwm_values_pub.publish(pwm_values_msg);


        if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );
	cout<<"Objective: "<<acado_getObjective()<<endl;


        //shift the initialization
        acado_shiftStates(2, 0, 0);
        acado_shiftControls( 0 );

        // Prepare for the next step. */
        acado_preparationStep();

        //acado_printDifferentialVariables();
        //acado_printControlVariables();

        ros::spinOnce();
        rate.sleep();

    }

    /* Read the elapsed time. */
    real_t te = acado_toc( &t );

    if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

    /* Eye-candy. */

    if( !VERBOSE )
        printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

    acado_printDifferentialVariables();
    acado_printControlVariables();


    return 0;

}
