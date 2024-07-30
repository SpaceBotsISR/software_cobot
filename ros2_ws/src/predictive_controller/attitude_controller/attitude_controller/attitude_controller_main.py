# MIT License
# 
# Copyright (c) 2024 Andre Rebelo Teixeira
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.import rclpy 

## Ros2 modules
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSPresetProfiles
from ament_index_python.packages import get_package_share_directory

## Ros2 message modules
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

## Self defined modules
from attitude_controller.controller import RotationDynamics as Controller
from attitude_controller.ThrustPwmConvert import ThrustPwmConvert as ThrustPwmConvert

## Python modules
import os 
import time
import numpy as np
import sympy as sp  

class AttitudeController(Node):
    def __init__(self):
        '''
        Initialize the attitude controller node
        
        Parameters
            None
        
        Returns
            None
        '''
        super().__init__('predictive_attitude_controller')

        ## Declare the subscriber
        self.imu_data_subscriber = self.create_subscription(Imu, '/mavros/imu/data', self.imu_data_callback, QoSPresetProfiles.SENSOR_DATA.value)
        self.desired_attitude_subscriber = self.create_subscription(Float64MultiArray, 'desired_attitude', self.desired_attitude_callback, 10)
        ## Declare the publisher
        self.actuation_publisher = self.create_publisher(Float64MultiArray, '/desired_attuation', 10)

        ## Declare the parameters 
        self.declare_parameter('controller_dt', 0.1) # 10 Hz = 0.1 s
        self.declare_parameter('desired_attitude', [0.0, 0.0, 0.0])
        self.declare_parameter('J_matrix_path', 'no_file')
        self.declare_parameter('c_vector_path', 'no_file')
        self.declare_parameter('A_matrix_path', 'no_file')
        self.declare_parameter('neg_thrust_model_path', 'no_file')
        self.declare_parameter('pos_thrust_model_path', 'no_file')
        self.declare_parameter('horizon', 40)
        self.declare_parameter('time_step', 0.01)

        ## Get the parameter value if it was changed at launch
        self.controller_dt = self.get_parameter('controller_dt').get_parameter_value().double_value
        self.desired_attitude = self.get_parameter('desired_attitude').get_parameter_value().double_array_value
        self.J_matrix_path = self.get_parameter('J_matrix_path').get_parameter_value().string_value
        self.c_vector_path = self.get_parameter('c_vector_path').get_parameter_value().string_value
        self.A_matrix_path = self.get_parameter('A_matrix_path').get_parameter_value().string_value
        self.neg_thrust_model_path = self.get_parameter('neg_thrust_model_path').get_parameter_value().string_value
        self.pos_thrust_model_path = self.get_parameter('pos_thrust_model_path').get_parameter_value().string_value
        self.horizon = self.get_parameter('horizon').get_parameter_value().integer_value
        self.time_step = self.get_parameter('time_step').get_parameter_value().double_value

        if self.J_matrix_path == 'no_file' or os.path.isfile(self.J_matrix_path) == False: 
            self.get_logger().warn("The J matrix path is either not valid or not set. Using the default file provided with the package")
            package_name = 'attitude_controller'
            package_share_directory = get_package_share_directory(package_name)
            self.J_matrix_path = os.path.join(package_share_directory, 'configs', 'J_matrix.npy')

        if self.c_vector_path == 'no_file' or os.path.isfile(self.c_vector_path) == False:
            self.get_logger().warn("The c vector path is either not valid or not set. Using the default file provided with the package")
            package_name = 'attitude_controller'
            package_share_directory = get_package_share_directory(package_name)
            self.c_vector_path = os.path.join(package_share_directory, 'configs', 'c_vector.npy')
        
        if self.A_matrix_path == 'no_file' or os.path.isfile(self.A_matrix_path) == False:
            self.get_logger().warn("The A matrix path is either not valid or not set. Using the default file provided with the package")
            package_name = 'attitude_controller'
            package_share_directory = get_package_share_directory(package_name)
            self.A_matrix_path = os.path.join(package_share_directory, 'configs', 'A_matrix.npy')
        
        if self.neg_thrust_model_path == 'no_file' or os.path.isfile(self.neg_thrust_model_path) == False:
            self.get_logger().warn("The negative thrust model path is either not valid or not set. Using the default file provided with the package")
            package_name = 'attitude_controller'
            package_share_directory = get_package_share_directory(package_name)
            self.neg_thrust_model_path = os.path.join(package_share_directory, 'configs', 'neg_thrust_model.npy')
        
        if self.pos_thrust_model_path == 'no_file' or os.path.isfile(self.pos_thrust_model_path) == False:
            self.get_logger().warn("The positive thrust model path is either not valid or not set. Using the default file provided with the package")
            package_name = 'attitude_controller'
            package_share_directory = get_package_share_directory(package_name)
            self.pos_thrust_model_path = os.path.join(package_share_directory, 'configs', 'pos_thrust_model.npy')
    

        ## Load the matrices
        J = np.load(self.J_matrix_path)
        c = np.load(self.c_vector_path)
        A = np.load(self.A_matrix_path)
        neg_pwm_to_thrust_coefs = np.load(self.neg_thrust_model_path)
        pos_pwm_to_thrust_coefs = np.load(self.pos_thrust_model_path)

        J *= 10

        A = A[0:3, :]

        self.neg_ThrustPwmConvert = ThrustPwmConvert(neg_pwm_to_thrust_coefs, [1000, 1500], 1500)
        self.pos_ThrustPwmConvert = ThrustPwmConvert(pos_pwm_to_thrust_coefs, [1500, 2000], 1500)
        
        self.controller = Controller(J, c, A, self.horizon, self.time_step)
        self.controller.setup()
        self.timer = self.create_timer(self.controller_dt, self.controller_step)

        ## System state variables
        self.imu = None
        self.w = np.zeros((3, 1))
        self.w_dot = np.zeros((3, 1))
        self.q = np.array([1, 0, 0, 0])

    def invert_thrust_model(self, thrust : list) -> list:
        '''
        This function inverts model we got from the motor calibration that map our pwm to thrust, 
        allowing us to map from thrust value to their desired actuation value, taking into consideration the 
        motor dynamics, and making use of the same equation

        PWM model converts from us to thrust. In the RCbenchmark output file, is the value given as ESC time.
        
        Parameters
            thrust - list: The coeficients of the polynomial model that maps the pwm to thrust 
                    in the form [a, b, c, d] where the model is a + b * x + c * x**2 + d * x **3, the degree of the polinomial 
                    can be changed, but the order should be maintained
        Return: 
            solution - list: the coeficients of the polynomial model that maps the thrust to the actuation value
        '''
        x, y  = sp.symbols('x y')
        eq = thrust[0]

        for i in range(1, len(thrust)):
            eq += thrust[i] * x ** i
        
        solution = sp.solve(eq - y, x)

        return solution

    def imu_data_callback(self, msg : Imu) -> None:
        '''
        Callback for the imu data topic, to update the system state variables

        Parameters
            msg - sensor_msgs.msg.Imu: the imu message

        Returns
            None
        '''

        ## Update the system angular velocity
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        ## Update the system rotation quaternion
        self.q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        ## The handling of the first message receive in the topic must be different since I can't compute the angular acceleration
        if self.imu is None:
            self.w_dot = np.zeros((3, 1))
        else:
            ## Compute the angular acceleration as the difference between the current and the last angular velocity divided by the time difference
            time_diff_ns = (msg.header.stamp.sec - self.imu.header.stamp.sec) * 1e9 + (msg.header.stamp.nanosec - self.imu.header.stamp.nanosec)
            self.w_dot = (w - self.w) / (time_diff_ns * 1e-9) # rads per second squared

        self.w = w

        ## Update the message from the imu
        self.imu = msg

        return
        
    def desired_attitude_callback(self, msg: Float64MultiArray) -> None:
        '''
        Callback or the desired attitude topic, to make sure we can
        change the values of the attitude controller in real time
        
        Parameters
            msg - std_msgs.msg.Float64MultiArray: the desired attitude
        
        Returns
            None
        '''
        
        if len(msg.data) == 3:
            self.desired_attitude = np.array(msg.data)
        
        else:
            self.get_logger().warn('The desired attitude must be a 3D vector. Value not updated.')
            self.get_logger().warn('Current value: {}'.format(self.desired_attitude))
        return
    
    def controller_step(self):
        '''
        Compute the next actuation value to send to the motors
        
        Parameters
            None
        
        Returns
            actuation - list: the actuation value to send to the motors        
        '''
        ## get actuation in rps (rotation per second)
        if self.imu is not None:
            start = time.time_ns()
            sol = self.controller.step(self.w, self.w_dot, self.q, self.desired_attitude)
            end = time.time_ns()

            self.get_logger().warn('Time (ms) to solve the optimization problem: {}'.format((end - start) * 1e-6)) 

            actuation = sol.value(self.controller.u[:, 0])
            
            msg = Float64MultiArray()
            msg.data = []


            ## convert thrust to pwm 
            for i in range(len(actuation)):
                msg.data.append( int(self.thrust_to_pwm(actuation[i], i + 1)) )
            
            self.get_logger().warn('Actuation: {}'.format(actuation))
            
            ## publish the pwm instead of returning it 


            self.actuation_publisher.publish(msg)

        else: 
            self.get_logger().warn('The imu data is still not available, not running control')
            return np.zeros((6, 1))

    def thrust_to_pwm(self, thrust : float, motor_id : int): 
        '''
        Convert the desired thrust to pwm value to send to the motor
        The polinomial function we used here was obtained by fitting a curve of collect data from the motor using RC benchmark
        
        Parameters
            thrust - float: the desired thrust value

        Returns
            pwm - float: the pwm value to send to the motor
        '''
        pwm = None

        if motor_id % 2 != 0: 
            thrust *= -1

        if thrust > 0:
            pwm = self.neg_ThrustPwmConvert.thrust_to_pwm(thrust)
        else:
            pwm  = self.pos_ThrustPwmConvert.thrust_to_pwm(thrust)

        return pwm

    def rpm_to_pwm(self, rpm : float): 
        '''
        Convert the desired rpm to pwm value to send to the motor
        The polinomial function we used here was obtained by fitting a curve of collect data from the motor using RC benchmark

        Parameters
            rpm - float: the desired rpm value
        
        Returns
            pwm - float: the pwm value to send to the motor
        '''
        pwm = None

        ## Compute the pwm value 
        if rpm < 0: 
            pwm =  -0.00000000000645913910427410 * pow(rpm,3) -0.000000338986537473653 * pow(rpm,2) + 0.0115794605304886 * rpm + 1465.70343578529  # cubic regression curve
        else:
            pwm =  -0.0000000000110106978797619 * pow(rpm,3) + 0.000000448602698785971 * pow(rpm,2) + 0.0108341387639997 * rpm + 1500.87746062623

        ## Make sure that the pwm is inside the correct boundaries
        return min(1750, max(1250, pwm))

def main(args=None):
    rclpy.init(args=args)
    attitude_controller = AttitudeController()
    rclpy.spin(attitude_controller)
    attitude_controller.destroy_node()
    rclpy.shutdown()