#!/usr/bin/env python
import rospy
import numpy as np
import numpy.linalg as la
import numpy.random as rnd
import matplotlib.pyplot as plt
import tf


from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from random import randint
from math import pi

COPTER_LOOP_RATE = 100
COPTER_CM_HEIGHT = 8.7830e-02

#a function to generate the random number
def generate_random_number(min_n,max_n):
    rnd= randint(min_n,max_n)
    return rnd

def generate_random_pose():
    msg = PoseStamped()
#       msg.header.seq
#       msg.header.stamp
    msg.header.frame_id="fcu"

    msg.pose.position.x = generate_random_number(-5,5)
    msg.pose.position.y = generate_random_number(-5,5)
    msg.pose.position.z = generate_random_number(-5,5)

    roll = np.random.ranf(1)*2*pi
    pitch = np.random.ranf(1)*2*pi
    yaw = np.random.ranf(1)*2*pi

    q = quaternion_from_euler(roll,pitch,yaw)

    msg.pose.orientation.w = q[0]
    msg.pose.orientation.x = q[1]
    msg.pose.orientation.y = q[2]
    msg.pose.orientation.z = q[3]
    return msg

def generate_pose(position,euler):
    msg = PoseStamped()
#       msg.header.seq
#       msg.header.stamp
    msg.header.frame_id="fcu"

    msg.pose.position.x = position[0]
    msg.pose.position.y = position[1]
    msg.pose.position.z = position[2]

    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    q = quaternion_from_euler(roll,pitch,yaw)

    msg.pose.orientation.w = q[0]
    msg.pose.orientation.x = q[1]
    msg.pose.orientation.y = q[2]
    msg.pose.orientation.z = q[3]
    return msg

def generate_twist(linear, angular=[0,0,0]):
    msg = TwistStamped()
    msg.header.frame_id="fcu"

    msg.twist.linear.x = linear[0]
    msg.twist.linear.y = linear[1]
    msg.twist.linear.z = linear[2]

    msg.twist.angular.x = angular[0]
    msg.twist.angular.y = angular[1]
    msg.twist.angular.z = angular[2]

    return msg

#         x  y  z                   Ox Oy Oz
PATH1 = [(0, 0, COPTER_CM_HEIGHT,   0, 0, 0),
         (0, 0, 1,                  0, 0, 1),
         (1, 0, 1,                  0, 0, 1),
         (1, 1, 1,                  1, 0, 0),
         (1, 1, COPTER_CM_HEIGHT,   0, 0, 0)]

VREF = 0.05  # translation speed
OREF = 0.02  # rotation speed
T = 1.0/COPTER_LOOP_RATE
    
def gen_from_path(path=PATH1):
    out = []
    for i in xrange(len(path)-1):
        pi = np.array(path[i])
        pf = np.array(path[i+1])
        xi, oi = pi[0:3], pi[3:6]
        xf, of = pf[0:3], pf[3:6]
        d = la.norm(xf-xi)
        r = la.norm(of-oi)
        tt = max(d/VREF, r/OREF)
        for t in np.arange(0, tt, T):
            xd = xi + (xf-xi)*t/tt
            vd = (xf-xi)/tt
            #Rd = expr(oi + (of-oi)*t/tt)
            Od = (of-oi)/tt
            #q_des = quaternion_from_euler(Od[0], Od[1], Od[2])
            out.append((xd, Od, vd))
    return out

def quaternion_from_euler(roll, pitch, yaw):
    conversion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    q = [conversion[3],conversion[0],conversion[1],conversion[2]]
    return q


if __name__=='__main__':
    rospy.init_node('random_number')
    pose_pub=rospy.Publisher('/scobot/target_pose',PoseStamped, queue_size=0)
    vel_pub=rospy.Publisher('/scobot/target_vel',TwistStamped, queue_size=0)

    rate= rospy.Rate(COPTER_LOOP_RATE)
    loop_count = 0

    waypoints = gen_from_path()
    max_loops = len(waypoints)

    str = "Trajectory points: %d" % max_loops
    rospy.loginfo(str)
    #print 'Trajectory time: %d seconds'%(max_loops*1/COPTER_LOOP_RATE)
    str = "Trajectory time: %d seconds" % (max_loops*1/COPTER_LOOP_RATE)
    rospy.loginfo(str)

    while loop_count < max_loops and not rospy.is_shutdown():

        pose = waypoints[loop_count]

        position = pose[0]
        euler = pose[1]
        vel = pose[2]

        msg = generate_pose(position,euler)
        pose_pub.publish(msg)


        msg = generate_twist(vel)
        vel_pub.publish(msg)

        loop_count+=1
        rate.sleep()

    rospy.loginfo('Trajectory finished')
#    while not rospy.is_shutdown():
#        rnd_gen=generate_random_number(0,5000)
#
#        msg = generate_pose()
#
#        pose_pub.publish(msg)

