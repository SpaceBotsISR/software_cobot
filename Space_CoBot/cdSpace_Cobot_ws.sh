#! /bin/bash
#Script access the folders of the space cobot ws in an easier manner


#For the interface package src
function sc_interface() {
    cd Space_CoBot/Space_CoBot/SpaceCobot-ros_px4/px4-ROS/ros_controllers/space_cobot_ws/src/space_cobot_interface/src
}

#For the ws folder
function sc_ws() {
    cd Space_CoBot/Space_CoBot/SpaceCobot-ros_px4/px4-ROS/ros_controllers/space_cobot_ws
}

#For the controller package src
function sc_controller() {
    cd Space_CoBot/Space_CoBot/SpaceCobot-ros_px4/px4-ROS/ros_controllers/space_cobot_ws/src/space_cobot_controller/src
}

#For the pixhawk usb door premissions
function sc_PixDoor() {
    sudo chmod 666 /dev/ttyACM0
}

#To record a bag
function sc_Bag() {
  #statements
  rosbag record -o sc_bag /csi_cam_0/image_raw /mavros/imu/data /csi_cam_0/camera_info
}
