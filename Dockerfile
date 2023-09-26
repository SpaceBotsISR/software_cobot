#from osrf/ros:humble-desktop-full
ARG ROS_DISTRO=humble

from arm64v8/ros


# Install packages
RUN sudo apt-get -y update && sudo apt-get -y upgrade \  
    && sudo apt-get -y install \ 
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rviz-common \
    ros-$ROS_DISTRO-rviz-default-plugins \
    ros-$ROS_DISTRO-rviz-visual-tools \
    ros-$ROS_DISTRO-rviz-rendering \
    ros-$ROS_DISTRO-nav2-rviz-plugins  \
    && sudo apt-get -y install libtf2-dev \ 
    && sudo apt-get -y install \
    git \
    ccache \
    net-tools \
    ros-humble-rosbag2-storage-mcap \
    python3-ament-clang-format \
    python3-argcomplete \
    python3-pip \
    bash-completion \
    vim \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /root/.bashrc

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]




