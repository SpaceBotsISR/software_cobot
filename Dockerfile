# Use the base image for arm64 architecture and the ROS distribution "humble"
FROM arm64v8/ros:humble

# Install packages
RUN apt-get update && apt-get install -y \
    # ROS:
ros-humble-mavros \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-rviz-visual-tools \
    ros-humble-rviz-rendering \
    ros-humble-nav2-rviz-plugins \
    ros-humble-foxglove-bridge \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-sensor-msgs \
    ros-humble-tf-transformations \
    libtf2-dev \
    # Others:
    git \
    ccache \
    net-tools \
    python3-ament-clang-format \
    python3-argcomplete \
    python3-pip \
    python3-numpy \
    libboost-python-dev \
    bash-completion \
    vim \
    libopencv-dev \
    python3-opencv && \
    rm -rf /var/lib/apt/lists/* && \
    pip install \
    opencv-python \
    opencv-contrib-python \
    transforms3d \
    pip install opencv-contrib-python==4.6.0.66

# Add environment setup to the user's .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/humble/" >> /root/.bashrc


