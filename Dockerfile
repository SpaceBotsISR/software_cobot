# Use the base image for arm64 architecture and the ROS distribution "humble"
FROM arm64v8/ros:humble

# Install packages
RUN apt-get update && apt-get install -y \
    # ROS:
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
    ros-humble-cv-bridge \
    libtf2-dev \
    # gstreamer:
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev \
    # Camera:
    cmake libgtk-3-dev libjpeg-dev libgles2-mesa-dev libgstreamer1.0-dev \
    # Others:
    git \
    ccache \
    net-tools \
    python3-ament-clang-format \
    python3-argcomplete \
    python3-pip \
    bash-completion \
    vim \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install opencv-python

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh

# Add environment setup to the user's .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> /root/.bashrc


# Set the entry point and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["/bin/bash", "-c", "export GSCAM_CONFIG='videotestsrc pattern=snow ! video/x-raw,width=1280,height=720 ! videoconvert' \
    && ros2 run gscam2 gscam_main"]