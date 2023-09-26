# Use the base image for arm64 architecture and the ROS distribution "humble"
FROM arm64v8/ros:humble

# Install packages
RUN apt-get update && apt-get install -y \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-rviz-visual-tools \
    ros-humble-rviz-rendering \
    ros-humble-nav2-rviz-plugins \
    libtf2-dev \
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

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh

# Add environment setup to the user's .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> /root/.bashrc

# Set the entry point and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
