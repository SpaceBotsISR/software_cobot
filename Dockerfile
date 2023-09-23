from osrf/ros:humble-desktop-full

### Run with: docker run -it --network=host --ipc=host -v $PWD/ros2_ws

# Install packages
RUN sudo apt-get -y update && sudo apt-get -y upgrade \  
    && sudo apt-get -y install libtf2-dev \ 
    && sudo apt-get -y install \
    git \
    ccache \
    net-tools \
    ros-humble-rosbag2-storage-mcap \
    python3-ament-clang-format \
    python3-argcomplete \
    bash-completion \
    vim \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]




