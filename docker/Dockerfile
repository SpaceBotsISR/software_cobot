FROM nvidia/cuda:12.6.0-base-ubuntu22.04
#! ARM: nvidia/cuda:12.6.0-base-ubuntu22.04-arm64

ENV DEBIAN_FRONTEND=noninteractive

# Set up locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Add ROS 2 apt repository
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 and your dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-mavros \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-rviz-visual-tools \
    ros-humble-rviz-rendering \
    ros-humble-nav2-rviz-plugins \
    ros-humble-foxglove-bridge \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-angles \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    git \
    ccache \
    net-tools \
    vim \
    libopencv-dev \
    zsh \
    curl \
    wget \
    sudo \
    python3-venv \
    fonts-powerline \
    python3-colcon-common-extensions

# Create user
ARG USERNAME=space_cobot
ARG UID=1000
RUN useradd -m -u $UID -s /bin/zsh $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

# Set up Oh My Zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-/home/$USERNAME/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-/home/$USERNAME/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/' /home/$USERNAME/.zshrc && \
    sed -i 's/ZSH_THEME=".*"/ZSH_THEME="agnoster"/' /home/$USERNAME/.zshrc && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/.oh-my-zsh /home/$USERNAME/.zshrc

# Create Python venv and install packages
RUN python3 -m venv ~/venv && \
    ~/venv/bin/pip install --upgrade pip && \
    ~/venv/bin/pip install \
    numpy \
    opencv-python \
    scipy \
    catkin_pkg \
    lark \
    "empy<4.0"

ENV PATH="/home/$USERNAME/venv/bin:$PATH"

# Add ROS and venv to shell
RUN echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc && \
    echo "source ~/venv/bin/activate" >> ~/.zshrc && \
    echo "alias rs='source install/setup.zsh'" >> ~/.zshrc && \
    echo "alias ws='cd ~/software_cobot/ros2_ws && rs'" >> ~/.zshrc
