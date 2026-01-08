FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    QT_X11_NO_MITSHM=1

# Base tools, OpenGL libs, locale
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales ca-certificates curl wget gnupg2 lsb-release \
    build-essential git vim net-tools sudo zsh python3-pip python3-venv \
    mesa-utils libgl1 libgl1-mesa-dri libglu1-mesa \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Garden repository
RUN apt-get update && apt-get install -y --no-install-recommends wget gnupg lsb-release \
    && wget -qO /usr/share/keyrings/gazebo-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# ROS 2 Humble + Gazebo Garden + useful packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Core
    ros-humble-desktop \
    ros-humble-ros-base \
    ros-humble-diagnostic-updater \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-tf-transformations \
    ros-humble-pluginlib ros-humble-class-loader \
    # Messages, geometry, and math
    ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-nav-msgs \
    ros-humble-angles ros-humble-geometry2 \
    # Control + Navigation
    ros-humble-nav2-bringup ros-humble-nav2-core ros-humble-nav2-costmap-2d \
    ros-humble-nav2-map-server ros-humble-nav2-planner ros-humble-nav2-controller ros-humble-nav2-waypoint-follower \
    ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-publisher ros-humble-joint-state-broadcaster \
    # Perception + Image
    ros-humble-vision-opencv ros-humble-cv-bridge ros-humble-image-transport ros-humble-image-transport-plugins \
    ros-humble-depth-image-proc ros-humble-pcl-msgs ros-humble-pcl-conversions \
    # Visualization
    ros-humble-rviz2 ros-humble-rviz-common ros-humble-rviz-default-plugins ros-humble-rviz-visual-tools ros-humble-rviz-rendering ros-humble-nav2-rviz-plugins \
    # Logging and bags
    ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins ros-humble-rosbag2-storage-mcap \
    # Bridges
    ros-humble-foxglove-bridge ros-humble-ros-gz \
    # MAVROS 
    ros-humble-mavros ros-humble-mavros-msgs \
    # Gazebo Garden
    gz-garden \
    # Misc
    ros-humble-robot-state-publisher ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir numpy opencv-python scipy catkin_pkg lark "empy<4.0"

# Non-root user
ARG USERNAME=space_cobot
ARG UID=1000
RUN useradd -m -u ${UID} -s /bin/zsh ${USERNAME} \
    && usermod -aG sudo ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Zsh + plugins
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
    && git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-/home/${USERNAME}/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
    && git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-/home/${USERNAME}/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting \
    && sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/' /home/${USERNAME}/.zshrc \
    && sed -i 's/ZSH_THEME=".*"/ZSH_THEME="agnoster"/' /home/${USERNAME}/.zshrc

# Shell setup
RUN echo "source /opt/ros/humble/setup.zsh" >> /home/${USERNAME}/.zshrc \
    && echo "alias rs='[ -f install/setup.zsh ] && source install/setup.zsh || echo \"No install/ yet\"'" >> /home/${USERNAME}/.zshrc \
    && echo "alias ws=\"cd ~/software_cobot/ros2_ws && rs\"" >> /home/${USERNAME}/.zshrc
