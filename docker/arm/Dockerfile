FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-rviz-visual-tools \
    ros-humble-rviz-rendering \
    ros-humble-nav2-rviz-plugins \
    ros-humble-foxglove-bridge \
    ros-humble-rosbag2-storage-mcap \
    git \
    ccache \
    net-tools \
    vim \
    libopencv-dev \
    zsh \
    locales \
    curl \
    wget \
    python3-venv \
    sudo \
    fonts-powerline \
    && rm -rf /var/lib/apt/lists/*

# Locale setup
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Create non-root user
ARG USERNAME=space_cobot
ARG UID=1000

RUN useradd -m -u $UID -s /bin/zsh $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to new user
USER $USERNAME
WORKDIR /home/$USERNAME

# Install Oh My Zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-/home/$USERNAME/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-/home/$USERNAME/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/' /home/$USERNAME/.zshrc && \
    sed -i 's/ZSH_THEME=".*"/ZSH_THEME="agnoster"/' /home/$USERNAME/.zshrc

# Set permissions for Oh My Zsh folders
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/.oh-my-zsh /home/$USERNAME/.zshrc

# Optional: Copy your custom .zshrc on top
# COPY --chown=$USERNAME:$USERNAME .zshrc /home/$USERNAME/.zshrc

# Python venv
RUN python3 -m venv /home/$USERNAME/venv && \
    /home/$USERNAME/venv/bin/pip install --upgrade pip && \
    /home/$USERNAME/venv/bin/pip install numpy opencv-python scipy

ENV PATH="/home/$USERNAME/venv/bin:$PATH"

# Add custom aliases and source commands to .zshrc
RUN echo "source /opt/ros/humble/setup.zsh" >> /home/$USERNAME/.zshrc && \
    echo "source ~/venv/bin/activate" >> /home/$USERNAME/.zshrc && \
    echo "alias rs='source install/setup.zsh'" >> /home/$USERNAME/.zshrc && \
    echo "alias ws='cd ~/software_cobot/ros2_ws && rs'" >> /home/$USERNAME/.zshrc
