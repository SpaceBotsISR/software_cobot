### ✅ `README.md`

# 🧠 ROS 2 Humble Docker Environment – space_cobot

This container provides a full-featured ROS 2 Humble development environment with GPU, X11 forwarding, zsh, and Python scientific tools — all under a user called `space_cobot`.

## 🔧 Setup

1. Make sure you have the following installed on your host:

   - **Docker**: Install Docker by following the [official Docker installation guide for Linux](https://docs.docker.com/engine/install/).

     ```bash
     sudo apt-get update
     sudo apt-get install -y docker.io
     sudo systemctl start docker
     sudo systemctl enable docker
     ```

   - **Docker Compose v2**: Install Docker Compose v2 by following the [official guide](https://docs.docker.com/compose/install/).

     ```bash
     DOCKER_CONFIG=${DOCKER_CONFIG:-$HOME/.docker}
     mkdir -p $DOCKER_CONFIG/cli-plugins
     curl -SL https://github.com/docker/compose/releases/download/v2.20.2/docker compose-linux-x86_64 -o $DOCKER_CONFIG/cli-plugins/docker compose
     chmod +x $DOCKER_CONFIG/cli-plugins/docker compose
     ```

   - **NVIDIA Container Toolkit** (for GPU access): Follow the [NVIDIA Container Toolkit installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

     ```bash
     distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
     curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
     curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
     sudo apt-get update
     sudo apt-get install -y nvidia-container-toolkit
     sudo nvidia-ctk runtime configure --runtime=docker
     sudo systemctl restart docker
     ```

   - **X11 server**: Ensure X11 is installed and running. On Linux, install it using:
     ```bash
     sudo apt-get install -y x11-xserver-utils
     ```

2. Allow X11 forwarding (Linux/macOS):

```bash
xhost +local:docker
```

3. Restart the Docker service:

```bash
sudo systemctl restart docker
```

---

## 🛠️ Build the Container

```bash
docker compose up -d
```

## 🏁 Start the Container

```bash
docker compose start
```

## 🧵 Open Terminals

```bash
docker exec -it space_cobot_container zsh
```

This drops you into a new shell inside the same running container.

---

### ✅ Helpful Container Aliases

| Command | What it does                                       |
| ------- | -------------------------------------------------- |
| `rs`    | Sources your ROS 2 workspace (`install/setup.zsh`) |
| `ws`    | Goes to the ros2 workspace (`~/host_ws/`)          |

## 🔚 Stop the Container

```bash
docker compose down
```

## 🗑️ Remove the Container and Image

```bash
docker compose down --rmi all
```

## ⬇️ Install Packages inside the container

```bash
apt-get update
apt-get install -y <package_name>
```
