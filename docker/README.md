# ROS 2 Humble Docker Environment – space_cobot

ROS 2 Humble container with GPU support, Gazebo, X11 forwarding, zsh, and Python tools under the `space_cobot` user.

---

## 🧩 Prerequisites (on host)

### Docker & Compose

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-plugin
sudo systemctl enable --now docker
```

### NVIDIA Toolkit (optional – only for NVIDIA GPU users)

If you want GPU acceleration (CUDA, GPU rendering, or Gazebo GPU sensors):

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### X11 (for RViz/Gazebo)

```bash
sudo apt install -y x11-xserver-utils
xhost +local:docker
```

---

## Build and Run

```bash
docker compose up -d --build
```

## Open a Shell

```bash
docker exec -it space_cobot_container zsh
```

---

## Inside the Container

**Aliases**

-   `rs` → source `install/setup.zsh`
-   `ws` → go to `~/software_cobot/ros2_ws` and source it

**Install packages**

```bash
sudo apt update
sudo apt install -y <package_name>
```

## 🧹 Stop or Remove

```bash
docker compose down         # stop
docker compose down --rmi all  # remove image
```

```

```
