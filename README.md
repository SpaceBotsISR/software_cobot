## ROS2 with Docker

This **docker_launch.sh** script simplifies the management of the created **ROS2 Docker** container for **Space Cobot** developement. It allows you to build, run, and execute commands inside the container.

### Usage:

1. Building the Docker Image:

    ```
    ./docker_launch.sh build
    ```

    This command creates a Docker image named **scobot_ros2** from your application's Dockerfile.

2. Running the Docker Container:
   To start or resume a Docker container with the name **scobot**," run:

    ```
    ./docker_launch.sh run
    ```

    - If the container is already running, it will display a message indicating that it's already running.
    - If the container exists but is stopped, it will start the existing container.
    - If the container does not exist, it will create and run a new container based on the "scobot_ros2" image.

3. Running a terminal inside the container:
   To run commands inside the running container, run:

    ```
    ./docker_launch.sh exec
    ```

    - If the container is running, it will open an interactive shell inside the container.
    - If the container is not running, it will display a message indicating that the container is not running.

4. Cleaning Up:
    - You do not need to remove or recreate the container each time you use the script.
    - The script will manage the container for you, either starting it if not running or executing commands inside it when running.
