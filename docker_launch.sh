#!/bin/bash

# sudo chmod -R a+rw /home/nvidia/Desktop/software_cobot

# Check if exactly one argument is provided
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 {build|run|exec}"
  exit 1
fi

# Check which argument was provided
case "$1" in
  "build")
    # Docker build command
    docker build -t scobot_ros2 .
    ;;
  "run")
    # Check if the container is already running
    if [ "$(docker ps -q -f name=scobot)" ]; then
      echo "The container 'scobot' is already running."
    else
      # Check if the container exists (stopped)
      if [ "$(docker ps -aq -f name=scobot)" ]; then
        # Start the existing container
        docker start scobot
        echo "Started the existing 'scobot' container."
      else
        # Create and run a new container
        docker run -it --network=host --ipc=host -e DISPLAY=$DISPLAY -v $PWD/ros2_ws:/ros2_ws -e DISPLAY=host.docker.internal:0 -v /tmp/.X11-unix:/tmp/.X11-unix:rw --name scobot scobot_ros2
      fi
    fi
    ;;
  "exec")
    # Check if the container is running
    if [ "$(docker ps -q -f name=scobot)" ]; then
      # Docker exec command
      docker exec -it scobot /bin/bash
    else
      echo "The container 'scobot' is not running."
    fi
    ;;
  *)
    echo "Invalid argument. Usage: $0 {build|run|exec}"
    exit 1
    ;;
esac

exit 0
