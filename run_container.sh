#!/usr/bin/env bash

# Allow GUI access for local root user
xhost +local:root

# Clean up old container (optional)
docker rm -f contact_graspnet_dev 2>/dev/null

# Run the container
docker run -it \
  --rm \
  --gpus all \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  -v /home/csrobot/graspnet_ws/src/contact_graspnet_ros/checkpoints/:/home/user/cgn_ws/src/cgn_ros/checkpoints/ \
  -v /home/csrobot/graspnet_ws/src/contact_graspnet_ros/test_data/:/home/user/cgn_ws/src/cgn_ros/test_data/ \
  -v /home/csrobot/graspnet_ws/src/contact_graspnet_ros/src/:/home/user/cgn_ws/src/cgn_ros/src/ \
  --network=host \
  --privileged \
  --name contact_graspnet_dev \
  contact-graspnet \
  bash

