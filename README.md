# contact_graspnet_ros

This is a ROS wrapper for contact graspnet.
The contact graspnet code can be found at: https://github.com/NVlabs/contact_graspnet

## Installation
0. Make sure you have Docker installed.
1. Clone this repository into the source folder of your catkin workspace.
2. Build the catkin workspace: `catkin build`.
3. Run `./build_container.sh` to build the docker container.

## Usage

Call the GetGrasps service with a point cloud and target points to get grasps and corresponding contact points.
