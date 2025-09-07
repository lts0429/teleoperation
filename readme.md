# Introduction
A project demonstrating teleoperation of robot using a VR headset.

![ros_tracking_hand](doc/ros_hand_tracker.gif)

# Requirement
The project has been tested on:
- Ubuntu 22.04
- ROS2 Humble
- Meta Quest 3

# Quickstart
To test this project you need a VR headset and a PC. They have to be in the same network.
## Meta Quest 3
1. Enable developer mode on your VR headset
https://developers.meta.com/horizon/documentation/native/android/mobile-device-setup/
2. Install the APK on your device
https://developers.meta.com/horizon/documentation/native/android/ts-mqdh-deploy-build/
3. After installation, run the teleoperator app

## PC
1. Download the meta_quest_client package into your workspace and build it.
2. Run the udp_client node.
```
ros2 run meta_quest_client udp_client
```

The following topics are published after starting the node. You can use rviz to visualize them.
- headset (PoseStamped)
- left_hand (PoseStamped)
- right_hand (PoseStamped)

# To Dos:
- [x] Retrieve headset and hand tracking data from VR headset
- [x] Convert and publish tracking data into ROS topics
- [ ] Set up robot arm with Moveit
- [ ] Apply pose tracking to robot arm