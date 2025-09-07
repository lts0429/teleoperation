# Introduction
A project demonstrating teleoperation of robot using a VR headset. 

Checkout the full video here https://www.youtube.com/watch?v=Dg-QU7UiqbA

![ros_tracking_hand](doc/ros_hand_tracker.gif)

# Requirement
The project has been tested on:
- Ubuntu 22.04
- ROS2 Humble
- Meta Quest 3
- MoveIt (required for hand pose tracking demo)

# Quickstart
To test this project you need a VR headset and a PC. They have to be in the same network.
## Meta Quest 3
1. Enable developer mode on your VR headset
https://developers.meta.com/horizon/documentation/native/android/mobile-device-setup/
2. Install the APK on your device
https://developers.meta.com/horizon/documentation/native/android/ts-mqdh-deploy-build/
3. After installation, run the teleoperator app

## PC
### 1. Visualize Hand Tracking
1. Download the meta_quest_client package into your workspace and build it.
2. Run the udp_client node.
```
ros2 launch meta_quest_client meta_quest_client_demo.launch.py
```

The following topics are published after starting the node. You can use rviz to visualize them.
- headset (PoseStamped)
- left_hand (PoseStamped)
- right_hand (PoseStamped)

### 2. Hand Pose Tracking Servo
1. Run the udp client node
```
ros2 run meta_quest_client udp_client
```
2. Launch the MoveIt pose tracking demo
```
ros2 launch moveit_servo pose_tracking_example.launch.py
```