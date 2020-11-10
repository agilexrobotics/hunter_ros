# ROS Packages for Hunter Mobile Base

## Packages

* hunter_base: a ROS wrapper around Hunter SDK to monitor and control the robot
* hunter_bringup: launch and configuration files to start ROS nodes 
* hunter_msgs: hunter related message definitions

## Communication interface setup

Please refer to the [README](https://github.com/westonrobot/ugv_sdk#hardware-interface) of "ugv_sdk" package for setup of communication interfaces.

#### Note on CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS package

1. Install dependent packages

    ```
    $ sudo apt install -y libasio-dev
    $ sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard
    ```
    ```
    $ sudo apt install libasio-dev
    ```
    
2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/agilexrobotics/agx_sdk.git
    $ git clone https://github.com/agilexrobotics/hunter_2_ros.git
	```
    ```
	$ cd agx_sdk
	$ git checkout hunter_2_ros
	```
    ```
    $ cd ~/catkin_ws
    $ catkin_make
    ```

3. Launch ROS nodes
 
* Start the base node for the real robot

    ```
    $ roslaunch hunter_bringup hunter_robot_base.launch
    ```
* Start the keyboard node to control the real robot

    ```
    $ roslaunch hunter_bringup hunter_teleop_keyboard.launch
    ```
    
**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 
