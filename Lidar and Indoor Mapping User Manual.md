# Lidar and Indoor Mapping User Manual

# System Requirements

## Ubuntu Mate 20.24

Please follow the [official guide](https://ubuntu-mate.org/raspberry-pi/install/#2-download-the-ubuntu-mate-image) from Ubuntu Mate to install Ubuntu Mate 20.04 on Raspberry Pi 4. You can find the image on this [link](https://releases.ubuntu-mate.org).

## ROS Noetic

Please follow the [official documentation](https://wiki.ros.org/noetic/Installation/Ubuntu) from ROS.org for installation guidance for ROS Noetic.

# RPLidar Installation

> https://github.com/robopeak/rplidar_ros

## Preparation
1. Add permission to the USB port. 

    ```bash
    $ sudo chmod 666 /dev/ttyUSB0
    ```

2. See what is connected to `/dev/ttyUSB0`, `/dev/ttyUSB1`, and `/dev/ttyUSB2` etc.

    ```bash
    $ dmesg | grep "tty"
    ```

## Install `rplidar_ros` package

1. In a `catkin_ws` workspace, `cd` into `/catkin_ws/src/`

    ```bash
    $ cd /catkin_ws/src
    ```

2. Then clone the git repo `git clone https://github.com/robopeak/rplidar_ros.git`

3. Under `/catkin_ws/` folder `catkin_make`

    ```bash
    $ roscore
    $ roslaunch rplidar_ros view_rplidar.launch
    ```

# Hector SLAM Installation

> https://github.com/tu-darmstadt-ros-pkg/hector_slam/tree/noetic-devel

## Install hector_slam package

1. In a `catkin_ws` workspace, `cd` into `/catkin_ws/src/`.

2. Then clone the git repo `git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git`

3. Modify several files
    * In file `/catkin_ws/src/hector_slam/hector_mapping/launch/mapping_default.launch`, change the following configurations.
        
        ```
        <!-- <arg name="base_frame" default="base_footprint"/> -->

        <arg name="base_frame" default="base_link"/>
        ```

        ```
        <!-- <arg name="odom_frame" default="nav"/> -->

        <arg name="odom_frame" default="base-link"/>
        ```

        ```
        <!-- <node pkg="tf" type="static_transform_publisher" name="map_nav_broadcaster" args="0 0 0 0 0 0 map nav 100"/> -->

        <node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0 0 0 0 base_link laser 100"/>
        ```

    * In file `/catkin_ws/src/hector_slam/hector_slam_launch/launch/tutorial.launch`, change the following configurations.

        ```
        <!-- <param name="/use_sim_time" value="true"/> -->

        <param name="/use_sim_time" value="false"/>
        ```

4. Under `/catkin_ws/` folder do `catkin_make`.

    ```bash
    $ sudo chmod 666 /dev/ttyUSB0
    $ roscore
    $ roslaunch rplidar_ros rplidar.launch
    $ roslaunch hector_slam_launch tutorial.launch
    ```

# Run Mapping

> WARNING! Due to the Lidar Mapping need to use lots of CPU resources, Raspberry Pi must be powered properly. If you are using a mobile charger, make sure it can deliver an output of 5V 3A.

## Check USB Port

1. Check the corresponding USB port for Roomba and Lidar. For simplicity, it is recommended to connect Lidar to Raspberry Pi first.

    ```bash
    dmesg | grep "tty"
    ```
    Make sure you see the following settings.
    ```
    cp210x converter → Lidar `ttyUSB0`
    FTDI USB Serial Device converter → Roomba `ttyUSB1`
    ```
    
2. Add executable permissions to both ports

    ```bash
    sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
    ```

## Control Roomba via Keyboard

```bash
$ cd code/
$ sudo python3 Create2_TetheredDrive.py

# Click Create → Connect
# Type in "dev/ttyUSB1" to connect the Roomba
# Use the keyboard to control the movement of the Roomba
```
    
## Run Hector Mapping
```bash
$ cd catkin_ws
$ roscore
$ roslaunch rplidar_ros rplidar.launch
$ roslaunch hector_slam_launch tutorial.launch
``` 
