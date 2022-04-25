# NVILIDAR ROS DRIVER(V1.0.1)


## How to [install ROS](http://wiki.ros.org/cn/ROS/Installation)

[ubuntu](http://wiki.ros.org/cn/Installation/Ubuntu)

[windows](http://wiki.ros.org/Installation/Windows)

## How to Create a ROS workspace

[Create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

you also can with this:

    1)  $mkdir -p ~/nvilidar_ros_ws/src
        $cd ~/nvilidar_ros_ws/src
    2)  $cd..
    3)  $catkin_make
    4)  $source devel/setup.bash
    5)  echo $ROS_PACKAGE_PATH
        /home/user/nvilidar_ws/src:/opt/ros/kinetic/share


## How to build NVILiDAR ROS Package

    1) Clone this project to your catkin's workspace src folder
    	(1). git clone https://github.com/nvilidar/     nvilidar_ros.git  
          or
          git clone https://gitee.com/nvilidar/nvilidar_ros.git
    	(2). git chectout master
    2) Copy the ros source file to the "~/nvilidar_ros_ws/src"
    2) Running "catkin_make" to build nvilidar_node and nvilidar_client
    3) Create the name "/dev/nvilidar" to rename serialport
    --$ roscd nvilidar_ros/startup
    --$ sudo chmod 777 ./*
    --$ sudo sh initenv.sh


## How to Run NVILIDAR ROS Package
#### 1. Run NVILIDAR node and view in the rviz
------------------------------------------------------------
	roslaunch nvilidar_ros lidar_view.launch

#### 2. Run NVILIDAR node and view using test application
------------------------------------------------------------
	roslaunch nvilidar_ros lidar.launch

	rosrun nvilidar_ros nvilidar_client


