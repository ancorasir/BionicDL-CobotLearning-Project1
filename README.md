# ROS integration for Franka Emika research robots

# BionicDL-CobotLearning-Project1
In this project, you must assimilate your work from previous exercises to successfully complete a tabletop pick and place operation using Franka in Gazebo.

The system is without perception and the pick list is given.

# Project Setup
For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the src directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/ancorasir/BionicDL-CobotLearning-Project1.git
```

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Build the project:
```sh
$ cd ~/catkin_ws
$ catkin build
```

Add following to your .bashrc file
```sh
export GAZEBO_MODEL_PATH=~/catkin_ws/src/BionicDL-CobotLearning-Project1/franka_description/models:$GAZEBO_MODEL_PATH
```

If you haven’t already, following line can be added to your .bashrc to auto-source all new terminals
```sh
source ~/catkin_ws/devel/setup.bash
```

# Run the demo:
```sh
roslaunch franka_gazebo franka.launch
```

![alt text](./images/easy_handeye.png)


# Some notes
1. The calibration result is not very good as show in Rviz. The aruco marker is actually lay on the table horizontally. The orientation of the handeye transformation is bad. The calibration result is good only in the center of the camera's view, which is commonly seen in practice.
