# Overview
`rqt_env` is a Python plugin for ROS that helps you configure ROS environment variables using a GUI (graphical user interface). `rqt_env` manage the ROS environment variables, with validation of environment variables and easy switching between robot configurations.

![rqt_env](https://github.com/hsarmiento/rqt_env/blob/master/screenshot.png)


# Motivation
When users set environment variables, they add new lines in different files for to persist values in Operating System. These files (e.g .bashrc, profile.d) can have different assignment for many applications. Then when users modify this files, they comment and uncomment this assignment and in some cases, they could edit or remove other variables different to ROS variables. Therefore, human errors is a common issue that can to affect variables in Operating Systems.
```
HISTSIZE=1000
HISTFILESIZE=2000
export ROS_MASTER_URI=http://192.189.0.1
export ROS_ROOT=/home/user/ros/ros
```

Also, users could to work with different robots (e.g PR2, Turtlebots) and these cases they comment and uncomment for current configuration:
```
export IP_WORKSTATION=171.180.0.1
export ROS_MASTER_URI=192.179.11.1
#export ROS_MASTER_URI=160.179.20.1
#export ROS_MASTER_URI=189.180.11.1
```

Then, the main goal of `rqt_env` is to avoid human errors when they configure ROS environment variables. So, for basic variables and set of variables for different robots, `rqt_env` allow add and remove these variables without using files as `.bashrc` or `profile.d`

# Installation

* Download source code unto your workspace
* Execute `catkin_make` in your workspace
* In terminal execute: `$ rosrun rqt_env rqt_env`

**Optional**

If appear `error on load_plugin`, execute in your terminal: 
```
$rm ~./config/ros.org/rqt_gui.ini
$rqt
```

# Contributors

We are MSc in Computer Science students of the Universidad de Chile. We are studying a course called "Software Engineering for Robotics" in which students’ projects consist of building or improving ROS tools.

**Contact**
* Teófilo Chambilla <tchambil@dcc.uchile.cl>
* Hernán Sarmiento <hsarmien@dcc.uchile.cl>

