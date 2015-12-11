# Overview
`rqt_env` is a Python plugin for ROS that helps you configure ROS environment variables using a GUI (graphical user interface). `rqt_env` manage the ROS environment variables, with validation of environment variables and easy switching between robot configurations.

![rqt_env](https://github.com/hsarmiento/rqt_env/blob/master/screenshot.png)


# Motivation

When users set environment variables, they add new lines in different files in order to persist values in Operating System. These files (e.g .bashrc, profile.d) can have different assignments for many applications. When users modify these files, they comment and uncomment this assignment and in some cases, they could edit or remove other variables different to ROS variables. Therefore, human error is a common issue that can affect variables in Operating Systems.

```
HISTSIZE=1000
HISTFILESIZE=2000
export ROS_MASTER_URI=http://192.189.0.1
export ROS_ROOT=/home/user/ros/ros
```
Also, users could to work with different robots (e.g PR2, Turtlebots) and these cases comment and uncomment for current configuration:

```
export IP_WORKSTATION=171.180.0.1
export ROS_MASTER_URI=192.179.11.1
#export ROS_MASTER_URI=160.179.20.1
#export ROS_MASTER_URI=189.180.11.1
```
Then, the main goal of `rqt_env` is to avoid human errors when they configure ROS environment variables. So, for basic variables and set of variables for different robots, `rqt_env` allow for addition and removal of these variables without using files as `.bashrc` or `profile.d`.

# Installation

* Download source code unto your workspace
* Execute `catkin_make` in your workspace
* In terminal execute: `$ rosrun rqt_env rqt_env`

**Optional**

If `PluginManager._load_plugin() could not load plugin "rqt_env/Env"` appear, execute in your terminal: 
```
$rm ~./config/ros.org/rqt_gui.ini
$rqt
```

# Contributors

Currently, we're studying a course called "Software Engineering for Robotics," in order to learn how to build and improve ROS tools.

**Contact**
* Teófilo Chambilla <tchambil@dcc.uchile.cl>
* Hernán Sarmiento <hsarmien@dcc.uchile.cl>

