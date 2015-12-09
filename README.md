# Overview
`rqt_env` is a Python plugin for ROS that helps you configure ROS environment variables using a GUI (graphical user interface). `rqt_env` allows add and to select robot independently though GUI plugin. Also `rqt_env` allows add required environment variables as `ROS_ROOT` or `ROS_MASTER_URI`.

# Motivation
When users set environment variables, they add new lines in different files for to persist values in Operating System. These files (e.g .bashrc, profile.d) can have different assignment for many applications. Then when users modify this files, they comment and uncomment this assignment and in some cases they could edit or remove other variables different to ROS variables. Therefore, human errors is a common issue that can to affect variables in Operating Systems.

Also, users could to work with different robots (e.g PR2, Turtlebots) and these cases they comment and uncomment for current configuration:

```
#export dasd
export dasdasd

```

# Installation

# Contributors



