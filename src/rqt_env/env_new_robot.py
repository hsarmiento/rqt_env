#!/usr/bin/env python
from __future__ import division
import os
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QHeaderView, QIcon, QMenu, QTreeWidgetItem, QWidget, QPushButton, QDialog
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException

class NewRobot(QDialog):
    def __init__(self, parent=None):
        super(NewRobot,self).__init__(parent)

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_env'), 'resource', 'NewRobotDialog.ui')
        loadUi(ui_file, self)