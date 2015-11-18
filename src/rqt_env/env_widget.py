#!/usr/bin/env python
from __future__ import division
import os
import sys
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QHeaderView, QIcon, QMenu, QTreeWidgetItem, QWidget, QPushButton, QMessageBox
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException
from .env_dialog_robot import DialogRobot,DialogXml
from .xml_info import XmlInfo 
 


class EnvWidget(QWidget):
    """
    main class inherits from the ui window class.
    You can specify the topics that the topic pane.
    EnvWidget.start must be called in order to update topic pane.
    """

    SELECT_BY_NAME = 0
    SELECT_BY_MSGTYPE = 1

    _column_names = ['topic', 'type', 'bandwidth', 'rate', 'value']
    #_column_names = ['variable', 'value']
    #_column_names_robot=['name','ip','status']
    _column_names_robot = ['topic', 'type', 'bandwidth', 'rate', 'value']
    def __init__(self, plugin=None, selected_topics=None, select_topic_type=SELECT_BY_NAME):
        """
        @type selected_topics: list of tuples.
        @param selected_topics: [($NAME_TOPIC$, $TYPE_TOPIC$), ...]
        @type select_topic_type: int
        @param select_topic_type: Can specify either the name of topics or by
                                  the type of topic, to filter the topics to
                                  show. If 'select_topic_type' argument is
                                  None, this arg shouldn't be meaningful.
        """
        super(EnvWidget, self).__init__()

        self._select_topic_type = select_topic_type

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_env'), 'resource', 'EnvWidget.ui')
        loadUi(ui_file, self)
        self._plugin = plugin
        #tree_widget ROS
        self.env_ros_tree_widget.sortByColumn(0, Qt.AscendingOrder)
        self.env_ros_tree_widget.clicked.connect(self.show_click_row)

        header = self.env_ros_tree_widget.header()
        header.setResizeMode(QHeaderView.ResizeToContents) 
        header.setContextMenuPolicy(Qt.CustomContextMenu)
     
        #tree_widget ROBOT
        self.env_robot_tree_widget.sortByColumn(0, Qt.AscendingOrder)
        self.env_robot_tree_widget.setEditTriggers(self.env_robot_tree_widget.NoEditTriggers)
        self.env_robot_tree_widget.clicked.connect(self.show_click_row)
        header_robot=self.env_robot_tree_widget.header()
        header_robot.setResizeMode(QHeaderView.ResizeToContents) 
        header_robot.setContextMenuPolicy(Qt.CustomContextMenu)

        #clicked buttons General       
        
        self.btnApply.clicked.connect(self.click_btnApply) 
        #clicked buttons ROS
        self.btnNewRos.clicked.connect(self.click_btnNewRos) 
        self.btnSaveRos.clicked.connect(self.click_btnSaveRos)
        self.btnRemoveRos.clicked.connect(self.click_btnRemoveRos)

        #clicked buttons robots
        self.btnAddRobot.clicked.connect(self.click_btnAddRobot)
        self.btnDetailsRobot.clicked.connect(self.click_btnDetailsRobot)
        self.btnSaveRobot.clicked.connect(self.click_btnSaveRobot)
        self.btnRemoveRobot.clicked.connect(self.click_btnRemoveRobot)

        self._selected_topics = selected_topics

        self._current_topic_list = []
        self._topics = {}
        self._tree_items = {}
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)

        self.refresh_env()

    def click_btnApply(self):
        quit_msg = "Are you sure you want to Apply this configuration?"
        reply = QMessageBox.question(self, 'Message', quit_msg, QMessageBox.Yes, QMessageBox.No)

        if reply == QMessageBox.Yes:
            print "OK"
            pass
        else:
            pass
    
   
    def click_btnNewRos(self):
        print "Test Click_btnNewRos"
        self.btnModifyRos.setEnabled(False)
        self.btnRemoveRos.setEnabled(False)
        self.btnSaveRos.setEnabled(True)
 
  
    def click_btnSaveRos(self):
        print "Test click_btnSaveRos"
        self.btnSaveRos.setEnabled(False)
        self.btnRemoveRos.setEnabled(True)
        self.btnModifyRos.setEnabled(True)


    def click_btnRemoveRos(self):
        # 
        print "Test click_btnRemoveRos s"
        quit_msg = "Are you sure you want to remove this element?"
        reply = QMessageBox.question(self, 'Message', quit_msg, QMessageBox.Yes, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self._recursive_delete_widget_items(self._tree_items['TURTLEBOT_BASE'])
        else:
            pass
                
    def click_btnAddRobot(self):
        print "Test click_btnAddRobot"
        q = DialogRobot()
        q.exec_()
        # sys.exit(App.exec_())

    def click_btnDetailsRobot(self):
        item = self.env_robot_tree_widget.currentItem()
        q = DialogRobot(item.text(0))
        q.exec_()

    def click_btnSaveRobot(self):
        print "Test click_btnSaveRobot"

    def click_btnRemoveRobot(self):
        print "Test click_btnRemoveRobot"
        quit_msg = "Are you sure you want to remove this element?"
        reply = QMessageBox.question(self, 'Message', quit_msg, QMessageBox.Yes, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.removeSelectedItemWidgetTree()
        else:
            pass

    def removeSelectedItemWidgetTree(self):
        root = self.env_robot_tree_widget.invisibleRootItem()
        for item in self.env_robot_tree_widget.selectedItems():
            (item.parent() or root).removeChild(item)

    def show_click_row(self,item=None,column=None):
        print "item" , item
        print "col" , column 
    def set_topic_specifier(self, specifier):
        self._select_topic_type = specifier

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
        self._timer_refresh_topics.start(1000)

    # @Slot()
    def refresh_env(self):
        self._xml_info = XmlInfo()
        self._xml_info.openXml()
        general_list = self._xml_info.getGeneralVariables()
        robots_list = self._xml_info.getRobots()
        print robots_list
        # topic_list= [['TURTLEBOT_BASE', 'kobuki'], ['ROS_HOSTNAME', 'localhost']]
        new_topics = {}
        for topic_name, topic_type in general_list:
                # if topic is new or has changed its type
            if topic_name not in self._topics or \
               self._topics[topic_name]['type'] != topic_type:
                # create new TopicInfo
                topic_info = 'ss'#opicInfo(topic_name, topic_type)
                message_instance = None
                topic_item = self._recursive_create_widget_items(self.env_ros_tree_widget, topic_name, topic_type, message_instance)
                new_topics[topic_name] = {
                   'item': topic_item,
                   'info': topic_info,
                   'type': topic_type,
                }

            else:
                # if topic has been seen before, copy it to new dict and
                # remove it from the old one
                new_topics[topic_name] = self._topics[topic_name]
                del self._topics[topic_name]

        new_topics_robots = {}
        for topic_name, topic_type,status in robots_list:
                # if topic is new or has changed its type
            if topic_name not in self._topics or \
               self._topics[topic_name]['type'] != topic_type:
                # create new TopicInfo
                topic_info = 'ss'#opicInfo(topic_name, topic_type)
                message_instance = None
                topic_item = self._recursive_create_widget_items(self.env_robot_tree_widget, topic_name, topic_type, message_instance,status)
                new_topics_robots[topic_name] = {
                   'item': topic_item,
                   'info': topic_info,
                   'type': topic_type,
                }
                print topic_item

            else:
                # if topic has been seen before, copy it to new dict and
                # remove it from the old one
                new_topics_robots[topic_name] = self._topics[topic_name]
                del self._topics[topic_name]

    def _recursive_create_widget_items(self, parent, topic_name, type_name, message,status=None):
        topic_text = topic_name
        item = TreeWidgetItem(self._toggle_monitoring, topic_name, parent,status)
        item.setText(self._column_index['topic'], topic_text)
        item.setText(self._column_index['type'], type_name) 
        return item
    def _toggle_monitoring(self, topic_name):
        item = self._tree_items[topic_name]
        if item.checkState(0):
            self._topics[topic_name]['info'].start_monitoring()
        else:
            self._topics[topic_name]['info'].stop_monitoring()

    def _recursive_delete_widget_items(self, item):
        def _recursive_remove_items_from_tree(item):
            for index in reversed(range(item.childCount())):
                _recursive_remove_items_from_tree(item.child(index))
            topic_name = item.data(0, Qt.UserRole)
            del self._tree_items[topic_name]
        _recursive_remove_items_from_tree(item)
        item.parent().removeChild(item)
    def shutdown_plugin(self):
        for topic in self._topics.values():
            topic['info'].stop_monitoring()
        #self._timer_refresh_topics.stop()

    def set_selected_topics(self, selected_topics):
        """
        @param selected_topics: list of tuple. [(topic_name, topic_type)]
        @type selected_topics: []
        """
        rospy.logdebug('set_selected_topics topics={}'.format(len(selected_topics)))
        self._selected_topics = selected_topics

    # TODO(Enhancement) Save/Restore tree expansion state
    def save_settings(self, plugin_settings, instance_settings):
        header_state = self.env_ros_tree_widget.header().saveState()
        instance_settings.set_value('tree_widget_header_state', header_state)

    def restore_settings(self, pluggin_settings, instance_settings):
        if instance_settings.contains('tree_widget_header_state'):
            header_state = instance_settings.value('tree_widget_header_state')
            if not self.env_ros_tree_widget.header().restoreState(header_state):
                rospy.logwarn("rqt_topic: Failed to restore header state.")


class TreeWidgetItem(QTreeWidgetItem):
    def __init__(self, check_state_changed_callback, topic_name, parent=None,status=None):
        super(TreeWidgetItem, self).__init__(parent)
        self._check_state_changed_callback = check_state_changed_callback
        self._topic_name = topic_name
        if status == '1':
            self.setCheckState(2, Qt.Checked)
        elif status == '0':
            self.setCheckState(2, Qt.Unchecked) 

    def setData(self, column, role, value):
        if role == Qt.CheckStateRole:
            state = self.checkState(column)
        super(TreeWidgetItem, self).setData(column, role, value)
        if role == Qt.CheckStateRole and state != self.checkState(column):
            self._check_state_changed_callback(self._topic_name)





