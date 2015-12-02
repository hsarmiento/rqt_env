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
from .env_os import EnvOs
 


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

        self.env_robot_tree_widget.itemClicked.connect(self.check_status)
 
      
        header_robot=self.env_robot_tree_widget.header()
        header_robot.setResizeMode(QHeaderView.ResizeToContents) 
        header_robot.setContextMenuPolicy(Qt.CustomContextMenu)

        #clicked buttons APPLY (all windows)         
        self.btnApply.clicked.connect(self.click_btn_apply) 

        #clicked buttons ROS
        self.btnNewRos.clicked.connect(self.click_btn_new_ros) 
        self.btnSaveRos.clicked.connect(self.click_btnSaveRos)
        self.btnRemoveRos.clicked.connect(self.click_btnRemoveRos)
        self.btnCancelRos.clicked.connect(self.click_btn_cancel_ros)

        #clicked buttons robots
        self.btnAddRobot.clicked.connect(self.click_btnAddRobot)
        self.btnDetailsRobot.clicked.connect(self.click_btnDetailsRobot)
        self.btnSaveRobot.clicked.connect(self.click_btnSaveRobot)
        self.btnRemoveRobot.clicked.connect(self.click_btnRemoveRobot)

        #clicked tabs
        self.tabWidget.currentChanged.connect(self.click_tab_ros)

        #init state general button
        self.btnSaveRos.setEnabled(False)
        self.btnRemoveRos.setEnabled(False)
        self.txtVariableRos.setEnabled(False)
        self.txtValueRos.setEnabled(False)

        self._selected_topics = selected_topics

        self._current_topic_list = []
        self._topics = {}
        self._tree_items = {}
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)

        self.refresh_env()

           

    def check_status(self):
        item1 = self.env_robot_tree_widget.currentItem()
        if item1.checkState(0) == Qt.Checked:
            print('item 1 is checked')
      

    def click_tab_ros(self,i):
        if i == 0:
            self.env_ros_tree_widget.clear()
            self.refresh_env()
        else:
            self.env_robot_tree_widget.clear()
            self.refresh_env()


    def show_click_row(self):
        self.btnRemoveRos.setEnabled(True)
        self.txtVariableRos.setEnabled(False)
        self.txtValueRos.setEnabled(True)
        item = self.env_ros_tree_widget.currentItem()
        self.txtVariableRos.setText(item.text(0))
        self.txtValueRos.setText(item.text(1))
        self.btnSaveRos.setEnabled(True)
        # return True


    def clear_checked(self):
        root = self.env_robot_tree_widget.invisibleRootItem()
        count = root.childCount()
        for i in range(count):
            folder = root.child(i)
            folder.setCheckState(2, Qt.Unchecked)
    
    def validate_item(self, item=None):
        root = self.env_ros_tree_widget.invisibleRootItem()
        count = root.childCount()
        for i in range(count):
            row = root.child(i)
            if(row.text(0)==item):
                return True
        return False
        

    def click_btn_apply(self):
        self.clear_checked()
        quit_msg = "Are you sure you want to Apply this configuration?"
        reply = QMessageBox.question(self, 'Message', quit_msg, QMessageBox.Yes, QMessageBox.No)
        if reply == QMessageBox.Yes:
            xml_info = XmlInfo()
            env_os = EnvOs()
            dialog_xml = DialogXml()
            deleted_general_items = xml_info.get_deleted_general_variable()  #get deleted general items (deleted status = 1 in xml)
            variable_general_items = xml_info.get_general_varibale()
            dialog_xml.get_deleted_variable_robot()
            deleted_robots_items=dialog_xml.get_deleted_variable_robot()
            variable_robot_items,active_robot=dialog_xml.get_general_variable_robot()
            
            deleted_robot=dialog_xml.get_deleted_robot()
            asociative_variable_robot = dialog_xml.get_asociative_robot_variable()

            env_os.unset_to_htbash(deleted_robots_items+deleted_robots_items)
            env_os.export_to_general_htbash(variable_general_items)
            env_os.export_to_robot_htbash(variable_robot_items,active_robot)
            dialog_xml.remove_asociative_robot_variable(asociative_variable_robot)

            for item in deleted_robot:
                dialog_xml.remove_robot_list_variable(item)

            for item in deleted_general_items:   
                xml_info.remove_general_variable(item)   
            
            self.lblmsg.setText("write file .htbash successfully")
        else:
             pass
    
    def click_btn_new_ros(self): 
        self.btnNewRos.setEnabled(False)
        self.txtVariableRos.setEnabled(True)
        self.txtValueRos.setEnabled(True)
        self.btnSaveRos.setEnabled(True)
        self.btnRemoveRos.setEnabled(False)
        self.env_ros_tree_widget.clearSelection()
        self.txtVariableRos.setFocus()
        self.txtVariableRos.setText("")
        self.txtValueRos.setText("")
  
    def click_btnSaveRos(self):
        if self.txtVariableRos.isEnabled():
            if self.txtVariableRos.text().strip() != "" and self.txtValueRos.text().strip() != "" :
                if not self.validate_item(self.txtVariableRos.text()):
                    xml_info = XmlInfo()
                    xml_info.add_variable_general(self.txtVariableRos.text(),self.txtValueRos.text())
                    message_instance = None
                    self._recursive_create_widget_items(self.env_ros_tree_widget, self.txtVariableRos.text(), self.txtValueRos.text(), message_instance)
                else:
                     QMessageBox.information(self, 'Variable exists',self.txtVariableRos.text()+" exists in list")
        else:
            message_instance = None
            xml_info = XmlInfo()
            xml_info.modify_variable_general(self.txtVariableRos.text(),self.txtValueRos.text())
            self.env_ros_tree_widget.clear()
            self.refresh_env()
        self.btnSaveRos.setEnabled(False)
        self.btnRemoveRos.setEnabled(True)
        self.txtVariableRos.setText("")
        self.txtValueRos.setText("")
        self.txtVariableRos.setEnabled(False)
        self.txtValueRos.setEnabled(False)
        self.btnRemoveRos.setEnabled(False)
        self.btnNewRos.setEnabled(True)


    def click_btnRemoveRos(self):
        quit_msg = "Are you sure you want to remove this element?"
        reply = QMessageBox.question(self, 'Message', quit_msg, QMessageBox.Yes, QMessageBox.No)
        item = self.env_ros_tree_widget.currentItem()
        if reply == QMessageBox.Yes:
            xml_info = XmlInfo()
            xml_info.modify_deleted_status_general_variable(item.text(0))
            # xml_info.remove_general_variable(item.text(0))   #not yet, just apply when apply button is clicked
            self.remove_Selected_Item_WidgetTree_ros()
            self.txtVariableRos.setText("")
            self.txtValueRos.setText("")
            self.txtVariableRos.setEnabled(True)
            self.btnRemoveRos.setEnabled(False)
            self.btnNewRos.setFocus()
            self.btnSaveRos.setEnabled(False)

        else:
            pass

    def click_btn_cancel_ros(self):
        self.txtVariableRos.setText("")
        self.txtValueRos.setText("")
        self.btnNewRos.setEnabled(True)
        self.btnSaveRos.setEnabled(False)
        self.txtVariableRos.setEnabled(False)
        self.txtValueRos.setEnabled(False)

                
    def click_btnAddRobot(self):
        q = DialogRobot()
        q.exec_()
        self.env_robot_tree_widget.clear()
        self.refresh_env() 

    def click_btnDetailsRobot(self):
        item = self.env_robot_tree_widget.currentItem()
        if not item==None:
            q = DialogRobot(item.text(0))
            q.exec_()
            self.env_robot_tree_widget.clear()
            self.refresh_env()

    def click_btnSaveRobot(self):
        pass

    def click_btnRemoveRobot(self):
      
        quit_msg = "Are you sure you want to remove this element?"
        reply = QMessageBox.question(self, 'Message', quit_msg, QMessageBox.Yes, QMessageBox.No)
        item = self.env_robot_tree_widget.currentItem()
        if reply == QMessageBox.Yes:
            xml_info = XmlInfo()
            xml_info.remove_robot_list_variable(item.text(0))
            self.removeSelectedItemWidgetTree()
        else:
            pass

    def remove_Selected_Item_WidgetTree_ros(self):
        root = self.env_ros_tree_widget.invisibleRootItem()
        for item in self.env_ros_tree_widget.selectedItems():
            (item.parent() or root).removeChild(item)

    def removeSelectedItemWidgetTree(self):
        root = self.env_robot_tree_widget.invisibleRootItem()
        for item in self.env_robot_tree_widget.selectedItems():
            (item.parent() or root).removeChild(item)
 
    def get_selected_item(self,tree_widget):
        pass
        # root = tree_widget.invisibleRootItem()
        # print tree_widget.selectedItems()

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
        #if role == Qt.CheckStateRole and state != self.checkState(column):
         #   self._check_state_changed_callback(self._topic_name)





