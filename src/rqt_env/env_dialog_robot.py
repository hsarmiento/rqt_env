#!/usr/bin/env python
from __future__ import division
import os
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QHeaderView, QIcon, QMenu, QTreeWidgetItem, QWidget, QPushButton, QDialog, QMessageBox
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException
import xml.etree.ElementTree as ET
import sys
from .xml_info import XmlInfo 
from xml.dom import minidom

class DialogRobot(QDialog):
	_column_names_robot = ['variable', 'value']
	def __init__(self,alias=None):
		super(DialogRobot,self).__init__()
		self._alias = alias
		rp = rospkg.RosPack()
		ui_file = os.path.join(rp.get_path('rqt_env'), 'resource', 'NewRobotDialog.ui')
		loadUi(ui_file, self)
		self.treeWidgetRobot.sortByColumn(0, Qt.AscendingOrder)
		self.treeWidgetRobot.setEditTriggers(self.treeWidgetRobot.NoEditTriggers)
		# self.treeWidgetRobot.clicked.connect(self.show_click_row)
 
 
		# self.treeWidget.clicked.connect(self.show_click_row)
		header_robot=self.treeWidgetRobot.header()
		header_robot.setResizeMode(QHeaderView.ResizeToContents) 
		header_robot.setContextMenuPolicy(Qt.CustomContextMenu)
		# self.btnRemoveRobot.clicked.connect(self.click_btnRemoveRobot)
		self.txtAlias.setText(self._alias)

		#clicked buttons robots
		self.btnAddRobot.clicked.connect(self.click_btn_add_robot)
		self.btnSaveRobot.clicked.connect(self.click_btnSaveRobot)
		self.btnRemoveRobot.clicked.connect(self.click_btnRemoveRobot)
		self.btnCancelRobot.clicked.connect(self.click_btnCancelRobot)

		#initial button state
		self.btnSaveRobot.setEnabled(False)
		self.btnRemoveRobot.setEnabled(False)
		self.txtVariableRobot.setEnabled(False)
		self.txtValueRobot.setEnabled(False)
		self._column_index = {}
		self._variables = {}
		for column_name in self._column_names_robot:
			self._column_index[column_name] = len(self._column_index)
		self.refresh_variables()

  # def show_click_row(self):
  #       self.btnRemoveRobot.setEnabled(True)
  #       self.txtVariableRobot.setEnabled(True)
  #       self.txtValueRobot.setEnabled(True)
  #       item = self.treeWidgetRobot.currentItem()
  #       self.txtVariableRobot.setText(item.text(0))
  #       self.txtValueRobot.setText(item.text(1))

	def click_btn_add_robot(self):
		self.btnAddRobot.setEnabled(False)
		self.txtVariableRobot.setEnabled(True)
		self.txtValueRobot.setEnabled(True)
		self.btnSaveRobot.setEnabled(True)
		self.btnRemoveRobot.setEnabled(False)
		self.treeWidgetRobot.clearSelection()
		self.txtVariableRobot.setFocus()

	def click_btnSaveRobot(self):
		if self.txtAlias.text().strip() != "":
			if self.txtVariableRobot.text().strip() != "" and self.txtValueRobot.text().strip() != "" :
			    if not self.validate_item(self.txtVariableRobot.text()):
			    	dialog_xml = DialogXml()
			    	dialog_xml.add_variable_robot(self.txtAlias.text().strip(),self.txtVariableRobot.text(),self.txtValueRobot.text())
			    	message_instance = None
			    	variable_item = self._recursive_create_widget_items(self.treeWidgetRobot, self.txtVariableRobot.text(), self.txtValueRobot.text(), message_instance)
			    	self.btnSaveRobot.setEnabled(False)
			    	self.btnRemoveRobot.setEnabled(True)
			    	self.txtVariableRobot.setText("")
			    	self.txtValueRobot.setText("")
			    	self.txtVariableRobot.setEnabled(False)
			    	self.txtValueRobot.setEnabled(False)
			    	self.btnRemoveRobot.setEnabled(False)
			    	self.btnAddRobot.setEnabled(True)
			    	self.btnAddRobot.setFocus()
			    else:
			         QMessageBox.information(self, 'Variable exists',self.txtVariableRobot.text()+" exists in list")
		else:
			QMessageBox.information(self, 'Alias is empty',"Please, insert a value for alias")
			self.txtAlias.setFocus()

	def click_btnRemoveRobot(self):
		pass

	def click_btnCancelRobot(self):
		self.txtVariableRobot.setText("")
		self.txtValueRobot.setText("")
		self.btnSaveRobot.setEnabled(False)
		self.btnRemoveRobot.setEnabled(False)
		self.txtVariableRobot.setEnabled(False)
		self.txtValueRobot.setEnabled(False)


	def validate_item(self, item=None):
		root = self.treeWidgetRobot.invisibleRootItem()
		count = root.childCount()
		for i in range(count):
		    row = root.child(i)
		    if(row.text(0)==item):
		        return True
		return False

	def refresh_variables(self):
	    xml_dialog = DialogXml()
	    xml_dialog.openXml()
	    robot_variables = xml_dialog.getRobotVariables(self._alias)
	    new_topics = {}
	    for variable_name, variable_value in robot_variables:
	            # if topic is new or has changed its type
	        if variable_name not in self._variables or \
	           self._variables[variable_name]['variable'] != variable_value:
	            # create new TopicInfo
	            topic_info = 'ss'#opicInfo(topic_name, topic_type)
	            message_instance = None
	            variable_item = self._recursive_create_widget_items(self.treeWidgetRobot, variable_name, variable_value, message_instance)
	            new_topics[variable_name] = {
	               'item': variable_item,
	               'value': variable_value,
	            }

	        else:
	            # if topic has been seen before, copy it to new dict and
	            # remove it from the old one
	            new_topics[variable_name] = self._topics[variable_name]
	            del self._topics[variable_name]

	def _recursive_create_widget_items(self, parent, variable_name, variable_value, message):
	    # topic_text = variable_name
	    print variable_name,variable_value
	    item = TreeWidgetItem(variable_name, parent)
	    item.setText(self._column_index['variable'], variable_name)
	    item.setText(self._column_index['value'], variable_value) 
	    return item
 

class TreeWidgetItem(QTreeWidgetItem):
    def __init__(self, topic_name, parent=None):
        super(TreeWidgetItem, self).__init__(parent)
        self._topic_name = topic_name


class DialogXml(object):
	def __init__(self):
		self._root = None

	def openXml(self):
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml' 
		self._root = ET.parse(cpath).getroot()

	def getRobotVariables(self,alias):
		l = []
		for elem in self._root.iter(tag='robot'):
	 		if elem.attrib['id']==alias:
				for node in elem.iterfind('variable'):
					l.append([node.attrib['name'],node.attrib['value']])
		return l

	def add_variable_robot(self,alias,variable,value):
		self.openXml()
		is_new = True
		for elem in self._root.iter(tag='robot'):
			if elem.attrib['id'] == alias:
				new_element = ET.Element('variable',{'name':variable,'value':value,'deleted':'0'})
				elem.append(new_element)
				is_new = False
		if is_new:		
			for child in self._root:
				if child.tag == "robots":
						new_element = ET.Element('robot',{'id':alias,'status':'1','deleted':'0'})
						subElem = ET.SubElement(new_element, "variable", {'deleted':'0','name':variable,'value':value})
						ET.dump(new_element)
						child.append(new_element)
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml'
		ET.ElementTree(self._root).write(cpath)
		xml = minidom.parse(cpath)
		pretty_xml_as_string = xml.toprettyxml()
		f = open(cpath,'w')
		f.write(pretty_xml_as_string)
		f.close()


