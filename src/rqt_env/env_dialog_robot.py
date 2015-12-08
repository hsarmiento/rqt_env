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
from PyQt4.QtCore import SIGNAL, QObject
from urlparse import urlparse

class DialogRobot(QDialog):
	_column_names_robot = ['variable', 'value']
	def __init__(self,alias=None):
		super(DialogRobot,self).__init__()
		self._alias = alias
		self._temp_alias = alias
		self.rp =rospkg.RosPack()
		ui_file = os.path.join(self.rp.get_path('rqt_env'), 'resource', 'NewRobotDialog.ui')
		loadUi(ui_file, self)
		self.treeWidgetRobot.sortByColumn(0, Qt.AscendingOrder)
		self.treeWidgetRobot.setEditTriggers(self.treeWidgetRobot.NoEditTriggers)
		self.treeWidgetRobot.clicked.connect(self.click_row)

		header_robot=self.treeWidgetRobot.header()
		header_robot.setResizeMode(QHeaderView.ResizeToContents) 
		header_robot.setContextMenuPolicy(Qt.CustomContextMenu)
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

		self.connect(self, SIGNAL('triggered()'), self.closeEvent)

	def click_row(self):
		self.btnRemoveRobot.setEnabled(True)
		self.txtVariableRobot.setEnabled(False)
		self.txtValueRobot.setEnabled(True)
		item = self.treeWidgetRobot.currentItem()
		self.txtVariableRobot.setText(item.text(0))
		self.txtValueRobot.setText(item.text(1))
		self.btnSaveRobot.setEnabled(True)
		self.btnAddRobot.setEnabled(True)
		if item.text(0)=="ROS_MASTER_URI" or item.text(0)=="ROS_HOSTNAME":
			self.btnRemoveRobot.setEnabled(False)

	def closeEvent(self, event):
		uri,hostname = self.validate_uri_hostname()
		if uri and hostname:
			self.destroy()
		elif not uri and not hostname:
			event.ignore()
			QMessageBox.information(self, 'ROS_HOSTNAME and ROS_MASTER_URI not exists',"You must add ROS_HOSTNAME AND ROS_MASTER_URI to initial variables")
		elif not uri:
			event.ignore()
			QMessageBox.information(self, 'ROS_MASTER_URI not exists',"You must add ROS_MASTER_URI to initial variables")
		elif not hostname:
			event.ignore()
			QMessageBox.information(self, 'ROS_HOSTNAME not exists',"You must add ROS_HOSTNAME to initial variables")

  	def validate_uri_hostname(self):
	    root = self.treeWidgetRobot.invisibleRootItem()
	    count = root.childCount()
	    uri = False
	    hostname = False
	    for i in range(count):
	        row = root.child(i)
	        if row.text(0)=='ROS_MASTER_URI':
	            uri = True
	        if row.text(0)=='ROS_HOSTNAME':
	        	hostname = True
	    return (uri,hostname)


  	def validate_ip(self,s):
	    a = s.split('.')
	    if len(a) != 4:
	        return False
	    for x in a:
	        if not x.isdigit():
	            return False
	        i = int(x)
	        if i < 0 or i > 255:
	            return False
	    return True

	def validate_hostname_value(self,s):
		if not self.validate_ip(s):
			if s.split(':')[0].lower() == 'localhost':
				return True
		else:
			return True
		return False

	def click_btn_add_robot(self):
		self.btnAddRobot.setEnabled(False)
		self.txtVariableRobot.setEnabled(True)
		self.txtValueRobot.setEnabled(True)
		self.btnSaveRobot.setEnabled(True)
		self.btnRemoveRobot.setEnabled(False)
		self.treeWidgetRobot.clearSelection()
		self.txtVariableRobot.setFocus()
		self.txtVariableRobot.setText("")
		self.txtValueRobot.setText("")

	def click_btnSaveRobot(self):
		if self.txtVariableRobot.text() == 'ROS_MASTER_URI':
			parse = urlparse(self.txtValueRobot.text())
			if not self.validate_ip(parse.netloc.split(':')[0]):
				QMessageBox.information(self, 'ROS_MASTER_URI format',"ROS_MASTER_URI must have http://")
				return False
		if self.txtVariableRobot.text()=='ROS_HOSTNAME':
			if not self.validate_hostname_value(self.txtValueRobot.text().split(":")[0]):
				QMessageBox.information(self, 'ROS_HOSTNAME format',"ROS_HOSTNAME must have localhost or IP")
				return False
			else:
				if self.txtValueRobot.text().split(":")[0].lower()=="localhost":
					self.txtValueRobot.setText(self.txtValueRobot.text().lower())
		if self.txtVariableRobot.isEnabled():
			if self.txtAlias.text().strip() != "":
				if self.txtVariableRobot.text().strip() != "" and self.txtValueRobot.text().strip() != "" :
				    if not self.exist_item(self.txtVariableRobot.text()):
				    	dialog_xml = DialogXml()					    	
				     	dialog_xml.add_variable_robot(self.txtAlias.text().strip(),self.txtVariableRobot.text(),self.txtValueRobot.text())
				    	message_instance = None
				    	variable_item = self._recursive_create_widget_items(self.treeWidgetRobot, self.txtVariableRobot.text(), self.txtValueRobot.text(), message_instance)
				    else:
				         QMessageBox.information(self, 'Variable exists',self.txtVariableRobot.text()+" exists in list")
			else:
				QMessageBox.information(self, 'Alias is empty',"Please, insert a value for alias")
				self.txtAlias.setFocus()
		else: 
			xml_dialog = DialogXml()
			xml_dialog.modify_variable_robot(self.txtVariableRobot.text(),self.txtValueRobot.text(),self.txtAlias.text().strip())
			self.treeWidgetRobot.clear()
  			self.refresh_variables()

		self.btnSaveRobot.setEnabled(False)
		self.btnRemoveRobot.setEnabled(True)
		self.txtVariableRobot.setText("")
		self.txtValueRobot.setText("")
		self.txtVariableRobot.setEnabled(False)
		self.txtValueRobot.setEnabled(False)
		self.btnRemoveRobot.setEnabled(False)
		self.btnAddRobot.setEnabled(True)
		self.btnAddRobot.setFocus()

	def click_btnRemoveRobot(self):
		quit_msg = "Are you sure you want to remove this element?"
		reply = QMessageBox.question(self, 'Message', quit_msg, QMessageBox.Yes, QMessageBox.No)
		item = self.treeWidgetRobot.currentItem()
		if reply == QMessageBox.Yes:
			xml_dialog = DialogXml()
			xml_dialog.modify_deleted_status_variable(item.text(0),self.txtAlias.text().strip())
			self.remove_selected_item_widgetTree()
			self.txtVariableRobot.setText("")
			self.txtValueRobot.setText("")
			self.txtVariableRobot.setEnabled(True)
			self.btnRemoveRobot.setEnabled(False)
			self.btnAddRobot.setEnabled(True)
			self.btnAddRobot.setFocus()

	def click_btnCancelRobot(self):
		self.txtVariableRobot.setText("")
		self.txtValueRobot.setText("")
		self.btnSaveRobot.setEnabled(False)
		self.btnRemoveRobot.setEnabled(False)
		self.txtVariableRobot.setEnabled(False)
		self.txtValueRobot.setEnabled(False)
		self.btnAddRobot.setEnabled(True)
 

	def exist_item(self, item=None):
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
	        if variable_name not in self._variables or \
	           self._variables[variable_name]['variable'] != variable_value:
	            message_instance = None
	            variable_item = self._recursive_create_widget_items(self.treeWidgetRobot, variable_name, variable_value, message_instance)
	            # new_topics[variable_name] = {
	            #    'item': variable_item,
	            #    'value': variable_value,
	            # }

	        else:
	            new_topics[variable_name] = self._topics[variable_name]
	            del self._topics[variable_name]

	def _recursive_create_widget_items(self, parent, variable_name, variable_value, message):
	    item = TreeWidgetItem(variable_name, parent)
	    item.setText(self._column_index['variable'], variable_name)
	    item.setText(self._column_index['value'], variable_value) 
	    return item

	def remove_selected_item_widgetTree(self):
	    root = self.treeWidgetRobot.invisibleRootItem()
	    for item in self.treeWidgetRobot.selectedItems():
	        (item.parent() or root).removeChild(item)
	       

class TreeWidgetItem(QTreeWidgetItem):
    def __init__(self, topic_name, parent=None):
        super(TreeWidgetItem, self).__init__(parent)
        self._topic_name = topic_name

class DialogXml(object):
	def __init__(self):
		self._root = None
		self.rp =rospkg.RosPack()

	def openXml(self):
		cpath = os.path.join(self.rp.get_path('rqt_env'),'resource','env.xml') 
		self._root = ET.parse(cpath).getroot()

	def getRobotVariables(self,alias):
		l = []
		for elem in self._root.iter(tag='robot'):
	 		if elem.attrib['id']==alias:
				for node in elem.iterfind('variable'):
					if node.attrib['deleted'] == '0':
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
						#ET.dump(new_element)
						child.append(new_element)
		cpath = os.path.join(self.rp.get_path('rqt_env'),'resource','env.xml') 
		ET.ElementTree(self._root).write(cpath)
		xml = minidom.parse(cpath)
		pretty_xml_as_string = xml.toprettyxml()
		f = open(cpath,'w')
		f.write(pretty_xml_as_string)
		f.close()

	def modify_variable_robot(self,variable,value,alias):
		self.openXml()
		for elem in self._root.iter(tag='robot'):
  			if elem.attrib["id"]==alias:
  				for node in elem.iterfind('variable'):
  					if node.attrib['name'] == variable:
						node.set('value',value)
		cpath = os.path.join(self.rp.get_path('rqt_env'),'resource','env.xml') 
		ET.ElementTree(self._root).write(cpath)

	def remove_robot_variable(self,variable,alias):
		self.openXml()
  		for elem in self._root.iter(tag='robot'):
  			if elem.attrib["id"]==alias:
	   			for node in elem.iterfind('variable'):   				
   					for child in node.getiterator():
   						if child.attrib['name']==variable:
   							elem.remove(node) 
		cpath = os.path.join(self.rp.get_path('rqt_env'),'resource','env.xml') 
		ET.ElementTree(self._root).write(cpath)

	def remove_robot_list_variable(self,alias):
		self.openXml()
  		for elem in self._root.iter(tag='robots'):
   			for node in elem.iterfind('robot'):
   				if node.attrib['id']==alias:
   					elem.remove(node) 
		cpath = os.path.join(self.rp.get_path('rqt_env'),'resource','env.xml') 
		ET.ElementTree(self._root).write(cpath)

	def remove_asociative_robot_variable(self,d):
		for robot,variables in d.items():
			for variable in variables:
				self.remove_robot_variable(variable,robot)


	def modify_deleted_status_variable(self,variable,alias):
		self.openXml()
		for elem in self._root.iter(tag='robot'):
  			if elem.attrib["id"]==alias:
  				for node in elem.iterfind('variable'):
  					if node.attrib['name'] == variable:
						node.set('deleted','1')
		cpath = os.path.join(self.rp.get_path('rqt_env'),'resource','env.xml') 
		ET.ElementTree(self._root).write(cpath)

	def get_deleted_variable_robot(self):
		c=set()
		self.openXml()
		for elem in self._root.iter(tag='robots'):
  			for node in elem.iterfind('robot'):
  					if node.attrib['deleted'] == "1":
  						for child in node.iterfind("variable"):
  							c.add(child.attrib['name'])
  					else: 
  						for child in node.iterfind("variable"):
  							if child.attrib["deleted"]=="1":
  								c.add(child.attrib["name"])
		return list(c)

  	def get_deleted_robot(self):
  		l = []
  		self.openXml()
		for elem in self._root.iter(tag='robots'):
  			for node in elem.iterfind('robot'):
  					if node.attrib['deleted'] == "1":
						l.append(node.attrib['id'])
		return l

	def get_general_variable_robot(self):
		l = []
		self.openXml()
		for elem in self._root.iter(tag='robots'):
  			for node in elem.iterfind('robot'):
  					if node.attrib['status'] == "1":
  						for child in node.iterfind("variable"):
  							if child.attrib['deleted'] == "0":
	  							l.append((child.attrib['name'],child.attrib['value']))
	  							alias = node.attrib['id']
 		return l,alias


 	def get_asociative_robot_variable(self):
 		d = {}
		self.openXml()
		for elem in self._root.iter(tag='robots'):
  			for node in elem.iterfind('robot'):
  					if node.attrib['deleted'] == "0":
						for child in node.iterfind("variable"):
  							if child.attrib["deleted"]=="1":
  								if node.attrib['id'] not in d:
  									d[node.attrib['id']] = set()
  								d[node.attrib['id']].add(child.attrib["name"])
		return d

	


