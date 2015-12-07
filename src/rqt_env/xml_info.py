#!/usr/bin/env python
import xml.etree.ElementTree as ET 
import os
import sys
from xml.dom import minidom

class XmlInfo(object):
	
	def __init__(self):
		self._root = None

	def openXml(self):
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml' 
		self._root = ET.parse(cpath).getroot()

	def getGeneralVariables(self):
		l = []
		for elem in self._root.iter(tag='general'):
			for node in elem.iterfind('variable'):
				if node.attrib['deleted'] == '0':
					l.append([node.attrib['name'],node.attrib['value']])
		return l

	def getRobots(self):
		l = []
		for elem in self._root.iter(tag='robot'):
			if elem.attrib['deleted'] == '0':
				t = [elem.attrib['id'],None,None,elem.attrib['status']]
				for node in elem.iterfind('variable'):
					if node.attrib['name'] == 'ROS_MASTER_URI':
						t[1] = node.attrib['value']
					if node.attrib['name'] == 'ROS_HOSTNAME':
						t[2] = node.attrib['value']
				l.append(t)
		return l
	

	def modify_deleted_status_general_variable(self,variable):
		self.openXml()
  		for elem in self._root.iter(tag='general'):
   			for node in elem.iterfind('variable'):
   				for child in node.getiterator():
   					if child.attrib['name']==variable:
   						child.set('deleted','1')
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml'
		ET.ElementTree(self._root).write(cpath)

	def remove_general_variable(self,variable):
		self.openXml()
  		for elem in self._root.iter(tag='general'):
   			for node in elem.iterfind('variable'):
   				for child in node.getiterator():
   					if child.attrib['name']==variable:
   						elem.remove(node) 
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml'
		ET.ElementTree(self._root).write(cpath)
	
	def save_selected_robot(self,alias,value):
		self.openXml()
  		for elem in self._root.iter(tag='robots'):
   			for node in elem.iterfind('robot'):
   				if node.attrib['id']==alias:
   					node.set('status',value)
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml'
		ET.ElementTree(self._root).write(cpath)

	def modify_deleted_robot_list_variable(self,alias):
		self.openXml()
  		for elem in self._root.iter(tag='robots'):
   			for node in elem.iterfind('robot'):
   				if node.attrib['id']==alias:
   					node.set('deleted','1')
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml'
		ET.ElementTree(self._root).write(cpath)

	def remove_robot_list_variable(self,variable):
		self.openXml()
  		for elem in self._root.iter(tag='robots'):
   			for node in elem.iterfind('robot'):
   				if node.attrib['id']==variable:
   					elem.remove(node) 
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml'
		ET.ElementTree(self._root).write(cpath) 
 	def add_variable_general(self,variable,value):
		self.openXml()
		for child in self._root:
			if child.tag == "general":
					new_element = ET.Element('variable',{'name':variable,'value':value,'deleted':'0'})
					child.append(new_element)
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml'
		ET.ElementTree(self._root).write(cpath)
		xml = minidom.parse(cpath)
		pretty_xml_as_string = xml.toprettyxml()
		f = open(cpath,'w')
		f.write(pretty_xml_as_string)
		f.close()

	def modify_variable_general(self,variable,value):
		self.openXml()
  		for elem in self._root.iter(tag='general'):
  			for node in elem.iterfind('variable'):
				if node.attrib['name'] == variable:
					node.set('value',value)
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml'
		ET.ElementTree(self._root).write(cpath)

	def get_deleted_general_variable(self):
		l = []
		self.openXml()
  		for elem in self._root.iter(tag='general'):
  			for node in elem.iterfind('variable'):
  				if node.attrib['deleted'] == '1':
  					l.append(node.attrib['name'])
		return l 
	def get_general_varibale(self):
		l = []
		self.openXml()
  		for elem in self._root.iter(tag='general'):
  			for node in elem.iterfind('variable'):
  				if node.attrib['deleted'] == '0':
  					l.append((node.attrib['name'],node.attrib['value']))
  		return l

