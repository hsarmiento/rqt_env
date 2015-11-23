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
				for node in elem.iterfind('variable'):
					if node.attrib['name'] == 'ROS_MASTER_URI':
						l.append([elem.attrib['id'],node.attrib['value'],elem.attrib['status']])
		return l
		
	def removeGeneralVariable(self):
		 for elem in self._root.iter(tag='general'):
		 	rank = int(elem.find('variable').text)
		 	if rank > 50:
		 		root.remove(country)
		 		tree.write('output.xml')

	def add_variable_ros(self,variable,value):
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
  