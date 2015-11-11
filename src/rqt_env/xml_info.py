import xml.etree.ElementTree as ET
import os
import sys

class XmlInfo(object):
	
	def __init__(self):
		self._root = None

	def openXml(self):
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml' 
		self._root = ET.parse(cpath).getroot()

	def getRobots(self):
		l = []
		for elem in self._root.iter(tag='robot'):
			for node in elem.iterfind('variable'):
				l.append([node.attrib['name'],node.attrib['value']])
		return l

	def getGeneralVariables(self):
		l = []
		for elem in self._root.iter(tag='general'):
			for node in elem.iterfind('variable'):
				l.append([node.attrib['name'],node.attrib['value']])
		return l



