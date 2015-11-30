#!/usr/bin/env python
import os
import sys
from .xml_info import XmlInfo


class EnvOs(object):
	def __init__(self):
		super(EnvOs, self).__init__()
		self._root = None

	def openXml(self):
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))+'/../resource/env.xml' 
		self._root = ET.parse(cpath).getroot()

	def unset_to_htbash(self,list_variables):
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))
		cpath = '/'.join(cpath.split('/')[:3])+'/'
		f=open(cpath+".htbash","w")
		for item in set(list_variables):
			f.write("unset "+item+'\n')
		f.close()
		
	def export_to_htbash(self):
		pass
