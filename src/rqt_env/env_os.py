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

	def parse_to_htbash(self):
		xml_info = XmlInfo()
		general_deleted = xml_info.get_deleted_general_variable()
		print general_deleted



