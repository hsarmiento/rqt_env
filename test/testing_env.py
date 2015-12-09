#!/usr/bin/env python
import xml.etree.ElementTree as ET 
import unittest
from rqt_env.xml_info import XmlInfo
import os
import sys
from xml.dom import minidom
import rospkg
from rqt_env.env_os import EnvOs
import difflib


# Here's our "unit tests".
class TestingEnv(unittest.TestCase):
	# compare two sorted list for general variables(if order is different python return false for same elements in the lists) 
	def test_get_general_variables(self):
		xml_info = XmlInfo()
		xml_info.openXml()
		rp =rospkg.RosPack()
		cpath = os.path.join(rp.get_path('rqt_env'),'test','env_test.xml') 
		xml_info._root = ET.parse(cpath).getroot()
		list_ok = [['UNKNOWN1','ABC'],['ORBIT_SOCKETDIR','/tmp/orbit-robotica'],['PYTHONPATH','/home/robotica/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/indigo/lib/python2.7/dist-packages3'],
		['FOOBAR1','1']]
		list_test = xml_info.getGeneralVariables()
		list_ok.sort()
		list_test.sort()
		# print list_test
		# print list_ok
		self.assertEqual(list_ok,list_test) 


	# compare two sorted lists for robots(if order is different, python return false for same elements in the lists) 
	def test_get_robot_list(self):
		list_ok = [["robot1","http://172.43.69.421:78312","localhost","1"],["bot2","http://192.78.69.123:12432","localhost","0"]]
		xml_info = XmlInfo()
		xml_info.openXml()
		rp =rospkg.RosPack()
		cpath = os.path.join(rp.get_path('rqt_env'),'test','env_test.xml') 
		xml_info._root = ET.parse(cpath).getroot()
		list_test = xml_info.getRobots()
		list_test.sort()
		list_ok.sort()
		self.assertEqual(list_ok,list_test) 
		

	def assertFiles(self,first,second):
		file1 = open(first)
		file2 = open(second)
		i = 0
		j = 0
		for line1 in file1:
			for line2 in file2:
				print line1
				print line2
				if line1 != line2:
					
					file1.close()
					file2.close()
					return False 
				j+=1
				break
			i+=1
		file1.close()
		file2.close()
		return True

	def test_export_to_htbash(self):
		general_list = [('FOOBAR1','ABC'),('TEST2','AAB')]
		robot_variables = [('TEST_CIRCLE','123'),('TEST_MAP','BAR'),('TEST_MASTER_URI','http://100.100.100.100')]
		env_os = EnvOs()
		env_os.unset_to_htbash()  #default parameter empty list
		env_os.export_to_general_htbash(general_list)
		env_os.export_to_robot_htbash(robot_variables,'testrobot')
		rp =rospkg.RosPack()
		cpath1 = os.path.join(rp.get_path('rqt_env'),'test','.htbashtest') 
		cpath2 = os.path.join(rp.get_path('rqt_env'),'test','.htbashcompare')
		self.assertTrue(self.assertFiles(cpath1,cpath2))


def main():
    unittest.main()

if __name__ == '__main__':
    main()