#!/usr/bin/env python
import os
import sys
from .xml_info import XmlInfo
import rospkg

class EnvOs(object):
	rp = rospkg.RosPack()
	# cpathtest = os.path.join(rp.get_path('rqt_env'),'test','.htbashtest') #uncomment for testing
	def __init__(self):
		super(EnvOs, self).__init__()
		self._root = None

	def unset_to_htbash(self,list_variables = []):
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))  #comment for testing
		cpath = '/'.join(cpath.split('/')[:3])+'/'  #comment for testing
		# f = open(self.cpathtest,"w")  #uncomment for testing
		f=open(cpath+".htbash","w") #comment for testing
		for item in set(list_variables):
			f.write("unset "+item+'\n')
		f.close()
		
	def export_to_general_htbash(self,list_variables):
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))
		cpath = '/'.join(cpath.split('/')[:3])+'/'
		# f = open(self.cpathtest,"a")  #uncomment for testing
		f = open(cpath+".htbash","a") #comment for testing
		f.write('\n'+"#---variables ROS General------"+'\n')
		for item in list_variables:
			f.write("export "+item[0]+'='+item[1]+'\n')
		f.close()

	def export_to_robot_htbash(self,list_variables,alias):
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))
		cpath = '/'.join(cpath.split('/')[:3])+'/'
		# f = open(self.cpathtest,"a")  #uncomment for testing
		f = open(cpath+".htbash","a") #comment for testing
		f.write('\n'+"#---variables Robot for "+alias+ " ----------"+'\n')
		for item in list_variables:
			f.write("export "+item[0]+'='+item[1]+'\n')
		f.close()

	def is_include(self):
		cpath = os.path.dirname(os.path.abspath(sys.argv[0]))
		cpath = '/'.join(cpath.split('/')[:3])+'/'
		f = open(cpath+".bashrc")
		for line in f:
			if line.strip() == 'source ~/.htbash':
				f.close()
				return True  
		return False

	def include_htbash(self):
		if not self.is_include():
			cpath = os.path.dirname(os.path.abspath(sys.argv[0]))
			cpath = '/'.join(cpath.split('/')[:3])+'/'
			f = open(cpath+".bashrc",'a')
			f.write('source ~/.htbash')
			f.close()



