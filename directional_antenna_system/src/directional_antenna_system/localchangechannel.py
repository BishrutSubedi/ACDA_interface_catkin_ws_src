#! /usr/bin/python3.5
import re
import fileinput
import numpy
import time
from selenium import webdriver
from selenium.webdriver.support.ui import Select

'''
File operations to find the best channel 
'''

def local_change_channel(*args):
	Lookup_table = ['UAV channel 13, 14, 15 is not recommended.','UAV channel 13, 14, 15 is not recommended.','UAV channel 13, 14, 15, 16 is not recommended.','UAV channel 14, 15, 16 is not recommended.','UAV channel 14, 15, 16, 17 is not recommended.','UAV channel 15, 16, 17 is not recommended.','UAV channel 15, 16, 17, 18 is not recommended.','UAV channel 16, 17, 18 is not recommended.','UAV channel 16, 17, 18, 19 is not recommended.','UAV channel 17, 18, 19 is not recommended.','UAV channel 17, 18, 19, 20 is not recommended.','UAV channel 18, 19, 20 is not recommended.','UAV channel 18, 19, 20 is not recommended.']


	file1 = open('/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/file1.txt','r')
	lines1 = file1.readlines()
	file1.close()

	mod_list = []
	channelNo = 11
	channelArray = numpy.zeros(channelNo)
	channelRSSI = numpy.ones(channelNo)*999;
	count = 1


	file1 = open('/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/file_update.txt','w')
	for line in lines1:
		line = line.strip()
		if count%2==0:
			mod_list.append(old_line+line)
		else: 
			old_line = line

		count+=1



	for item in mod_list:
		linenumbers = re.findall(r"[-+]?\d*\.\d+|\d+",item)
		if (float(linenumbers[0])<3) and (float(linenumbers[0])>2) and (float(linenumbers[1])<=channelNo):
			file1.writelines("%s\n" % item)
	file1.close()

		
	file1 = open('/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/file_update.txt','r')
	chan_file = file1.read()
	numbers = re.findall(r"[-+]?\d*\.\d+|\d+",chan_file)
	numbers = [float(i) for i in numbers]


	for i in range(1,channelNo+1):
		channelArray[i-1] = numbers.count(i)

	for i in range(1,int(len(numbers)/5)+1):
		temp = numbers[(i-1)*5+1]
		rssiValue = numbers[(i-1)*5+4]
		if (rssiValue < channelRSSI[int(temp)-1]):
			channelRSSI[int(temp)-1] = rssiValue	


	print(channelArray)
	print(channelRSSI)

	temp1 = 1/(channelArray+1)
	temp1 = temp1/numpy.sum(temp1)
	temp2 = channelRSSI/numpy.sum(channelRSSI)
	prob_channel = temp2+temp1
	prob_channel = prob_channel/numpy.sum(prob_channel)
	prob1 = list(prob_channel)

	
	#channel1 = numpy.random.choice(3,1,[0.1,0.6,0.3])
	
	channel1 = channel1 = numpy.random.choice(channelNo,1,prob1)
	channel1 = channel1+1
	print(channel1[0])
	print(Lookup_table[channel1[0]])
	print(type(channel1[0]))
	
	file_channelno = open('/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/file_channelno_local.txt','w')
	file_channelno.write(str(channel1[0]))
	
	
	'''
	Open local Huawei website to change channel automatically

	'''
	
	
	
	usr = "admin"
	pwd = "admin"

	browser1 = webdriver.Firefox()
	browser1.get("http://192.168.33.111")
	elem1 = browser1.find_element_by_id("txt_Username")
	elem2 = browser1.find_element_by_id("txt_Password")
	elem1.clear()
	elem2.clear()
	elem1.send_keys(usr)
	elem2.send_keys(pwd)
	login1 = browser1.find_element_by_xpath("/html/body/table[4]/tbody/tr[3]/td/table/tbody/tr[2]/td[2]")
	login1.click()

	browser1.get("http://192.168.33.111/html/content1.asp")
	browser1.switch_to.frame("menufrm")
	elem = browser1.find_element_by_id("link_User_1")
	elem.click()
	elem = browser1.find_element_by_id("link_User_1_4")
	elem.click()

	browser1.switch_to.parent_frame()
	browser1.switch_to.frame("contentfrm1")
	elem = browser1.find_element_by_name("wlChannel")
	select = Select(elem)

	select.select_by_value(str(channel1[0]))

	browser1.switch_to.parent_frame()
	browser1.close()

	time.sleep(5)

	return str(channel1[0])
	
	
	
#channel_value = local_change_channel()
#print(channel_value)












