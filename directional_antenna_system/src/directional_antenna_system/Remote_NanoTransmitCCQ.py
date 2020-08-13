#! /usr/bin/python3.5
import re
import fileinput
import numpy
import time
from selenium import webdriver
from selenium.webdriver.support.ui import Select
from selenium.webdriver.firefox.options import Options

def RemoteGetTransmitCCQ():

	'''
	Open local Huawei website to change channel automatically
	'''

	usr = "ubnt"
	pwd = "asdfghjkl"


	option = Options()
	option.add_argument("--headless")
	browser1 = webdriver.Firefox(firefox_options=option)
	browser1.get("http://192.168.33.110")  # 100:local 110: remote
	elem1 = browser1.find_element_by_id("username")
	elem2 = browser1.find_element_by_id("password")
	elem1.clear()
	elem2.clear()
	elem1.send_keys(usr)
	elem2.send_keys(pwd)

	login1 = browser1.find_element_by_css_selector("input[value='Login'][type='submit']")
	login1.click()

	browser1.get("https://192.168.33.110/index.cgi")

	TransmitCCQ = browser1.find_element_by_id("ccq")


	text = (TransmitCCQ.text)

	#print(text)

	return text



'''
option = Options()
option.add_argument("--headless")
browser1 = webdriver.Firefox(firefox_options=option)
browser1.get("http://www.google.com")  # 100:local 110: remote
'''


