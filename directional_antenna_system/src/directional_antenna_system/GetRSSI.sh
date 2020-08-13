#! /bin/bash 
# This is a test shell script for fire project.
# This file gives all the information of WIFI access point to file1 and file2.

#sudo iwlist wlp2s0 scan |egrep -i ESSID\|Frequency\|Quality > file1.txt
echo optimize channels
sudo iwlist wlp2s0 scan |egrep -i Frequency\|Quality > /home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/file1.txt
#sleep 3
#echo optimize channels Done
#sleep 2
#sudo iwlist wlp2s0 scan | grep Frequency | sort | uniq -c | sort -n > file2.txt
#sleep 1
#sudo iw wlp2s0 scan | egrep 'SSID|signal' | egrep -B1 'DNC lab|UTA Bonjour'> file3.txt






