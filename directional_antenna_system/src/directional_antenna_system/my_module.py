#! /usr/bin/python3.5
import os
import sys
import time
import threading
import math
import multiprocessing
import rospkg
import rospy
from PyQt5.QtGui import QFont, QPixmap
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String
import subprocess
sys.path.insert(0, "../")
import qOSM
qOSM.use("PyQt5")
from qOSM.common import QOSM
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Signal
from compass import CompassWidget
from rcompass import RCompassWidget
import localchangechannel
import remotechangechannel
import Remote_NanoTransmitCCQ
import socket

# import gc
# import objgraph
# import random


if qOSM.get_backed() == "PyQt5":
    from PyQt5.QtCore import *
    from PyQt5.QtWidgets import *

class MyPlugin(Plugin):
    sig_sysmsg = Signal(str)
    l_location = pyqtSignal(float, float)
    r_location = pyqtSignal(float, float)

    LOS_optimize_chal=pyqtSignal(str,str)
    NLOS_optimize_chal=pyqtSignal(str)



    com_timer = QTimer()
    rssi_timer = QTimer()
    baro_timer = QTimer()
    ConnInd_timer = QTimer()

    _pid = 0000000
    # save_file_processing





    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')
        rp = rospkg.RosPack()
        self._lrssivalue=0.0
        self._rrssivalue=0.0
        self._lcomvalue=0.0
        self._rcomvalue=0.0
        self._lbaroaltvalue = 0.0
        self._rbaroaltvalue = 0.0
        self._enable_gps=0
        self._LOS_NLOS=1
        self.imu_done=4
        self.imu_run_stop_value=1
        self.initial_heading=500
        self.ros_run_stop_value=1
        self.run_initial_scan=1
        self.initial_scan_state=1
        self.localimu=0
        self.remoteimu=0
        self.camera_run_save_stop_state=1
        self.initial_map=1
        self.rssi_test_gps_rssi=1



        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

##        def goCoords():
##            def resetError():
##               # coordsEdit.setStyleSheet('')
##
##            try:
##		print("work")
##                #latitude, longitude = coordsEdit.text().split(",")
##		
##            except ValueError:
##                #coordsEdit.setStyleSheet("color: red;")
##                QTimer.singleShot(500, resetError)
##            else:
##                map.centerAt(latitude, longitude)
##                # map.moveMarker("MyDragableMark", latitude, longitude)

        def onMarkerMoved(key, latitude, longitude):
            print("Moved!!", key, latitude, longitude)
            if key == 'local GPS':
                l_coordsEdit.setText("{0:.8}, {1:.8}".format(latitude, longitude))

            else:
                r_coordsEdit.setText("{0:.8}, {1:.8}".format(latitude, longitude))
            #coordsEdit.setText("{}, {}".format(latitude, longitude))
	        #self.map.moveMarker("local GPS",latitude+0.01, longitude+0.01)
        def onMarkerRClick(key):
            print("RClick on ", key)
            # map.setMarkerOptions(key, draggable=False)

        def onMarkerLClick(key):
            print("LClick on ", key)

        def onMarkerDClick(key):
            print("DClick on ", key)
            # map.setMarkerOptions(key, draggable=True)

        def onMapMoved(latitude, longitude):
            print("Moved to ", latitude, longitude)

        def onMapRClick(latitude, longitude):
            print("RClick on ", latitude, longitude)

        def onMapLClick(latitude, longitude):
            print("LClick on ", latitude, longitude)

        def onMapDClick(latitude, longitude):
            print("DClick on ", latitude, longitude)

        def move_l_mark(lat_l,lon_l):

            #onMarkerMoved("local gps",lat_,lon_)
            #print("get location",lat_,lon_)
            self.map.moveMarker("local GPS",lat_l,lon_l)
            l_coordsEdit.setText("{0:.9}, {1:.9}".format(lat_l,lon_l))
            if self.initial_map==1:
                self.map.centerAt(lat_l,lon_l)
                self.map.setZoom(15)
                self.initial_map=0


	    #time.sleep(0.01)
        def local_callback(llocation):
            llat=llocation.data[0]
            llon=llocation.data[1]
            #move_mark(self,lat,lon)
            self.l_location.emit(llat,llon)
            
               
        def move_r_mark(lat_r,lon_r):

            #onMarkerMoved("local gps",lat_,lon_)
            #print("get location",lat_,lon_)
            self.map.moveMarker("remote GPS",lat_r,lon_r)
            r_coordsEdit.setText("{0:.9}, {1:.9}".format(lat_r,lon_r))


	    #time.sleep(0.01)

           
        def remote_callback(rlocation):
            rlat=rlocation.data[0]
            rlon=rlocation.data[1]
            #move_mark(self,lat,lon)
            self.r_location.emit(rlat,rlon)

        @pyqtSlot()
        def change_los_nos_value():
            if self._LOS_NLOS==1:
                self._LOS_NLOS=0
                initial_heading_calculation_button.setText("Run")
            else:
                self._LOS_NLOS=1
                initial_heading_calculation_button.setText("Scan")

            print(self._LOS_NLOS)



        def optimize_channel_fun():
            os.system(
                "sudo /home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/GetRSSI.sh")
            if self._LOS_NLOS == 1:
                a = localchangechannel.local_change_channel()
                print("channel a", a)
                time.sleep(2)
                b = remotechangechannel.remote_change_channel()
                print("channel b", b)
                self.LOS_optimize_chal.emit(a,b)
                # channel_textedit.setText("L:%s, R:%s" % (a, b))
            else:
                a = localchangechannel.local_change_channel()
                self.NLOS_optimize_chal.emit(a)
                # channel_textedit.setText("L:%s, R:N" % a)

        @pyqtSlot()
        def LOS_channel_testedit(vala,valb):
            channel_textedit.setText("L:%s, R:%s" % (vala, valb))


        @pyqtSlot()
        def NLOS_channel_testedit(vala):
            channel_textedit.setText("L:%s, R:N" % vala)






        @pyqtSlot()
        def channel_on_click():


            optimize_channel_process= threading.Thread(target=optimize_channel_fun)
            optimize_channel_process.start()


            # optimize_channel_process.start()





        # imu calibration *******************************************

        def run_calibration_fun():
            print("calibration")
            if (self._LOS_NLOS == 1):
                # print("calibrate two IMUs")
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/stop_IMU_calibration.sh 1")
                time.sleep(7)
                os.system( "/home/dnc2/Documents/catkin_ws/devel/sshnode/start_IMU_calibration.sh 1")
            else:
                # print("calibrate local IMU")
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/stop_IMU_calibration.sh 0")
                time.sleep(7)
                os.system( "/home/dnc2/Documents/catkin_ws/devel/sshnode/start_IMU_calibration.sh 0")


        @pyqtSlot()
        def imu_run_stop_click():

            # imu_threading = threading.Thread(target=run_calibration_fun)
            imu_threading= multiprocessing.Process(target=run_calibration_fun)
            if self.imu_run_stop_value==1:

                print("stat threading")
                IMU_run_stop_button.setText("Stop")
                IMU_state_state_label.setText("Working")
                time.sleep(1)
                self.imu_state_timer.start(60000)
                imu_threading.start()

                self.imu_run_stop_value = 0
            else:
                IMU_run_stop_button.setText("Run")
                self.imu_run_stop_value = 1
                IMU_state_state_label.setText("State")
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/stop_IMU_calibration.sh 1")
                # if imu_threading.isAlive():


            # self.imu_threading = threading.Thread(target=run_calibration_fun(selqf, self._LOS_NLOS))
            # self.imu_threading.start()

            # parent_conn, child_conn = multiprocessing.Pipe()
            # self.imu_processing = multiprocessing.Process(target=run_calibration_fun)
            # self.imu_processing.start()


        def stat_ros_nodes():
            print("stat ROS node ...")
            # os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/save_rosbag.sh")
            # if self._LOS_NLOS==1:



            if self.initial_heading==500:
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/stop_ROS_nodes.sh %d" % (self._LOS_NLOS))
                time.sleep(7)
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/start_ros_no_initial.sh %d " % (self._LOS_NLOS))

            else:
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/stop_ROS_nodes.sh %d" % (self._LOS_NLOS))
                time.sleep(7)
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/start_ros_with_initial.sh %d %d" %(self._LOS_NLOS,self.initial_heading))

            # else:
            #     os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/start_local.sh %d %f" % self._LOS_NLOS,self.initial_heading)





        @pyqtSlot()
        def stat_ros_click():
            # self.imu_timer.stop();

            start_ros_node_threading = threading.Thread(target=stat_ros_nodes)

            # if not self.monitor_ccq.isRunning():
            #     self.monitor_ccq.start()


            # start_save_threading = threading.Thread(target=start_save_data)

            if self.ros_run_stop_value==1:
                self.ros_run_stop_value=0

                Ros_node_run_button.setText("Stop")
                Ros_node_state_label.setText("Working")
                IMU_run_stop_button.setEnabled(False)
                self.distance_timer.start(10000)
                IMU_run_stop_button.setStyleSheet(
                    "background-color: rgb(192,197,197);padding: 6px; border-radius:5px;text_align:left")
                start_ros_node_threading.start()

            else:
                self.ros_run_stop_value=1
                # self.monitor_ccq.terminate()
                Ros_node_run_button.setText("Run")
                Ros_node_state_label.setText("Done")
                IMU_run_stop_button.setEnabled(True)
                self.distance_timer.stop()
                IMU_run_stop_button.setStyleSheet(
                    "background-color: rgb(245,128,38);padding: 6px; border-radius:5px;text_align:left")
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/stop_ROS_nodes.sh %d" % (self._LOS_NLOS))
                # if start_ros_node_threading.isAlive():
            # start_save_threading.start()



        def run_camera_nodes():
            if self.camera_run_save_stop_state==1:

                if infrared_camera.isChecked():
                    camera_comman=10
                elif not infrared_camera.isChecked():
                    camera_comman=0

                if optical_camera.isChecked():
                    opt_camera_comman=10
                elif not optical_camera.isChecked():
                    opt_camera_comman=0
                camera_run_button.setText("Confirm")
                self.camera_run_save_stop_state = 2
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/start_camera_nodes.sh %d %d" %(camera_comman,opt_camera_comman))


            elif self.camera_run_save_stop_state==2:
                if save_infrared.isChecked():
                    save_inf_var=1
                else:
                    save_inf_var=0

                if save_optical.isChecked():
                    save_opt_var=1
                else:
                    save_opt_var=0
                camera_run_button.setText("Stop")
                self.camera_run_save_stop_state = 3
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/save_camera_nodes.sh %d %d" % (save_inf_var, save_opt_var))


            elif self.camera_run_save_stop_state==3:
                camera_run_button.setText("Confirm")
                self.camera_run_save_stop_state = 1
                os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/stop_camera_nodes.sh")




            # if optical_camera.isChecked() and infrared_camera.isChecked():
            #     os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/start_camera_nodes.sh 1 1")
            # elif optical_camera.isChecked() and infrared_camera.isChecked()==0:
            #     os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/start_camera_nodes.sh 1 0")
            # elif optical_camera.isChecked()==0 and infrared_camera.isChecked()==1:
            #     os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/start_camera_nodes.sh 0 1")




        @pyqtSlot()
        def start_camera_click():
            start_camera_threading=threading.Thread(target=run_camera_nodes)
            if optical_camera.isChecked() or infrared_camera.isChecked():
                start_camera_threading.start()


        def initial_scan_run():
            os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/start_initial_scan.sh")




        @pyqtSlot()
        def calculate_Azimuth():
            if self._LOS_NLOS==1 and self.run_initial_scan==1:
                initial_heading_calculation_button.setText("Stop")
                self.run_initial_scan = 0
                IMU_run_stop_button.setEnabled(False)
                IMU_run_stop_button.setStyleSheet("background-color: rgb(192,197,197);padding: 6px; border-radius:5px;text_align:left")

                initial_scan_threading =threading.Thread(target=initial_scan_run)
                initial_scan_threading.start()
            elif self._LOS_NLOS ==1 and self.run_initial_scan==0:
                initial_heading_calculation_button.setText("Run")
                self.run_initial_scan = 1
                IMU_run_stop_button.setEnabled(True)
                IMU_run_stop_button.setStyleSheet(
                    "background-color: rgb(245,128,38);padding: 6px; border-radius:5px;text_align:left")
                # os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/stop_initial_scan.sh")

            if self._LOS_NLOS==0:
                if l_coordsEdit.text() and l_coordsEdit.text().find(",") !=-1:
                    llocation_string = l_coordsEdit.text()
                    print(llocation_string)
                    l_lat_lon = llocation_string.split(",")
                    try:
                        l_latitiude = float(l_lat_lon[0])
                        l_longitude = float(l_lat_lon[1])
                    except:
                        Azimuth_warning()
                        return

                if r_coordsEdit.text() and r_coordsEdit.text().find(",") != -1:
                    rlocation_string = r_coordsEdit.text()
                    print(rlocation_string)
                    r_lat_lon = rlocation_string.split(",")
                    try:
                        r_latitiude = float(r_lat_lon[0])
                        r_longitude = float(r_lat_lon[1])
                        temp_remote_gps_msg.data= [r_latitiude, r_longitude]
                        temp_remote_gps.publish(temp_remote_gps_msg)
                        # print("temp_gps_test1")
                        # temp_remote_gps.publish(temp_remote_gps_msg)
                        # print("temp_gps_test")
                    except:
                        Azimuth_warning()
                        return


                try:
                    LON1 = l_longitude*0.0174532925199433
                    LAT1 = l_latitiude*0.0174532925199433
                    LON2 = r_longitude*0.0174532925199433
                    LAT2 = r_latitiude*0.0174532925199433
                    az = math.atan2(math.sin(LON2-LON1) * math.cos(LAT2),math.cos(LAT1) * math.sin(LAT2) - math.sin(LAT1) * math.cos(LAT2) * math.cos(LON2-LON1))
                    az = math.fmod(az, 9.118906528)
                    az = math.fmod(az, 6.283185307179586)*(57.2957795131)
                    if az<0 :
                        az = az+360
                    Azimuth =az
                except:
                    Azimuth_warning()
                    return

                initial_heading_display_label.setText(" %0.1f"%Azimuth)
                self.initial_heading=int(Azimuth)














        @pyqtSlot()
        def calculate_distance():
            if l_coordsEdit.text() and l_coordsEdit.text().find(",") != -1:
                llocation_string = l_coordsEdit.text()
                l_lat_lon = llocation_string.split(",")
                try:
                    l_latitiude = float(l_lat_lon[0])
                    l_longitude = float(l_lat_lon[1])
                except:
                    # Azimuth_warning()
                    # print("error1")
                    return

            if r_coordsEdit.text() and r_coordsEdit.text().find(",") != -1:
                rlocation_string = r_coordsEdit.text()
                r_lat_lon = rlocation_string.split(",")
                try:
                    r_latitiude = float(r_lat_lon[0])
                    r_longitude = float(r_lat_lon[1])
                except:
                    # Azimuth_warning()
                    # print("error2")
                    return

            try:
                LON1 = l_longitude * 0.0174532925199433
                LAT1 = l_latitiude * 0.0174532925199433
                LON2 = r_longitude * 0.0174532925199433
                LAT2 = r_latitiude * 0.0174532925199433
                A =math.sin((LAT2-LAT1)/2)*math.sin((LAT2-LAT1)/2)+math.cos(LAT1)*math.cos(LAT2)*math.sin((LON2-LON1)/2)*math.sin((LON2-LON1)/2)
                # print('A',A)
                AB= 2*math.atan2(math.sqrt(A),math.sqrt(1-A))
                # print('AB',AB)
                AB_DIS=6371000*AB
                # print('AB_DIS',AB_DIS)
                # distance_text.setText("%0.1f m"%AB_DIS)
                self.distance_label.setText("%0.1f m"%AB_DIS)
                # distance_lable.setText("0.1f m"%AB_DIS)
            except:
                # print("error3")
                # Azimuth_warning()
                return




        def Azimuth_warning():
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Azimuth calculation")
            msg.setText("The locations of local and remote UAVs cannot be empty.\nPlease input locations(latitude,longitude) correctly!")
            msg.exec_()


        # @pyqtSlot()
        # def pub_new_Azimuth():
        #     try:
        #         N_Azimuth = float(Azimuth_data.text())
        #     except:
        #         Azimuth_warning()
        #         return
        #     if math.fabs(N_Azimuth)>= 360:
        #         Azimuth_warning()
        #         return
        #     pub_Azimuth.publish(N_Azimuth)

        @pyqtSlot()
        def change_baroalt():
            # self._rrssivalue=str(rssi2)

            r_baro_text.setText("%.1f" % self._rbaroaltvalue)
            l_baro_text.setText("%.1f" % self._lbaroaltvalue)
            time.sleep(0.01)
            # time.sleep(0.01)

        def l_baro_callback(lbaroinfo):
            self._lbaroaltvalue = lbaroinfo.data[2]


        def r_baro_callback(rbaroinfo):
            self._rbaroaltvalue = rbaroinfo.data[2]






        @pyqtSlot()
        def change_rssi():
            #self._rrssivalue=str(rssi2)

            r_rssi_text.setText(str(self._rrssivalue))
            l_rssi_text.setText(str(self._lrssivalue))
            time.sleep(0.01)
            #time.sleep(0.01)



        def l_rssi_callback(rssil):
            self._lrssivalue=rssil.data
            # self.l_rssi.emit(rssil.data)
            # del rssil
            #l_rssi_text.update()
            #l_rssi_text.setText(str(rssi.data))


        def r_rssi_callback(rssir):
            self._rrssivalue=rssir.data
            # self.r_rssi.emit(rssir.data)

            # self.memory_tracker.print_diff()
            # print(sys.getsizeof(move_l_compass(headidng1)))
            #r_rssi_text.setText(str(rssi.data))

        @pyqtSlot()
        def move_compass():
            l_com_text.setText("%.1f" % self._lcomvalue)
            time.sleep(0.01)
            # print('work')
            l_com_widget.setAngle(self._lcomvalue)
            time.sleep(0.01)
            r_com_text.setText("%.1f" % self._rcomvalue)
            time.sleep(0.01)
            r_com_widget.setAngle(self._rcomvalue)
            time.sleep(0.01)

        # def map_rssi_fun_click():
        #     if self.rssi_test_gps_rssi==1:
        #         for i in range(5):
        #             pub_gps_Azimuth_enable.publish(0)
        #             time.sleep(0.2)
        #             self.rssi_test_gps_rssi=0
        #     else:
        #         for i in range(5):
        #             pub_gps_Azimuth_enable.publish(1)
        #             time.sleep(0.2)
        #             self.rssi_test_gps_rssi = 1






        def l_com_callback(heading):
            self._lcomvalue=heading.data
            # print('work1')
            #l_com_text.setText("%.1f" % heading.data)
            # self.l_com.emit(heading.data)
            # heading.data =None
            #self.update()
            #l_com_widget.setAngle(heading.data)


        def r_com_callback(heading):

            self._rcomvalue=heading.data
            #r_com_text.setText("%.1f" % heading.data)
            # self.r_com.emit(heading.data)
            # del heading
            #r_com_widget.setAngle(heading.data)

        def local_imucal_callback(l_imu_msg):
            self.localimu=l_imu_msg.data
            # if self.localimu == 1 and self.remoteimu == 1:
            #     imu_lock.acquire()
            #     try:
            #         IMU_state_state_label.setText("Done")
            #     finally:
            #         imu_lock.release()


        def remote_imucal_callback(r_imu_msg):
            self.remoteimu=r_imu_msg.data
            # if self.localimu == 1 and self.remoteimu == 1:
            #     imu_lock.acquire()
            #     try:
            #         IMU_state_state_label.setText("Done")
            #     finally:
            #         imu_lock.release()



        @pyqtSlot()
        def imu_state_display():
            if self._LOS_NLOS==1:
                if self.remoteimu==1 and self.localimu==1:
                    IMU_state_state_label.setText("Done")
                    self.imu_state_timer.stop()
            else:
                if self.localimu == 1:
                    IMU_state_state_label.setText("Done")
                    self.imu_state_timer.stop()




        @pyqtSlot()
        def local_bbb_conn(var):
            if var ==1:
                lbbb_conn_icon.setPixmap(green_icon)
            else:
                lbbb_conn_icon.setPixmap(red_icon)

        @pyqtSlot()
        def remote_bbb_conn(var):
            if var==1:
                rbbb_conn_icon.setPixmap(green_icon)

            else:
                rbbb_conn_icon.setPixmap(red_icon)



        @pyqtSlot()
        def local_connection_indicator(): # var: RSSI
            l_var = self._lrssivalue
            r_var = self._rrssivalue
            #r_var = -55
            #l_var = -75

            if l_var>=-60 and l_var<0:
                l_con_ind_icon.setPixmap(ConnectionBar5)
            elif l_var<-60 and l_var>=-70:
                l_con_ind_icon.setPixmap(ConnectionBar4)
            elif l_var <-70 and l_var>= -80:
                l_con_ind_icon.setPixmap(ConnectionBar3)
            elif l_var <-80 and l_var > -96:
                l_con_ind_icon.setPixmap(ConnectionBar2)
            else:
                l_con_ind_icon.setPixmap(ConnectionBar1)


            if r_var >= -60 and l_var<0:
                r_con_ind_icon.setPixmap(ConnectionBar5)
            elif r_var < -60 and r_var >= -70:
                r_con_ind_icon.setPixmap(ConnectionBar4)
            elif r_var < -70 and r_var >= -80:
                r_con_ind_icon.setPixmap(ConnectionBar3)
            elif r_var < -80 and r_var > -96:
                r_con_ind_icon.setPixmap(ConnectionBar2)
            else:
                r_con_ind_icon.setPixmap(ConnectionBar1)



        # def local_connection_indicator(): # var: RSSI
        #     #var = self._rrssivalue
        #     var = -55
        #     if var>=-60:
        #         l_con_ind_icon.setPixmap(ConnectionBar5)
        #     elif var<-60 and var>=-70:
        #         l_con_ind_icon.setPixmap(ConnectionBar4)
        #     elif var <-70 and var>= -80:
        #         l_con_ind_icon.setPixmap(ConnectionBar3)
        #     elif var <-80 and var > -96:
        #         l_con_ind_icon.setPixmap(ConnectionBar2)
        #     else:
        #         l_con_ind_icon.setPixmap(ConnectionBar1)


        self._widget = QWidget()
        gcv=QVBoxLayout(self._widget)




        v = QVBoxLayout()
        h = QHBoxLayout()

        FUN_FONT= QFont()
        FUN_FONT.setBold(True)
        FUN_FONT.setPointSize(14)
        FUN_FONT.setWeight(87)
        pub_gps_Azimuth_enable = rospy.Publisher('lenablegpsazimuth', Int32, queue_size=10)

        temp_remote_gps = rospy.Publisher('temp_gps_location', Float64MultiArray,queue_size=1)
        temp_remote_gps_msg= Float64MultiArray()


        #ros_threading
        #############################
        # star_ros_threading = threading.Thread(target= stat_ros_nodes)
        # # save_file_processing = multiprocessing.Process(target= save_topic_fun)
        # save_file_processing = threading.Thread(target= save_topic_fun)
        # star_ros_manual_com_threading = threading.Thread(target= stat_ros_manual_com_nodes)
        # imu_threading = threading.Thread(target=run_calibration_fun)

        green_icon = QPixmap(("/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/greenenergy.png"))
        green_icon = green_icon.scaled(40, 40, Qt.KeepAspectRatio)

        red_icon = QPixmap(("/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/redenergy.png"))
        red_icon = red_icon.scaled(40, 40, Qt.KeepAspectRatio)


        ConnectionBar1 = QPixmap(("/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/ConnectionIndex1.png"))
        ConnectionBar2 = QPixmap(("/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/ConnectionIndex2.png"))
        ConnectionBar3 = QPixmap(("/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/ConnectionIndex3.png"))
        ConnectionBar4 = QPixmap(("/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/ConnectionIndex4.png"))
        ConnectionBar5 = QPixmap(("/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/ConnectionIndex5.png"))
        ConnectionBar1 = ConnectionBar1.scaled(50, 110, Qt.KeepAspectRatio)
        ConnectionBar2 = ConnectionBar2.scaled(50, 110, Qt.KeepAspectRatio)
        ConnectionBar3 = ConnectionBar3.scaled(50, 110, Qt.KeepAspectRatio)
        ConnectionBar4 = ConnectionBar4.scaled(50, 110, Qt.KeepAspectRatio)
        ConnectionBar5 = ConnectionBar5.scaled(50, 110, Qt.KeepAspectRatio)

        ##################################
        # local widget
        local_widget = QWidget()
        local_Vlayout = QVBoxLayout(local_widget)
        local_widget.setStyleSheet("background-color: white")

        l_coordsEdit = QLineEdit()
        l_coordsEdit.setFont(FUN_FONT)
        l_coordsEdit.setMinimumWidth(230)
        lgps_label = QLabel('GPS:')
        lgps_label.setFont(FUN_FONT)
        l_com_widget = CompassWidget()
        l_rssi_lable = QLabel('RSSI:')
        l_rssi_lable.setFont(FUN_FONT)
        l_rssi_text = QLineEdit()
        l_rssi_text.setFont(FUN_FONT)
        l_rssi_text.setMaximumWidth(70)

        l_baro_lable = QLabel('Alt:')
        l_baro_lable.setFont(FUN_FONT)
        l_baro_text = QLineEdit()
        l_baro_text.setFont(FUN_FONT)
        l_baro_text.setMaximumWidth(80)

        l_com_label = QLabel('Antenna Heading:')
        l_com_label.setFont(FUN_FONT)
        l_com_text = QLineEdit()
        l_com_text.setFont(FUN_FONT)
        # l_com_text.setText(self._lcomvalue)
        l_com_text.setMaximumWidth(80)
        l_h1 = QHBoxLayout()
        l_h1.addStretch()
        l_h1.addWidget(l_rssi_lable)
        l_h1.addWidget(l_rssi_text)
        l_h1.addWidget(l_com_label)
        l_h1.addWidget(l_com_text)
        l_h1.addStretch()
        l_h2 = QHBoxLayout()
        l_h2.addStretch()
        l_h2.addWidget(lgps_label)
        l_h2.addWidget(l_coordsEdit)
        l_h2.addWidget(l_baro_lable)
        l_h2.addWidget(l_baro_text)
        l_h2.addStretch()
        # l_h1.setSpacing(0)
        local_Vlayout.addWidget(l_com_widget)
        local_Vlayout.addLayout(l_h1)
        local_Vlayout.addLayout(l_h2)
        local_widget.setMinimumSize(450,300)
        local_widget.setMaximumWidth(500)
        lbbb_conn_icon=QLabel(l_com_widget)
        lbbb_conn_icon.setText("ICON")
        lbbb_conn_icon.setGeometry(380,200,40,40)

        lbbb_conn_icon.setPixmap(red_icon)


        l_con_ind_icon = QLabel(l_com_widget)
        l_con_ind_icon.setText("ICON")
        l_con_ind_icon.setGeometry(380,40,50,110)

        l_con_ind_icon.setPixmap(ConnectionBar1)




        ##################################
        # remote widget
        remote_widget = QWidget()
        remote_vlayout = QVBoxLayout(remote_widget)
        remote_widget.setStyleSheet("background-color: white")
        r_coordsEdit = QLineEdit()
        r_coordsEdit.setFont(FUN_FONT)
        r_coordsEdit.setMinimumWidth(230)
        rgps_label = QLabel('GPS:')
        rgps_label.setFont(FUN_FONT)
        r_com_widget = RCompassWidget()
        r_rssi_lable=QLabel('RSSI')
        r_rssi_lable.setFont(FUN_FONT)
        r_rssi_text=QLineEdit()
        r_rssi_text.setFont(FUN_FONT)
        r_rssi_text.setMaximumWidth(70)

        r_baro_lable = QLabel('Alt:')
        r_baro_lable.setFont(FUN_FONT)
        r_baro_text = QLineEdit()
        r_baro_text.setFont(FUN_FONT)
        r_baro_text.setMaximumWidth(80)

        r_com_label=QLabel('Antenna Heading:')
        r_com_label.setFont(FUN_FONT)
        r_com_text=QLineEdit()
        r_com_text.setFont(FUN_FONT)
        r_com_text.setMaximumWidth(80)

        r_h1 = QHBoxLayout()
        r_h1.addStretch()
        r_h1.addWidget(r_rssi_lable)
        r_h1.addWidget(r_rssi_text)
        r_h1.addWidget(r_com_label)
        r_h1.addWidget(r_com_text)
        r_h1.addStretch()

        r_h2 =QHBoxLayout()
        r_h2.addStretch()
        r_h2.addWidget(rgps_label)
        r_h2.addWidget(r_coordsEdit)
        r_h2.addWidget(r_baro_lable)
        r_h2.addWidget(r_baro_text)
        r_h2.addStretch()

        remote_vlayout.addWidget(r_com_widget)
        remote_vlayout.addLayout(r_h1)
        remote_vlayout.addLayout(r_h2)
        remote_widget.setMinimumSize(450,300)
        # remote_widget.setMinimumSize(local_widget.width())
        rbbb_conn_icon = QLabel(r_com_widget)
        rbbb_conn_icon.setText("ICON")
        rbbb_conn_icon.setGeometry(380, 200, 40, 40)

        rbbb_conn_icon.setPixmap(red_icon)

        r_con_ind_icon = QLabel(r_com_widget)
        r_con_ind_icon.setText("ICON")
        r_con_ind_icon.setGeometry(380, 40, 50, 110)

        r_con_ind_icon.setPixmap(ConnectionBar1)


        ##################################
        #distance and CCQ
        distance_lable=QLabel("Distance")
        distance_text=QLabel("")
        CCQ_lable=QLabel("Transmit CCQ: ")
        CCQ_text=QLabel()

        DIS_CCQ_widget= QWidget()
        DIS_CCQ_layout=QHBoxLayout(DIS_CCQ_widget)
        DIS_CCQ_layout.addStretch(1)
        DIS_CCQ_layout.addWidget(distance_lable)
        DIS_CCQ_layout.addWidget(distance_text)
        DIS_CCQ_layout.addStretch(2)
        DIS_CCQ_layout.addWidget(CCQ_lable)
        DIS_CCQ_layout.addWidget(CCQ_text)
        DIS_CCQ_layout.addStretch(1)
        DIS_CCQ_widget.setStyleSheet("background-color: white")




        # functional buttons

        func_widget = QWidget()
        Func_widget_layout= QGridLayout(func_widget)
        # Func_widget_layout.setRowStretch(0,1)
        Func_widget_layout.setHorizontalSpacing(5)
        # Func_widget_layout.setSpacing(1)
        distan_option_label= QLabel("Distance Option: ")

        LOS = QRadioButton("LOS")
        LOS.setChecked(True)
        NLOS = QRadioButton("NLOS")
        NLOS.setChecked(False)



        channel_selection_label= QLabel("Channel Selection: ")
        channel_textedit= QLineEdit("CH")
        channel_optimize= QPushButton("Optimize")
        self.LOS_optimize_chal.connect(LOS_channel_testedit)
        self.NLOS_optimize_chal.connect(NLOS_channel_testedit)


        IMU_calibration_label= QLabel("IMU Calibration: ")
        IMU_run_stop_button = QPushButton("Run")
        # IMU_run_stop_button.seta
        IMU_state_state_label= QLineEdit("state")
        initial_heading_label=QLabel("Initial Heading: ")
        initial_heading_display_label= QLineEdit("Default")
        initial_heading_calculation_button = QPushButton("Scan")
        ROS_node_label= QLabel("ROS Nodes: ")
        Ros_node_run_button= QPushButton("Run")
        Ros_node_state_label= QLineEdit("state")
        camera_label =QLabel("Cameras in Use: ")
        camera_group_widget=QWidget()
        camera_group=QGridLayout(camera_group_widget)
        # camera_group.setAlignment(Qt.AlignLeft)
        optical_camera=QCheckBox("Optical")
        optical_camera.setChecked(True)
        infrared_camera=QCheckBox("Infrared")
        infrared_camera.setChecked(True)
        save_infrared=QCheckBox("Save")
        save_optical=QCheckBox("Save")
        camera_group.addWidget(optical_camera,0,0)
        camera_group.addWidget(save_optical,0,1)
        camera_group.addWidget(infrared_camera,1,0)
        camera_group.addWidget(save_infrared,1,1)

        camera_run_button= QPushButton("Confirm")
        #map_rssi_button=QPushButton("RSSI_MAPING")





        Func_widget_layout.addWidget(distan_option_label,0,0)
        Func_widget_layout.addWidget(LOS,0,1)
        Func_widget_layout.addWidget(NLOS,0,2)
        Func_widget_layout.addWidget(channel_selection_label,1,0)
        Func_widget_layout.addWidget(channel_textedit,1,2)
        Func_widget_layout.addWidget(channel_optimize,1,1)
        Func_widget_layout.addWidget(IMU_calibration_label,2,0)
        Func_widget_layout.addWidget(IMU_run_stop_button,2,1)
        Func_widget_layout.addWidget(IMU_state_state_label,2,2)
        Func_widget_layout.addWidget(initial_heading_label,3,0)
        Func_widget_layout.addWidget(initial_heading_display_label,3,2)
        Func_widget_layout.addWidget(initial_heading_calculation_button,3,1)
        Func_widget_layout.addWidget(ROS_node_label,4,0)
        Func_widget_layout.addWidget(Ros_node_run_button,4,1)
        Func_widget_layout.addWidget(Ros_node_state_label,4,2)
        Func_widget_layout.addWidget(camera_label, 5,0)
        Func_widget_layout.addWidget(camera_group_widget,5,1)
        Func_widget_layout.addWidget(camera_run_button,5,2)
        #Func_widget_layout.addWidget(map_rssi_button, 6,0)


        Func_widget_layout.setColumnMinimumWidth(2,70)
        # Func_widget_layout.set
        func_widget.setStyleSheet("color:white;font:bold 15px")

        # font1 = QFont()
        # font1.setPointSize(20)
        # # font1.setPixelSize(30)
        # # font1.setBold(True)
        # font1.setWeight(75)



        LOS.setStyleSheet("background-color: rgb(245,128,38);padding: 6px; border-radius:5px")
        channel_optimize.setStyleSheet("background-color: rgb(245,128,38);padding: 6px; border-radius:5px")
        # channel_textedit.setStyleSheet("background-color: rgb(245,128,38)")
        NLOS.setStyleSheet("background-color: rgb(245,128,38);padding: 6px; border-radius:5px")
        camera_run_button.setStyleSheet("background-color: rgb(245,128,38);padding: 6px; border-radius:5px")
        IMU_run_stop_button.setStyleSheet("background-color: rgb(245,128,38);padding: 6px; border-radius:5px;text_align:left")
        # IMU_state_state_label.setStyleSheet("background-color: rgb(245,128,38)")

        initial_heading_calculation_button.setStyleSheet("background-color: rgb(245,128,38);padding: 6px; border-radius:5px")
        Ros_node_run_button.setStyleSheet("background-color: rgb(245,128,38);padding: 6px; border-radius:5px;text_align:left")
        camera_run_button.setStyleSheet("background-color: rgb(245,128,38);padding: 6px; border-radius:5px")
        #map_rssi_button.setStyleSheet("background-color: rgb(245,128,38);padding: 6px; border-radius:5px")

        # camera_group.setStyleSheet("font:bold 11px")


        ###############################
        LOS.toggled.connect(change_los_nos_value)
        channel_optimize.clicked.connect(channel_on_click)
        IMU_run_stop_button.clicked.connect(imu_run_stop_click)
        initial_heading_calculation_button.clicked.connect(calculate_Azimuth)
        Ros_node_run_button.clicked.connect(stat_ros_click)
        camera_run_button.clicked.connect(start_camera_click)
        #map_rssi_button.clicked.connect(map_rssi_fun_click)


        # channel_button.clicked.connect(channel_on_click)
        # ROS_button.clicked.connect(stat_ros_click)
        # save_button.clicked.connect(save_file_click)
        # Azimuth_button.clicked.connect(calculate_Azimuth)
        # New_Azimuth_button.clicked.connect(pub_new_Azimuth)
        # Enable_gps_Azimuth.clicked.connect(enable_disable_gps_Azimuth)
        # manual_com.clicked.connect(stat_ros_manual_com_click)




        mid_layout=QHBoxLayout()
        #mid_layout.setSpacing(20)
        mid_layout.addWidget(local_widget)
        mid_layout.addWidget(remote_widget)

        top_layout=QVBoxLayout()
        # top_layout.addWidget(DIS_CCQ_widget)
        top_layout.addLayout(mid_layout)
        # top_layout.setStretchFactor(DIS_CCQ_layout,1)
        # top_layout.setStretchFactor(mid_layout,9)

        com_fun_layout= QHBoxLayout()
        com_fun_layout.addLayout(top_layout)
        com_fun_layout.addWidget(func_widget)
        com_fun_layout.setStretchFactor(mid_layout,5)
        com_fun_layout.setStretchFactor(func_widget,1.5)

        # mid_layout.addWidget(func_widget)
        # mid_layout.setStretchFactor(local_widget,4)
        # mid_layout.setStretchFactor(remote_vlayout,4)
        # mid_layout.setStretchFactor(func_widget,2)
        # mid_layout.addStretch(1)
        # gcv.addLayout(mid_layout)
        gcv.addLayout(com_fun_layout)





	

        # l.addRow('Coords:', coordsEdit)
        # l.addRow('lcoords:',r_coordsEdit)
        #h.addLayout(v)
        #l = QFormLayout()
        #h.addLayout(l)
        #coordsEdit = QLineEdit()
        #l.addRow('Coords:', coordsEdit)
        #coordsEdit.editingFinished.connect(goCoords)
        self.map = QOSM(self._widget)


        self.map.mapMoved.connect(onMapMoved)

        self.map.markerMoved.connect(onMarkerMoved)

        self.map.mapClicked.connect(onMapLClick)
        self.map.mapDoubleClicked.connect(onMapDClick)
        self.map.mapRightClicked.connect(onMapRClick)
        self.map.markerClicked.connect(onMarkerLClick)
        self.map.markerDoubleClicked.connect(onMarkerDClick)
        self.map.markerRightClicked.connect(onMarkerRClick)
        self.l_location.connect(move_l_mark)
        self.r_location.connect(move_r_mark)


        self.distance_widget=QWidget(self.map)
        self.distance_widget_layout=QHBoxLayout(self.distance_widget)
        self.distance_widget_layout.addWidget(QLabel("Distance: "))
        self.distance_label = QLabel()
        self.distance_label.setText("the distance")
        self.distance_widget_layout.addWidget(self.distance_label)
        self.distance_widget.setGeometry(50,10,180,30)
        self.distance_widget.setStyleSheet("background-color: white")
        # self.distance_label.setGeometry(50,10,120,30)
        # self.distance_label.setStyleSheet("background-color: rgb(245,128,38);padding: 6px; border-radius:5px")

        # self.distance_label.setGeometry()

        # self.distance_dock= QDockWidget()
        # self.distance_dock.setWidget(self.distance_label)
        # self.distance_dock.setFloating(True)
        # self.addDockWidget(Qt.LeftDockWidgetArea,self.distance_dock)
        # self._widget.addDockWidget(Qt.LeftDockWidgetArea, self.distance_dock)






        gcv.addWidget(self.map)
        gcv.setStretchFactor(com_fun_layout, 5)
        gcv.setStretchFactor(self.map, 4)
        self.map.setSizePolicy(
            QSizePolicy.MinimumExpanding,
            QSizePolicy.MinimumExpanding)



        # l_com_timer = QTimer()
        # self.l_com_timer.setSingleShot(True)
        self.com_timer.timeout.connect(move_compass)
        self.com_timer.start(200)

        self.rssi_timer.timeout.connect(change_rssi)
        self.rssi_timer.start(1000)

        self.baro_timer.timeout.connect(change_baroalt)
        self.baro_timer.start(1000)

        self.ConnInd_timer.timeout.connect(local_connection_indicator)
        self.ConnInd_timer.start(1000)

        self.imu_state_timer=QTimer()
        self.imu_state_timer.setInterval(2000)
        # self.imu_state_timer.interval(2000)
        self.imu_state_timer.timeout.connect(imu_state_display)
        self.distance_timer = QTimer()
        self.distance_timer.setInterval(1000)
        self.distance_timer.timeout.connect(calculate_distance)
        self.distance_timer.start()
        # self.monitor_ccq = CCQ_threading()

        self.CONN_state=conn_Test()
        self.CONN_state.lBBB_conn.connect(local_bbb_conn)
        self.CONN_state.rBBB_conn.connect(remote_bbb_conn)
        self.CONN_state.start()





        # self.memory_tracker=tracker.SummaryTracker(o)










        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('gps_com_node_v2'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setStyleSheet("background-color:rgb(0,68,124)")



        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.map.waitUntilReady()

        self.map.centerAt(32.731263, -97.114334)
        #self.map.centerAt(38.833005, -77.117415)
        self.map.setZoom(12)
        # Many icons at: https://sites.google.com/site/gmapsdevelopment/
        coords = self.map.center()
        self.map.addMarker("local GPS", *coords, **dict(
            icon="file:///home/dnc2/Documents/catkin_ws/src/gps_com_node _v2/src/gps_com_node_v2/local_uav.png",
            draggable=True,
            title="locat GPS marker!"
        ))

        coords = coords[0] , coords[1] 
        self.map.addMarker("remote GPS", *coords, **dict(
            # icon="https://thumb.ibb.co/bvE9FT/remote_uav.png",
            icon= "file:///home/dnc2/Documents/catkin_ws/src/gps_com_node _v2/src/gps_com_node_v2/remote_uav.png",
            draggable=True,
            title="remote GPS marker"
        ))
	sub1 = rospy.Subscriber("/local_gps", Float64MultiArray, local_callback)
        sub2 = rospy.Subscriber("/remote_gps", Float64MultiArray, remote_callback)
        sub3=rospy.Subscriber("/local_rssi",Int32,l_rssi_callback)
        sub4=rospy.Subscriber("/remote_rssi",Int32,r_rssi_callback)
        sub5=rospy.Subscriber("/local_com",Float64,l_com_callback)
        sub6=rospy.Subscriber("/remote_com",Float64,r_com_callback)
        sub7=rospy.Subscriber("/local_imucal_msg", Int32, local_imucal_callback)
        sub8=rospy.Subscriber("/remote_imucal_msg", Int32, remote_imucal_callback)
        sub9=rospy.Subscriber("/local_baro",Float64MultiArray,l_baro_callback)
        sub10=rospy.Subscriber("/remote_baro",Float64MultiArray,r_baro_callback)







	#time.sleep(10)

	

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        # self.save_file_processing.terminate()
        # os.system("kill ")
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
    
        
        
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


class CCQ_threading(QThread):
    remote_ccq=pyqtSignal(object)

    def __init__(self, parent=None):
        QThread.__init__(self, parent)


    def run(self):
        while(True):
            r_ccq_value= Remote_NanoTransmitCCQ.RemoteGetTransmitCCQ()
            print("remote_ccq: ",r_ccq_value)
            self.sleep(10)



def testconn(host,port):
    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sk.settimeout(1)
    var=0
    try:
        sk.connect((host, port))
        var=1
        return var
    except Exception:
        var=2
        return var
    sk.close()

class conn_Test(QThread):
    lBBB_conn= pyqtSignal(object)
    rBBB_conn=pyqtSignal(object)
    Rpi_conn = pyqtSignal(object)
    opt_conn= pyqtSignal(object)
    rnon_conn= pyqtSignal(object)

    def __init__(self):
        QThread.__init__(self,parent=None)
    def nanotest(self):
        return testconn('192.168.33.110',80)
    def remote_BBB_conn(self):
        return testconn('192.168.33.130',80)
    def local_BBB_conn(self):
        return testconn('192.168.33.131',80)
    def rapi_conn(self):
        return testconn('192.168.33.133',22)
    def optical_conn(self):
        return testconn('192.168.33.181',80)

    def run(self):
        while True:
            #print time.strftime('%Y-%m-%d %H:%M:%S')
            a=self.local_BBB_conn()

            self.lBBB_conn.emit(a)
            b=self.remote_BBB_conn()
            self.rBBB_conn.emit(b)
            # c=self.rapi_conn()
            # self.Rpi_conn.emit(c)
            # d=self.optical_conn()
            # self.opt_conn.emit(d)
            # e=self.nanotest()
            # self.rnon_conn.emit(e)
            # print("a", a, "b", b,"d",d,"e",e)

            time.sleep(20)



