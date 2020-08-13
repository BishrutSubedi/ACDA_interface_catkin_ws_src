import os
import sys
import time
import threading
import math
import multiprocessing
import rospkg
import rospy
from PyQt5.QtGui import QFont
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String

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
    com_timer = QTimer()
    rssi_timer = QTimer()
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
        self._enable_gps=0








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
        def channel_on_click(self):

            print('test')
            os.system("sudo /home/dnc2/Desktop/testpython")
            print('done')

        def stat_ros_nodes():
            print("stat ROS node ...")
            os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/start_local.sh")


        def stat_ros_manual_com_nodes():
            print("stat ROS manual com node ...")
            os.system("/home/dnc2/Documents/catkin_ws/devel/sshnode/start_local_v4.sh")




        @pyqtSlot()
        def stat_ros_click():
            if star_ros_threading.isAlive()== True:
                pass
            else:
                # star_ros_threading._stop()
                star_ros_threading.start()

        @pyqtSlot()
        def save_file_click():
            if save_file_processing.is_alive()== True:
                pass
            else:
                save_file_processing.start()


        @pyqtSlot()
        def stat_ros_manual_com_click():
            if star_ros_manual_com_threading.isAlive()== True:
                pass
            else:
                star_ros_manual_com_threading.start()


        def save_topic_fun():
            print("save date ...")
            # self._pid= save_file_processing.pid
            os.system("/home/dnc2/Desktop/save_rosbag.sh")

        @pyqtSlot()
        def calculate_Azimuth():
            if l_coordsEdit.text() and l_coordsEdit.text().find(",") !=-1:
                llocation_string = l_coordsEdit.text()
                l_lat_lon = llocation_string.split(",")
                try:
                    l_latitiude = float(l_lat_lon[0])
                    l_longitude = float(l_lat_lon[1])
                except:
                    Azimuth_warning()
                    return

            if r_coordsEdit.text() and r_coordsEdit.text().find(",") != -1:
                rlocation_string = r_coordsEdit.text()
                r_lat_lon = rlocation_string.split(",")
                try:
                    r_latitiude = float(r_lat_lon[0])
                    r_longitude = float(r_lat_lon[1])
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

            Azimuth_data.setText(" %0.1f"%Azimuth)





        def Azimuth_warning():
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Azimuth calculation")
            msg.setText("The locations of local and remote UAVs cannot be empty.\nPlease input locations(latitude,longitude) correctly!")
            msg.exec_()


        @pyqtSlot()
        def pub_new_Azimuth():
            try:
                N_Azimuth = float(Azimuth_data.text())
            except:
                Azimuth_warning()
                return
            if math.fabs(N_Azimuth)>= 360:
                Azimuth_warning()
                return
            pub_Azimuth.publish(N_Azimuth)


        @pyqtSlot()
        def enable_disable_gps_Azimuth():
            if self._enable_gps==1:
                self._enable_gps=0
                Enable_gps_Azimuth.setText("Disable GPS Azimuth")
                for i in range(5):
                    pub_gps_Azimuth_enable.publish(self._enable_gps)
                    time.sleep(0.2)
            else:
                self._enable_gps=1
                Enable_gps_Azimuth.setText("Enable GPS Azimuth")
                for i in range(5):
                    pub_gps_Azimuth_enable.publish(self._enable_gps)
                    time.sleep(0.2)




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

        # def close_process():
        #
        #     save_file_processing.terminate()
        #     print("close widget")
        #

        # Create QWidget
        ############################################
        #public Azimuth

        pub_Azimuth = rospy.Publisher('lnewazimuth', Float64, queue_size=10)
        pub_gps_Azimuth_enable = rospy.Publisher('lenablegpsazimuth', Int32, queue_size = 10)
        # rospy.init_node('Azimuth', anonymous=True)





        self._widget = QWidget()
        gcv=QVBoxLayout(self._widget)



        v = QVBoxLayout()
        h = QHBoxLayout()

        FUN_FONT= QFont()
        FUN_FONT.setBold(True)
        FUN_FONT.setPointSize(14)
        FUN_FONT.setWeight(87)

        #ros_threading
        #############################
        star_ros_threading = threading.Thread(target= stat_ros_nodes)
        # save_file_processing = multiprocessing.Process(target= save_topic_fun)
        save_file_processing = threading.Thread(target= save_topic_fun)
        star_ros_manual_com_threading = threading.Thread(target= stat_ros_manual_com_nodes)




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
        l_h2.addStretch()
        # l_h1.setSpacing(0)
        local_Vlayout.addWidget(l_com_widget)
        local_Vlayout.addLayout(l_h1)
        local_Vlayout.addLayout(l_h2)
        local_widget.setMinimumSize(450,300)
        local_widget.setMaximumWidth(500)



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
        r_h2.addStretch()

        remote_vlayout.addWidget(r_com_widget)
        remote_vlayout.addLayout(r_h1)
        remote_vlayout.addLayout(r_h2)
        remote_widget.setMinimumSize(450,300)
        # remote_widget.setMinimumSize(local_widget.width())


        ##################################
        # functional buttons

        func_widget = QWidget()
        Func_widget_layout= QVBoxLayout(func_widget)
        one_master= QPushButton('One Master')
        manual_com= QPushButton('manual compass')
        master_layout = QHBoxLayout()
        master_layout.addWidget(one_master)
        master_layout.addWidget(manual_com)



        save_button = QPushButton("Save File")
        ROS_button = QPushButton("Start ROS\nNodes")
        channel_button = QPushButton("Select\nChannel")
        Azimuth_button = QPushButton("Azimuth")
        save_channel_widget = QWidget()
        save_channel_layout = QHBoxLayout(save_channel_widget)
        save_channel_layout.addWidget(ROS_button)
        save_channel_layout.addWidget(channel_button)


        Azimuth_widget = QWidget()
        Azimuth_data = QLineEdit('Heading')
        Azimuth_widget_layout = QHBoxLayout(Azimuth_widget)
        Azimuth_widget_layout.addWidget(Azimuth_data)
        Azimuth_widget_layout.addWidget(Azimuth_button)
        Enable_gps_Azimuth = QPushButton('Disable_GPS_Amizuth')
        New_Azimuth_button= QPushButton('NEW Azimuth')




        # font1 = QFont()
        # font1.setPointSize(20)
        # # font1.setPixelSize(30)
        # # font1.setBold(True)
        # font1.setWeight(75)

        # save_button.setFont(font1)
        one_master.setStyleSheet("color:white;background-color: rgb(245,128,38); padding: 12px; border-radius:20px; font:bold 20px ")
        manual_com.setStyleSheet("color:white;background-color: rgb(245,128,38); padding: 12px; border-radius:20px; font:bold 20px ")

        save_button.setStyleSheet("color:white;background-color: rgb(245,128,38); padding: 12px; border-radius:20px; font:bold 20px ")

        channel_button.setStyleSheet("color:white;background-color: rgb(245,128,38); padding: 15px; border-radius:20px; font:bold 20px ")

        ROS_button.setStyleSheet("color:white;background-color: rgb(245,128,38); padding: 15px; border-radius:20px; font:bold 20px ")
        Azimuth_button.setStyleSheet("color:white;background-color: rgb(245,128,38); padding: 12px; border-radius:20px; font:bold 20px ")
        Azimuth_data.setStyleSheet("color:white;background-color: rgb(245,128,38); padding: 12px; border-radius:20px; font:bold 20px ")
        New_Azimuth_button.setStyleSheet("color:white;background-color: rgb(245,128,38); padding: 12px; border-radius:20px; font:bold 20px ")
        Enable_gps_Azimuth.setStyleSheet("color:white;background-color: rgb(245,128,38); padding: 12px; border-radius:20px; font:bold 20px ")
        # button signals
        ###############################

        channel_button.clicked.connect(channel_on_click)
        ROS_button.clicked.connect(stat_ros_click)
        save_button.clicked.connect(save_file_click)
        Azimuth_button.clicked.connect(calculate_Azimuth)
        New_Azimuth_button.clicked.connect(pub_new_Azimuth)
        Enable_gps_Azimuth.clicked.connect(enable_disable_gps_Azimuth)
        manual_com.clicked.connect(stat_ros_manual_com_click)




        Func_widget_layout.addLayout(master_layout)
        Func_widget_layout.addWidget(save_button)
        # Func_widget_layout.addWidget(channel_button)
        Func_widget_layout.addWidget(save_channel_widget)
        # Func_widget_layout.addWidget(ROS_button)
        Func_widget_layout.addWidget(Enable_gps_Azimuth)
        Func_widget_layout.addWidget(New_Azimuth_button)
        # Func_widget_layout.addWidget(Azimuth_button)
        Func_widget_layout.addWidget(Azimuth_widget)

        mid_layout=QHBoxLayout()
        #mid_layout.setSpacing(20)
        mid_layout.addWidget(local_widget)
        mid_layout.addWidget(remote_widget)
        mid_layout.addWidget(func_widget)
        mid_layout.setStretchFactor(local_widget,4)
        mid_layout.setStretchFactor(remote_vlayout,4)
        mid_layout.setStretchFactor(func_widget,2)
        # mid_layout.addStretch(1)
        gcv.addLayout(mid_layout)






	

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
        gcv.addWidget(self.map)
        gcv.setStretchFactor(mid_layout, 3)
        gcv.setStretchFactor(self.map, 5)
        self.map.setSizePolicy(
            QSizePolicy.MinimumExpanding,
            QSizePolicy.MinimumExpanding)



        # l_com_timer = QTimer()
        # self.l_com_timer.setSingleShot(True)
        self.com_timer.timeout.connect(move_compass)
        self.com_timer.start(200)

        self.rssi_timer.timeout.connect(change_rssi)
        self.rssi_timer.start(1000)

        # self.memory_tracker=tracker.SummaryTracker()










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

        self.map.centerAt(33.214039, -97.128007)
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
