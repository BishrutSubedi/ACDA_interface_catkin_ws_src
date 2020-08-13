import os
import sys
import time
import rospkg
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32

sys.path.insert(0, "../")
import qOSM
qOSM.use("PyQt5")
from qOSM.common import QOSM
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Signal
from compass import CompassWidget
from pympler import tracker
# import gc
# import objgraph
# import random


if qOSM.get_backed() == "PyQt5":
    from PyQt5.QtCore import *
    from PyQt5.QtWidgets import *

class MyPlugin(Plugin):
    sig_sysmsg = Signal(str)
    l_location=pyqtSignal(float,float)
    r_location=pyqtSignal(float,float)
    l_rssi = pyqtSignal(int)
    r_rssi = pyqtSignal(int)
    l_com = pyqtSignal(float)
    r_com = pyqtSignal(float)



    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')
        rp = rospkg.RosPack()
        self._lrssivalue=''
        self._rrssivalue=''
        self._lcomvalue=''
        self._rcomvalue=''
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
            # print 'arguments: ', args
            # print 'unknowns: ', unknowns

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
            l_coordsEdit.setText("{0:.8}, {1:.8}".format(lat_l,lon_l))
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
            r_coordsEdit.setText("{0:.8}, {1:.8}".format(lat_r,lon_r))

	    #time.sleep(0.01)

           
        def remote_callback(rlocation):
            rlat=rlocation.data[0]
            rlon=rlocation.data[1]
            #move_mark(self,lat,lon)
            self.r_location.emit(rlat,rlon)


        def change_l_rssi(rssi1):
            l_rssi_text.setText(str(rssi1))
            #time.sleep(0.01)




        def change_r_rssi(rssi2):
            #self._rrssivalue=str(rssi2)

            r_rssi_text.setText(str(rssi2))
            #time.sleep(0.01)



        def l_rssi_callback(rssil):
            self.l_rssi.emit(rssil.data)
            del rssil
            #l_rssi_text.update()
            #l_rssi_text.setText(str(rssi.data))


        def r_rssi_callback(rssir):
            self.r_rssi.emit(rssir.data)
            del rssir
            # self.memory_tracker.print_diff()
            # print(sys.getsizeof(move_l_compass(headidng1)))
            #r_rssi_text.setText(str(rssi.data))


        def move_l_compass(headidng1):
            l_com_text.setText("%.1f" % headidng1)
            time.sleep(0.01)

            l_com_widget.setAngle(headidng1)
            time.sleep(0.01)
            del headidng1

        def move_r_compass(headidng2):
            r_com_text.setText("%.1f" % headidng2)
            time.sleep(0.01)
            r_com_widget.setAngle(headidng2)
            time.sleep(0.01)
            headidng2 = None
            #del headidng2




            # check the memory leak ###########################################
            # gc.collect()
            # objgraph.show_most_common_types(limit=500)
            # objgraph.show_backrefs(random.choice(objgraph.by_type('MyPlugin')),filename="MyPlugin_refs.png")
            # objgraph.show_refs(r_com_text,filename="MyPlugin_refs.png")

            #################################################


        def l_com_callback(heading):
            #l_com_text.setText("%.1f" % heading.data)
            self.l_com.emit(heading.data)
            heading.data =None
            #self.update()
            #l_com_widget.setAngle(heading.data)


        def r_com_callback(heading):
            #r_com_text.setText("%.1f" % heading.data)
            self.r_com.emit(heading.data)
            del heading
            #r_com_widget.setAngle(heading.data)

            
        

        # Create QWidget
        self._widget = QWidget()
        gcv=QVBoxLayout(self._widget)



        h = QVBoxLayout()
        #h = QVBoxLayout(w)
        v = QHBoxLayout()
        # l = QFormLayout()
        # v.addLayout(l)
        l_coordsEdit = QLineEdit()
        r_coordsEdit = QLineEdit()
        lgps_label = QLabel('LGPS:')
        rgps_label = QLabel('RGPS:')
        v.addWidget(lgps_label)
        v.addWidget(l_coordsEdit)
        v.addWidget(rgps_label)
        v.addWidget(r_coordsEdit)
        h.addLayout(v)


	
        l_com_widget=CompassWidget()
        r_com_widget = CompassWidget()
        l_rssi_lable=QLabel('local_rssi')
        r_rssi_lable=QLabel('remote_rssi')
        l_rssi_text=QLineEdit()
        l_rssi_text.setText(self._lrssivalue)
        l_rssi_text.setMaximumWidth(70)
        r_rssi_text=QLineEdit()
        r_rssi_text.setText(self._rrssivalue)
        r_rssi_text.setMaximumWidth(70)

        l_com_label=QLabel('LCOM')
        r_com_label=QLabel('RCOM')
        l_com_text=QLineEdit()
        l_com_text.setText(self._lcomvalue)
        l_com_text.setMaximumWidth(80)
        r_com_text=QLineEdit()
        r_com_text.setText(self._rcomvalue)
        r_com_text.setMaximumWidth(80)
        l_h_layout=QHBoxLayout()
        r_h_layout = QHBoxLayout()

        l_h_layout.addStretch(1)
        l_h_layout.addWidget(l_rssi_lable)
        l_h_layout.addWidget(l_rssi_text)
        l_h_layout.addWidget(l_com_label)
        l_h_layout.addWidget(l_com_text)
        l_h_layout.addStretch(1)

        r_h_layout.addStretch(1)
        r_h_layout.addWidget(r_rssi_lable)
        r_h_layout.addWidget(r_rssi_text)
        r_h_layout.addWidget(r_com_label)
        r_h_layout.addWidget(r_com_text)
        r_h_layout.addStretch(1)

        mid_layout=QHBoxLayout()
        #mid_layout.setSpacing(20)

        mid_layout.addLayout(l_h_layout)
        mid_layout.addLayout(r_h_layout)



        c_h_l=QHBoxLayout()
        c_h_l.addWidget(l_com_widget)
        c_h_l.addWidget(r_com_widget)
        #c_h_l.SetMinimumSize(240)
        gcv.addLayout(c_h_l)
        gcv.addLayout(mid_layout)
        gcv.addLayout(h)
        gcv.setStretchFactor(c_h_l,3)
        gcv.setStretchFactor(h,5)



	

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
        #self.map.markerMoved.connect(onMarkerMoved)
        self.map.mapClicked.connect(onMapLClick)
        self.map.mapDoubleClicked.connect(onMapDClick)
        self.map.mapRightClicked.connect(onMapRClick)
        self.map.markerClicked.connect(onMarkerLClick)
        self.map.markerDoubleClicked.connect(onMarkerDClick)
        self.map.markerRightClicked.connect(onMarkerRClick)
        self.l_location.connect(move_l_mark)
	self.r_location.connect(move_r_mark)	
        h.addWidget(self.map)
        self.map.setSizePolicy(
            QSizePolicy.MinimumExpanding,
            QSizePolicy.MinimumExpanding)

        self.l_com.connect(move_l_compass)
        self.r_com.connect(move_r_compass)
        self.l_rssi.connect(change_l_rssi)
        self.r_rssi.connect(change_r_rssi)

        self.memory_tracker=tracker.SummaryTracker()









        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('gps_com_node'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
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

        self.map.centerAt(32.7471012, -96.883642)
        self.map.setZoom(12)
        # Many icons at: https://sites.google.com/site/gmapsdevelopment/
        coords = self.map.center()
        self.map.addMarker("local GPS", *coords, **dict(
            icon="http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_gray.png",
            draggable=True,
            title="locat GPS marker!"
        ))

        coords = coords[0] + 0.1, coords[1] + 0.1
        self.map.addMarker("remote GPS", *coords, **dict(
            icon="http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_red.png",
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
if __name__='__main__':
    app=QApplication(sys.argv)
    ex=MyPlugin()
    sys.exit(app.exec_())

