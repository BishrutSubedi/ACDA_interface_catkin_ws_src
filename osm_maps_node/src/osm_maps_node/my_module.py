import os
import rospkg
import sys


import rospy
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, "../")
import qOSM
qOSM.use("PyQt5")
from qOSM.common import QOSM
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer, Signal

if qOSM.get_backed() == "PyQt5":
    from PyQt5.QtCore import *
    from PyQt5.QtWidgets import *

class MyPlugin(Plugin):
    sig_sysmsg = Signal(str)
    l_location=pyqtSignal(float,float)
    r_location=pyqtSignal(float,float)

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')
        rp = rospkg.RosPack()

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
           
        def remote_callback(rlocation):
            rlat=rlocation.data[0]
            rlon=rlocation.data[1]
            #move_mark(self,lat,lon)
            self.r_location.emit(rlat,rlon)
            
        

        # Create QWidget
        self._widget = QWidget()

        h = QVBoxLayout(self._widget)
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


        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('osm_maps_node'), 'resource', 'MyPlugin.ui')
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
