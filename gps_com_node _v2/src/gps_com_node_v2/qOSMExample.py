#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
sys.path.insert(0, "../")

import qOSM
qOSM.use("PyQt5")

from qOSM.common import QOSM
import time
if qOSM.get_backed() == "PyQt5":
    from PyQt5.QtCore import *
    from PyQt5.QtWidgets import *
elif qOSM.get_backed() == "PyQt4":
    from PyQt4.QtCore import *
    from PyQt4.QtGui import *


if __name__ == '__main__':
    @profile
    def goCoords():
        def resetError():
            coordsEdit.setStyleSheet('')

        try:
            latitude, longitude = coordsEdit.text().split(",")
        except ValueError:
            coordsEdit.setStyleSheet("color: red;")
            QTimer.singleShot(500, resetError)
        else:
            map.centerAt(latitude, longitude)
            # map.moveMarker("MyDragableMark", latitude, longitude)


    def onMarkerMoved(key, latitude, longitude):
        print("Moved!!", key, latitude, longitude)
        coordsEdit.setText("{}, {}".format(latitude, longitude))
        map.moveMarker("local GPS",latitude+0.001, longitude+0.001)
    


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

    app = QApplication(sys.argv)
    #w = QDialog()
    w=QWidget()
    h = QVBoxLayout(w)
    v =QHBoxLayout()
    #l = QFormLayout()
    #v.addLayout(l)
    coordsEdit = QLineEdit()
    r_coordsEdit=QLineEdit()
    lgps_label=QLabel('LGPS:')
    rgps_label=QLabel('RGPS:')
    v.addWidget(lgps_label)
    v.addWidget(coordsEdit)
    v.addWidget(rgps_label)
    v.addWidget(r_coordsEdit)
    #l.addRow('Coords:', coordsEdit)
    #l.addRow('lcoords:',r_coordsEdit)
    h.addLayout(v)
    coordsEdit.editingFinished.connect(goCoords)
    map = QOSM(w)
    map.mapMoved.connect(onMapMoved)
    map.markerMoved.connect(onMarkerMoved)
    map.mapClicked.connect(onMapLClick)
    map.mapDoubleClicked.connect(onMapDClick)
    map.mapRightClicked.connect(onMapRClick)
    map.markerClicked.connect(onMarkerLClick)
    map.markerDoubleClicked.connect(onMarkerDClick)
    map.markerRightClicked.connect(onMarkerRClick)
    h.addWidget(map)
    map.setSizePolicy(
        QSizePolicy.MinimumExpanding,
        QSizePolicy.MinimumExpanding)
    w.show()

    map.waitUntilReady()

    map.centerAt(32.7471012, -96.883642)
    map.setZoom(12)
    # Many icons at: https://sites.google.com/site/gmapsdevelopment/
    coords = map.center()
    map.addMarker("local GPS", *coords, **dict(
        icon="http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_gray.png",
        # icon="file:///home/dnc3/Downloads/local_uav.png",
        draggable=True,

        title="local GPS!"
    ))

    coords = coords[0] + 0.1, coords[1] + 0.1
    map.addMarker("remote GPS", *coords, **dict(
        icon="http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_red.png",
        draggable=True,
        title="remote gps!"
    ))
    lat=32.7471012
    lon=-96.883642
##    for i in range(1,10000):
##        map.moveMarker("local GPS",lat,lon)
##        lat+=0.0001
##        lon+=0.0001
##        #time.sleep(0.1)
    
    sys.exit(app.exec_())
