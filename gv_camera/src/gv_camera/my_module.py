
import os

import sys
import rospkg
import rospy
from PyQt5 import uic, QtGui, QtCore
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.uic import *
from PyQt5.QtGui import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from imutils.video import FileVideoStream
from imutils.video import FPS
import numpy as np
import argparse
import imutils
import time
from datetime import datetime
import cv2
import Queue
# from dnn_caffe_model import dnn_caffe

  
import threading

class rtsp_camera_thread(QThread):
    global state_wifi


    rtsp_signal = pyqtSignal(object)

    def __init__(self):
        QThread.__init__(self)
        self.capture = cv2.VideoCapture("rtsp://192.168.33.133:8554//CH001.sdp")
        # self.capture.set(3,640)
        # self.capture.set(4,480)
        FILE_OUTPUT = "/home/dnc2/Desktop/save_data/optical_camera_%s.avi" % str(
            datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
        if os.path.isfile(FILE_OUTPUT):
            os.remove(FILE_OUTPUT)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        # fourcc = cv2.VideoWriter_fourcc(*'H264')
        # fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(FILE_OUTPUT, fourcc, 7, (480, 480))
        self.state_wifi = 1


    #
    def capture_fram(self):
        if self.state_wifi==0:
            self.capture = cv2.VideoCapture("rtsp://192.168.33.133:8554//CH001.sdp")

            print (self.state_wifi)
            self.state_wifi=1
            time.sleep(10)
        self.ret, self.frame = self.capture.read()
        if self.ret == True:
            self.Frame = self.frame
            return self.frame


    def run(self):

        while(True):
            var = self.capture_fram()
            self.rtsp_signal.emit(var)
            self.usleep(10000)
            # time.sleep(0.1)

            # if self.ret == True:





                #########################################################################
                # save_frame = cv2.flip(self.frame, 0)

        # time.sleep(0.1)

class MyPlugin(Plugin):
    global state_wifi


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
        # if not args.quiet:
        #     print 'arguments: ', args
        #     print 'unknowns: ', unknowns

        self.runing = False
        self.capture_thread = None
        # self.q = Queue.Queue()
        # self.i = 4


        self.state_wifi=1
        # Create QWidget
        self._widget = QWidget()
        v = QVBoxLayout (self._widget)
        self.video_widget =QLabel()
        self.video_widget.setMinimumSize(420,420)
        self.GV_button_start=QPushButton()
        self.GV_button_start.setMaximumSize(160,20)
        self.GV_button_start.setText("START")
        self.GV_button_end = QPushButton()
        self.GV_button_end.setMaximumSize(160, 20)
        self.GV_button_end.setText("END")

        h = QHBoxLayout()
        h.addStretch(1)
        h.addWidget(self.GV_button_start)
        h.addWidget(self.GV_button_end)
        h.addStretch(1)

        v.addWidget(self.video_widget)
        v.addLayout(h)
        self.GV_button_start.clicked.connect(self.start_clicked)
        self.GV_button_end.clicked.connect(self.end_clicked)
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('gv_camera'), 'resource', 'MyPlugin.ui')
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
        test_qthread= rtsp_camera_thread()




        # self.dispaly_thread.rtsp_single.connect(self.update_frame)


    # def capture_rtsp(self):
    #     self.capture = cv2.VideoCapture("rtsp://192.168.33.181:8554//CH001.sdp")
    #     # self.capture.set(3,640)
    #     # self.capture.set(4,480)
    #     FILE_OUTPUT = "/home/dnc2/Desktop/save_data/optical_camera_%s.avi" % str(
    #         datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
    #     if os.path.isfile(FILE_OUTPUT):
    #         os.remove(FILE_OUTPUT)
    #     fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    #     # fourcc = cv2.VideoWriter_fourcc(*'H264')
    #     # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #     self.out = cv2.VideoWriter(FILE_OUTPUT, fourcc, 7, (480, 480))
    #     while True:
    #         try:
    #             self.ret, self.frame = self.capture.read()
    #             if self.ret == True:
    #                 image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
    #                 # image = self.rescale_frame(image,75)
    #                 # image = cv2.cvtColor(Frame, cv2.COLOR_BGR2RGB)
    #                 heigh, width, channel = image.shape
    #                 step = channel * width
    #                 qimg = QImage(image.data, width, heigh, step, QImage.Format_RGB888)
    #
    #                 p = qimg.scaled(550, 550, Qt.KeepAspectRatio)
    #                 self.video_widget.setPixmap(QPixmap.fromImage(p))
    #                 time.sleep(0.05)
    #
    #                 #######################################################
    #         except Exception as e:
    #             print (e)




    @pyqtSlot()
    def start_clicked(self):
        self.GV_button_start.setEnabled(False)
        self.GV_button_end.setEnabled(True)
        self.test=rtsp_camera_thread()
        self.test.rtsp_signal.connect(self.update_frame)
        time.sleep(0.5)
        self.test.start()
        # test_.start()

        # dis_rtsp= threading.Thread(target=self.capture_rtsp)
        # dis_rtsp.start()





    @pyqtSlot()
    def end_clicked(self):
        self.GV_button_start.setEnabled(True)
        self.GV_button_end.setEnabled(False)
        self.test.quit()
        self.runing = False

        # self.dispaly_thread.terminate()
        # self.release()
        # self.out.release()
        # self.capture_thread.


    #
    def update_frame(self,Frame):
        try:
            # self.ret, self.frame = self.capture.read()
            # if self.ret==True:
                self.frame=Frame
                image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                # image = self.rescale_frame(image,75)
                # image = cv2.cvtColor(Frame, cv2.COLOR_BGR2RGB)
                heigh, width, channel = image.shape
                step = channel * width
                qimg = QImage(image.data, width, heigh, step, QImage.Format_RGB888)

                p = qimg.scaled(550, 550, Qt.KeepAspectRatio)
                self.video_widget.setPixmap(QPixmap.fromImage(p))

                    #######################################################
        except Exception as e:
            print (e)
            self.state_wifi=0
            # print (self.state_wifi)





    def shutdown_plugin(self):
        # TODO unregister all publishers here
        # self.out.release()
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


