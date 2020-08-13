import socket

from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QWidget, QLabel, QPushButton,QHBoxLayout,QVBoxLayout,QApplication
from PyQt5.QtCore import Qt,QThread,pyqtSlot,pyqtSignal
import sys
import os
import time
import threading


class ACLSYSTEM(QWidget):
    conn_state_var=0

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):

        self.setStyleSheet("background-color:rgb(0,68,124)")
        self.green_icon = QPixmap((
                                 "/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/greenenergy.png"))
        self.green_icon = self.green_icon.scaled(40, 40, Qt.KeepAspectRatio)

        self.red_icon = QPixmap((
                               "/home/dnc2/Documents/catkin_ws/src/directional_antenna_system/src/directional_antenna_system/redenergy.png"))
        self.red_icon = self.red_icon.scaled(40, 40, Qt.KeepAspectRatio)
        self.roscore_button=QPushButton("Roscore")
        self.roscore_button.setStyleSheet("background-color: rgb(192,197,197);color:white;font:20px;padding: 15px; border-radius:5px")
        self.roscore_button.setEnabled(False)
        self.rqt_button=QPushButton("Interface")
        self.rqt_button.setStyleSheet("background-color: rgb(192,197,197);color:white;font:20px;padding: 15px; border-radius:5px")
        self.rqt_button.setEnabled(False)


        self.roscore_button.clicked.connect(self.run_roscore)
        self.rqt_button.clicked.connect(self.run_rqt)



        self.conn_led=QLabel()
        self.conn_led.setPixmap(self.red_icon)
        self.conn_led.setAlignment(Qt.AlignCenter)

        Hlay=QVBoxLayout()
        # Hlay.addStretch(1)
        Hlay.addWidget(self.conn_led)
        Hlay.addWidget(self.roscore_button)
        Hlay.addWidget(self.rqt_button)
        # Hlay.addStretch(1)
        self.setLayout(Hlay)
        self.setGeometry(300,300,300,200)
        self.setWindowTitle("ACL_SYSTEM")
        self.master_conn_threading=conn_Test()
        self.master_conn_threading.master_conn.connect(self.conn_state)
        self.master_conn_threading.start()

        self.show()

    def run_roscore(self):
        self.roscore_button.setEnabled(False)
        self.rqt_button.setEnabled(True)

        open_roscore_threading=threading.Thread(target=self.open_roscore_fun)
        open_roscore_threading.start()

        self.roscore_button.setStyleSheet(
            "background-color: rgb(192,197,197);color:white;font:20px;padding: 15px; border-radius:5px")


    def open_roscore_fun(self):
        os.system("/home/dnc2/Documents/catkin_ws/src/ACL_SYSTEM/shell_scripts/run_roscore.sh")


    def run_rqt(self):
        open_rqt_threading=threading.Thread(target=self.open_rqt_fun)
        open_rqt_threading.start()

    def open_rqt_fun(self):
        os.system("/home/dnc2/Documents/catkin_ws/src/ACL_SYSTEM/shell_scripts/run_rqt.sh")

    def conn_state(self,var):
        # print(var)
        if var==1:
            if self.conn_state_var==0:
                self.roscore_button.setEnabled(True)
                self.conn_led.setPixmap(self.green_icon)
                self.rqt_button.setStyleSheet(
                    "background-color: rgb(245,128,38);color:white;font:20px;padding: 15px; border-radius:5px")
                # self.roscore_button.setEnabled(True)
                self.roscore_button.setStyleSheet(
                    "background-color: rgb(245,128,38);color:white;font:20px;padding: 15px; border-radius:5px")
                self.conn_state_var=1

        else:
            self.roscore_button.setEnabled(False)
            self.conn_led.setPixmap(self.red_icon)
            # self.roscore_button.setEnabled(False)
            self.roscore_button.setStyleSheet(
                "background-color: rgb(192,197,197);color:white;font:20px;padding: 15px; border-radius:5px")



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
    master_conn= pyqtSignal(object)


    def __init__(self):
        QThread.__init__(self,parent=None)
    def mastertest(self):
        return testconn('192.168.33.21',22)


    def run(self):
        while True:
            #print time.strftime('%Y-%m-%d %H:%M:%S')
            a=self.mastertest()

            self.master_conn.emit(a)
            # c=self.rapi_conn()
            # self.Rpi_conn.emit(c)
            # d=self.optical_conn()
            # self.opt_conn.emit(d)
            # e=self.nanotest()
            # self.rnon_conn.emit(e)
            # print("a", a, "b", b,"d",d,"e",e)

            time.sleep(15)




if __name__== '__main__' :
    app= QApplication(sys.argv)
    fun=ACLSYSTEM()
    sys.exit(app.exec())









