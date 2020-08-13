import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import time
import gc

class RCompassWidget(QWidget):
    angleChanged = pyqtSignal(float)
    angleChanged1 = pyqtSignal(float)
    def __init__(self, parent=None):

        QWidget.__init__(self, parent)
        # gc.enable()
        # gc.set_debug(gc.DEBUG_COLLECTABLE | gc.DEBUG_UNCOLLECTABLE | gc.DEBUG_LEAK )

        self._angle = 0.0
        self._margins = 5
	self.setStyleSheet("background-color:white")
        self._pointText = {0: "N" , 90: "E",  180: "S",
                          270: "W"}

        self._pointText1={30:'30',60:'60',120:'120',150:'150',210:'210',240:'240',300:'300',330:'330'}
        # self.pixmap=QPixmap("compass.jpg")
        self.pixmap = QPixmap()
        # scale = min((self.width() - self._margins) / 120.0,
        #             (self.height() - self._margins) / 120.0)
        # self.pixmap.scaled(280,280)
        label=QLabel(self)
	label.setObjectName("L_LABEL")
        label.setPixmap(self.pixmap)
	label.setStyleSheet("QLabel#L_LABEL{background-color: white} ")





    def paintEvent(self, event):

        painter = QPainter()
        painter.begin(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.drawPixmap(self.rect(),self.pixmap)

        painter.fillRect(event.rect(), self.palette().brush(QPalette.Window))

        self.drawMarkings(painter)

        self.drawNeedle(painter)

        painter.end()

    def drawMarkings(self, painter):

        painter.save()
        painter.translate(self.width() / 2, self.height() / 2)
        scale = min((self.width() - self._margins) / 120.0,
                    (self.height() - self._margins) / 120.0)
        painter.scale(scale, scale)

	center= QPoint (0,0)
        font = QFont(self.font())
        font.setPixelSize(10)
        metrics = QFontMetricsF(font)

        painter.setFont(font)
        #painter.setPen(self.palette().color(QPalette.Dark))
        painter.setPen(QPen (QColor(191,191,191), 6))
	painter.drawEllipse(center,58,58)
	painter.setPen(QPen (QColor(0,176,240), 2))
	name_rect = QRectF (-85,-54, 45,60)
	painter.drawText(name_rect, Qt.AlignLeft,"REMOTE\nUAV")

        pen=QPen()


        i = 0
        while i < 360:

            if i % 90 == 0:

                font.setPixelSize(10)
		font.setWeight(81)
                metrics = QFontMetricsF(font)
                painter.setFont(font)
                pen.setWidthF(3)
                pen.setColor(QColor(0,176,240))
                painter.setPen(pen)
                #painter.setPen(QColor(255, 0, 0))
                #painter.drawLine(0, -40, 0, -50)
                painter.drawText(-metrics.width(self._pointText[i])/2 , -45,
                                 self._pointText[i])

            elif i%30==0:
                pen.setWidthF(2)
                #painter.setPen(QColor(0, 130, 255))
                pen.setColor(QColor(0, 0,0))
                painter.setPen(pen)
                font.setPixelSize(10)
		font.setWeight(50)
                metrics = QFontMetricsF(font)

                painter.setFont(font)
                #painter.drawLine(0, -45, 0, -50)

                painter.drawText(-metrics.width(self._pointText1[i]) /2.0, -45, self._pointText1[i])
            else:
                painter.setPen(QColor(0, 130, 255))
                #painter.drawLine(0, -45, 0, -50)

            painter.rotate(30)
            i += 30

        painter.restore()
        # print('work2')

    def drawNeedle(self, painter):

        painter.save()
        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(self._angle)
        scale = min((self.width() - self._margins) / 120.0,
                    (self.height() - self._margins) / 120.0)
        painter.scale(scale, scale)
        painter.drawPixmap(self.rect(),self.pixmap)
        painter.setPen(QPen(Qt.NoPen))
        # painter.setBrush(self.palette().brush(QPalette.Shadow))
        # #painter.setBrush(self.palette().brush(QColor(255, 0, 0)))
        #
        # painter.drawPolygon(
        #     QPolygon([QPoint(-5, 0), QPoint(5, 0),
        #               QPoint(0, 45), QPoint(-5, 0)])
        # )

        #painter.setBrush(self.palette().brush(QPalette.Highlight))
        painter.setBrush(QColor(0,176,240))
        painter.drawPolygon(
            QPolygon([QPoint(-5, 0), QPoint(0, -45), QPoint(5, 0),
                       QPoint(-5, 0)])
        )
	painter.setBrush(QColor(230, 240, 240))
        painter.drawPolygon(
            QPolygon([QPoint(-5, 0), QPoint(0, 45), QPoint(5, 0),
                       QPoint(-5, 0)])
        )


        painter.restore()
        # print('self._angle ren count: %d' % sys.getrefcount(self._angle))
        time.sleep(0.001)
        # print('work1')
        # self._angle

    def sizeHint(self):

        return QSize(180,180)

    def angle(self):
        return self._angle

    # @pyqtSlot(float)
    def setAngle(self, angle):

        if angle != self._angle:
            self._angle = angle
            self.angleChanged.emit(angle)
            self.update()
            time.sleep(0.005)

    angle = pyqtProperty(float, angle, setAngle)




