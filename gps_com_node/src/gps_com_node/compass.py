import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *


class CompassWidget(QWidget):
    angleChanged = pyqtSignal(float)
    angleChanged1 = pyqtSignal(float)
    def __init__(self, parent=None):

        QWidget.__init__(self, parent)

        self._angle = 0.0
        self._margins = 10
        self._pointText = {0: "N0" ,45: "NE", 90: "E90", 135: "SE", 180: "S180",
                           225: "SW", 270: "W270", 315: "NW"}

        self._pointText1={0: '0', 30:'30',60:'60',90:'90',120:'120',150:'150',180:'180',210:'210',240:'240',270:'270',300:'300',330:'330'}

    def paintEvent(self, event):

        painter = QPainter()
        painter.begin(self)
        painter.setRenderHint(QPainter.Antialiasing)

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

        font = QFont(self.font())
        font.setPixelSize(10)
        metrics = QFontMetricsF(font)

        painter.setFont(font)
        #painter.setPen(self.palette().color(QPalette.Dark))
        painter.setPen(QColor(0,130,255))

        pen=QPen()


        i = 0
        while i < 360:

            if i % 45 == 0:

                font.setPixelSize(7)
                metrics = QFontMetricsF(font)
                painter.setFont(font)
                pen.setWidthF(1.8)
                pen.setColor(QColor(0,80,255))
                painter.setPen(pen)
                #painter.setPen(QColor(255, 0, 0))
                painter.drawLine(0, -40, 0, -50)
                painter.drawText(-metrics.width(self._pointText[i]) / 2.0, -55,
                                 self._pointText[i])

            elif i%30==0:
                pen.setWidthF(1.8)
                #painter.setPen(QColor(0, 130, 255))
                pen.setColor(QColor(0, 130, 255))
                painter.setPen(pen)
                font.setPixelSize(7)
                metrics = QFontMetricsF(font)

                painter.setFont(font)
                painter.drawLine(0, -45, 0, -50)

                painter.drawText(-metrics.width(self._pointText1[i]) /2.0, -55, self._pointText1[i])
            else:
                painter.setPen(QColor(0, 130, 255))
                painter.drawLine(0, -45, 0, -50)

            painter.rotate(3)
            i += 3

        painter.restore()

    def drawNeedle(self, painter):

        painter.save()
        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(self._angle)
        scale = min((self.width() - self._margins) / 120.0,
                    (self.height() - self._margins) / 120.0)
        painter.scale(scale, scale)

        painter.setPen(QPen(Qt.NoPen))
        painter.setBrush(self.palette().brush(QPalette.Shadow))
        #painter.setBrush(self.palette().brush(QColor(255, 0, 0)))

        painter.drawPolygon(
            QPolygon([QPoint(-5, 0), QPoint(5, 0),
                      QPoint(0, 45), QPoint(-5, 0)])
        )

        #painter.setBrush(self.palette().brush(QPalette.Highlight))
        painter.setBrush(QColor(255, 0, 0))
        painter.drawPolygon(
            QPolygon([QPoint(-5, 0), QPoint(0, -45), QPoint(5, 0),
                       QPoint(-5, 0)])
        )

        painter.restore()

    def sizeHint(self):

        return QSize(150, 150)

    def angle(self):
        return self._angle

    #@pyqtSlot(float)
    def setAngle(self, angle):

        if angle != self._angle:
            self._angle = angle
            self.angleChanged.emit(angle)
            self.update()

    angle = pyqtProperty(float, angle, setAngle)



