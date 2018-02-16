import sys
import os
import rospy
from PyQt5 import QtCore, QtGui, QtWidgets, uic
# from PyQt5.QtWidgets import QMainWindow, QWidget, QApplication, QGraphicsScene,QGraphicsView
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsItem, QGridLayout, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QMainWindow, QWidget
import ipdb

class OrthoWindow(QtWidgets.QMainWindow):
    def __init__(self,masterWidget=None, parent=None):
        super(OrthoWindow,self).__init__()
        functionPath = os.path.dirname(os.path.realpath(__file__))
        uic.loadUi(functionPath + "/ortho_cam_v1.ui", self)
        
class TestWindow(QWidget):
    def __init__(self,masterWidget=None, parent=None):
        super(TestWindow,self).__init__()
        functionPath = os.path.dirname(os.path.realpath(__file__))
        uic.loadUi(functionPath + "/testWid.ui", self)
        scene=QGraphicsScene(self)
        # self.ui.myui_graphicsView.setScene(scene)


if __name__ == "__main__":
    # Init ROS node to read images
    rospy.init_node("stereo_cams_test")
    rate = rospy.Rate(24) # 24hz


    app = QApplication(sys.argv)
    # window = OrthoWindow()
    window=TestWindow()
    window.show()

    sys.exit(app.exec_())