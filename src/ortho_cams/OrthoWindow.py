import sys
# from PyQt5.QtWidgets import QMainWindow
# from PyQt5 import uic
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QApplication


class OrthoWindow(QtWidgets.QMainWindow):
    def __init__(self,masterWidget=None, parent=None):
        super(OrthoWindow,self).__init__()
        functionPath = os.path.dirname(os.path.realpath(__file__))
        uic.loadUi(functionPath + "/ortho_cam_v1.ui", self)
        