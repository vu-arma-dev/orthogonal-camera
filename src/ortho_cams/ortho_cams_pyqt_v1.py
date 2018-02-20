import sys
import os
from dvrk_vision.vtk_stereo_viewer import StereoCameras
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QMainWindow, QWidget, QApplication, QGraphicsScene
from orthoWidget import Ui_Orthogonal_Form
import rospy
import ipdb
import cv2
import numpy as np 
import pickle

class OrthoWindow(QWidget):
    def __init__(self,camInput,masterWidget=None, parent=None):
        super(OrthoWindow,self).__init__()
        self.ui=Ui_Orthogonal_Form()
        self.ui.setupUi(self)

        self.imR=[]
        self.imL=[]
        self.Pix=QPixmap()
        self.myIm=QImage()

        self.rightScn = QGraphicsScene()
        self.leftScn = QGraphicsScene()
        self.ui.rightView.setScene(self.rightScn)
        self.ui.leftView.setScene(self.leftScn)
        self.camera=camInput

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateIm) 
        self.ui.runButton.clicked.connect(self.startTimer)

        self.prevColor=0
        self.curColor=0
        self.threshList=[[0,0,0,255,255,255],\
                         [5,5,5,100,100,100],\
                         [0,0,0,255,255,255],\
                         [0,0,0,255,255,255]]
        self.loadColors()

        self.ui.colorChooseBox.valueChanged.connect(self.updateColorList)
        self.ui.saveBtn.clicked.connect(self.saveColors)
        self.ui.loadBtn.clicked.connect(self.loadColors)

    def closeEvent(self,closeEvent):
        self.updateColorList(self.ui.colorChooseBox.value())
        #save the color thresholds automatically on exit
        self.saveColors()

    def saveColors(self):
        self.updateColorList(self.ui.colorChooseBox.value())
        with open('colorThresh.pickle','w') as pickFile:
            pickle.dump(self.threshList,pickFile)

    def loadColors(self):
        with open('colorThresh.pickle','rb') as pickFile:
            # ipdb.set_trace()
            self.threshList=pickle.load(pickFile)
            curColorBox=self.ui.colorChooseBox.value()
            self.ui.minHBar.setValue(self.threshList[curColorBox][0])
            self.ui.minSBar.setValue(self.threshList[curColorBox][1])
            self.ui.minVBar.setValue(self.threshList[curColorBox][2])
            self.ui.maxHBar.setValue(self.threshList[curColorBox][3])
            self.ui.maxSBar.setValue(self.threshList[curColorBox][4])
            self.ui.maxVBar.setValue(self.threshList[curColorBox][5])

    def updateColorList(self,curColorBox):
        self.prevColor=self.curColor
        self.curColor=curColorBox

    # def updateHSV(self,colorNum):
        self.threshList[self.prevColor][0]=self.ui.minHBar.value()
        self.threshList[self.prevColor][1]=self.ui.minSBar.value()
        self.threshList[self.prevColor][2]=self.ui.minVBar.value()
        self.threshList[self.prevColor][3]=self.ui.maxHBar.value()
        self.threshList[self.prevColor][4]=self.ui.maxSBar.value()
        self.threshList[self.prevColor][5]=self.ui.maxVBar.value()
        # ipdb.set_trace()
        self.ui.minHBar.setValue(self.threshList[curColorBox][0])
        self.ui.minSBar.setValue(self.threshList[curColorBox][1])
        self.ui.minVBar.setValue(self.threshList[curColorBox][2])
        self.ui.maxHBar.setValue(self.threshList[curColorBox][3])
        self.ui.maxSBar.setValue(self.threshList[curColorBox][4])
        self.ui.maxVBar.setValue(self.threshList[curColorBox][5])

    def startTimer(self):
        self.timer.start(100) #100ms=10 Hz

    def updateIm(self):
        ptx,pty,self.imL=self.calculate3DPoint(self.camera.camL.image,self.camera.camR.image,self.ui.maskCheck.isChecked())

        if type(self.imL) != type(None) and type(self.imR) != type(None):
            if not self.ui.maskCheck.isChecked():
                self.imL = cv2.cvtColor(self.imL, cv2.COLOR_BGR2RGB)
                self.myIm=QImage(self.imL,self.imL.shape[1],self.imL.shape[0],QtGui.QImage.Format_RGB888)
            else:
                self.myIm=QImage(self.imL,self.imL.shape[1],self.imL.shape[0],QtGui.QImage.Format_Grayscale8)
            self.pix=QPixmap(self.myIm)
            self.leftScn.addPixmap(self.pix)

    def mask(self,img):
        # Convert to HSV and mask colors
        colorLower = (self.ui.minHBar.value(),self.ui.minSBar.value(), self.ui.minVBar.value())
        colorUpper = (self.ui.maxHBar.value(),self.ui.maxSBar.value(), self.ui.maxVBar.value())
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, colorLower, colorUpper )

        # Refine mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

    def getCentroid(self,maskImage):
        # With help from http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(maskImage.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 3:
                return center, int(radius)
        # Otherwise return nonsense
        return None, None

    def combineImages(self,imageL, imageR):
        (rows,cols) = imageL.shape[0:2]
        if len(imageL.shape) == 2:
            shape = (rows, cols*2)
        elif len(imageL.shape) == 3:
            shape = (rows, cols*2, imageL.shape[2])
        doubleImage = np.zeros(shape,np.uint8)
        doubleImage[0:rows,0:cols] = imageL
        doubleImage[0:rows,cols:cols*2] = imageR
        return doubleImage

    def calculate3DPoint(self,imageL, imageR,bMasked):
        point3d = None
        # Process left image if it exists
        (rows,cols,channels) = imageL.shape
        if cols > 60 and rows > 60 :
            maskImageL = self.mask(imageL)
            centerL, radiusL = self.getCentroid(maskImageL)

        # if it doesn't exist, don't do anything
        else:
            return None, None, None, None

        (rows,cols,channels) = imageR.shape
        if cols > 60 and rows > 60 :
            maskImageR = self.mask(imageR)
            centerR, radiusR = self.getCentroid(maskImageR)
        else:
            return None, None,None, None

        if(centerL != None and centerR != None):
            # disparity = abs(centerL[0] - centerR[0])
            cv2.circle(imageL, centerL, 2,(0, 255, 0), -1)
            cv2.circle(imageR, centerR, 2,(0, 255, 0), -1)
            cv2.circle(imageL, centerL, radiusL,(0, 255, 0), 1)
            cv2.circle(imageR, centerR, radiusR,(0, 255, 0), 1)


        if bMasked:
            return centerL, centerR, self.combineImages(maskImageL, maskImageR)
        else:
            return centerL, centerR, self.combineImages(imageL,imageR)

if __name__ == "__main__":
    #Set up stereo camera struct
    frameRate = 15
    slop = 1.0 / frameRate
    cams = StereoCameras("/stereo/left/image_rect",
                         "/stereo/right/image_rect",
                         "/stereo/left/camera_info",
                         "/stereo/right/camera_info",
                          slop = slop)
    
    #Set up rosnode
    rospy.init_node("stereo_cams_test")
    rate = rospy.Rate(24) # 24hz

    #Set up qt GUI widget
    app = QtWidgets.QApplication(sys.argv)
    myWindow=OrthoWindow(cams)
    myWindow.show()

    sys.exit(app.exec_())
    