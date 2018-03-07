import sys
import os
from dvrk_vision.vtk_stereo_viewer import StereoCameras
from dvrk_vision.cv_widget import CvWidget
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QMainWindow, QWidget, QApplication, QGraphicsScene
from orthoWidget import Ui_Orthogonal_Form
import rospy
import ipdb
import cv2
import numpy as np 
import pickle

_NUMCLR=4

class OrthoWindow(QWidget):
    def __init__(self,camInput,masterWidget=None, parent=None):
        super(OrthoWindow,self).__init__()
        functionPath = os.path.dirname(os.path.realpath(__file__))

        uic.loadUi(functionPath + "/ortho_cam_v2.ui", self)

        self.imCombine=[]
        self.maskImageL=[]
        self.maskImageR=[]
        self.Pix=QPixmap()
        self.myIm=QImage()

        self.camera=camInput

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateIm) 
        self.runButton.clicked.connect(self.startTimer)

        self.prevColor=0
        self.curColor=0
        self.threshList=[[0,0,0,255,255,255,0,0,255,255],\
                         [5,5,5,100,100,100,0,0,255,255],\
                         [0,0,0,255,255,255,0,0,255,255],\
                         [0,0,0,255,255,255,0,0,255,255]]
        self.loadColors()

        self.colorChooseBox.valueChanged.connect(self.updateColorList)
        self.saveBtn.clicked.connect(self.saveColors)
        self.loadBtn.clicked.connect(self.loadColors)

        #Set up publisher for pt Segmentation
        ptTopicName="colorSegOrthogonal"

        self.myNewGLWidget=CvWidget()
        self.myNewGLWidget.show()

    def closeEvent(self,closeEvent):
        self.updateColorList(self.colorChooseBox.value())
        #save the color thresholds automatically on exit
        self.saveColors()

    def saveColors(self):
        self.updateColorList(self.colorChooseBox.value())
        with open('colorThresh.pickle','w') as pickFile:
            pickle.dump(self.threshList,pickFile)

    def loadColors(self):
        with open('colorThresh.pickle','rb') as pickFile:
            self.threshList=pickle.load(pickFile)
            curColorBox=self.colorChooseBox.value()
            self.minHBar.setValue(self.threshList[curColorBox][0])
            self.minSBar.setValue(self.threshList[curColorBox][1])
            self.minVBar.setValue(self.threshList[curColorBox][2])
            self.maxHBar.setValue(self.threshList[curColorBox][3])
            self.maxSBar.setValue(self.threshList[curColorBox][4])
            self.maxVBar.setValue(self.threshList[curColorBox][5])

            self.minCrBar.setValue(self.threshList[curColorBox][6])
            self.minCbBar.setValue(self.threshList[curColorBox][7])
            self.maxCrBar.setValue(self.threshList[curColorBox][8])
            self.maxCbBar.setValue(self.threshList[curColorBox][9])

    def updateColorList(self,curColorBox):
        self.prevColor=self.curColor
        self.curColor=curColorBox

        self.threshList[self.prevColor][0]=self.minHBar.value()
        self.threshList[self.prevColor][1]=self.minSBar.value()
        self.threshList[self.prevColor][2]=self.minVBar.value()
        self.threshList[self.prevColor][3]=self.maxHBar.value()
        self.threshList[self.prevColor][4]=self.maxSBar.value()
        self.threshList[self.prevColor][5]=self.maxVBar.value()

        self.threshList[self.prevColor][6]=self.minCrBar.value()
        self.threshList[self.prevColor][7]=self.minCbBar.value()
        self.threshList[self.prevColor][8]=self.maxCrBar.value()
        self.threshList[self.prevColor][9]=self.maxCbBar.value()

        self.minHBar.setValue(self.threshList[curColorBox][0])
        self.minSBar.setValue(self.threshList[curColorBox][1])
        self.minVBar.setValue(self.threshList[curColorBox][2])
        self.maxHBar.setValue(self.threshList[curColorBox][3])
        self.maxSBar.setValue(self.threshList[curColorBox][4])
        self.maxVBar.setValue(self.threshList[curColorBox][5])
    
        self.minCrBar.setValue(self.threshList[curColorBox][6])
        self.minCbBar.setValue(self.threshList[curColorBox][7])
        self.maxCrBar.setValue(self.threshList[curColorBox][8])
        self.maxCbBar.setValue(self.threshList[curColorBox][9])
        
    def startTimer(self):
        self.timer.start(50) #50ms=20 Hz

    def updateIm(self):
        ptLList,ptRList,imCombine=self.calculate3DPoint(self.camera.camL.image,self.camera.camR.image,self.maskCheck.isChecked())

##############################
        # TODO ADD HERE: publish ptLList, ptRList
        # self.ptPublisher.publish()
##############################
        if type(imCombine) != type(None):
            if not self.maskCheck.isChecked():
                imCombine = cv2.cvtColor(imCombine, cv2.COLOR_BGR2RGB)
            else:
                imCombine = cv2.cvtColor(imCombine,cv2.COLOR_GRAY2RGB)
            self.myNewGLWidget.setImage(imCombine)

    def calculate3DPoint(self,imageL, imageR,bMasked):
        point3d = None

        imExist=0;  
        # Process left image if it exists
        (rows,cols,channels) = imageL.shape
        if cols > 60 and rows > 60 :
            imExist=1
        (rows,cols,channels) = imageR.shape
        if cols > 60 and rows > 60 :
            imRExist=imExist and 1

        if not imExist:
            return None,None,None,None

        centerL=[None]*_NUMCLR
        radiusL=[None]*_NUMCLR
        centerR=[None]*_NUMCLR
        radiusR=[None]*_NUMCLR
        for colorIndex in range(_NUMCLR):
            self.maskImageL = self.mask(imageL,colorIndex)
            centerL[colorIndex], radiusL[colorIndex] = self.getCentroid(self.maskImageL)
        
            self.maskImageR = self.mask(imageR,colorIndex)
            centerR[colorIndex], radiusR[colorIndex] = self.getCentroid(self.maskImageR)

            if(centerL[colorIndex] != None and centerR[colorIndex] != None):
                cv2.circle(imageL, centerL[colorIndex], 2,(0, 255, 0), -1)
                cv2.circle(imageR, centerR[colorIndex], 2,(0, 255, 0), -1)
                cv2.circle(imageL, centerL[colorIndex], radiusL[colorIndex],(0, 255, 0), 1)
                cv2.circle(imageR, centerR[colorIndex], radiusR[colorIndex],(0, 255, 0), 1)

            if colorIndex==self.colorChooseBox.value() and bMasked:
                imCombine=self.combineImages(self.maskImageL, self.maskImageR)
        if not bMasked:
            imCombine=self.combineImages(imageL,imageR)

        return centerL, centerR,imCombine

    def mask(self,img,curColor):
        
        # Convert to HSV and mask colors
        if curColor==self.colorChooseBox.value():
            colorLower = (self.minHBar.value(),self.minSBar.value(), self.minVBar.value())
            colorUpper = (self.maxHBar.value(),self.maxSBar.value(), self.maxVBar.value())
            colorLower2=(0,self.minCrBar.value(),self.minCbBar.value())
            colorUpper2=(255,self.maxCrBar.value(),self.maxCbBar.value())

        else:
            colorLower = (self.threshList[curColor][0],self.threshList[curColor][1],self.threshList[curColor][2])
            colorUpper = (self.threshList[curColor][3],self.threshList[curColor][4],self.threshList[curColor][5])
            colorLower2= (0,    self.threshList[curColor][6],  self.threshList[curColor][7])
            colorUpper2= (255,  self.threshList[curColor][8],  self.threshList[curColor][9])
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, colorLower, colorUpper )

        yCrCb=cv2.cvtColor(blurred,cv2.COLOR_BGR2YCrCb)
        
        mask2=cv2.inRange(yCrCb,colorLower2,colorUpper2)

        mask=mask&mask2

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

if __name__ == "__main__":
    #Set up stereo camera struct
    frameRate = 15
    slop = 1.0 / frameRate
    cams = StereoCameras("/stereo/left/image_rect",
                         "/stereo/right/image_rect",
                         "/stereo/left/camera_info",
                         "/stereo/right/camera_info",
                          slop = slop)

    rospy.init_node("test_stereo_cams")
    # rate=rospy.Rate(24)
    
    #Set up qt GUI widget
    app = QtWidgets.QApplication(sys.argv)
    myWindow=OrthoWindow(cams)
    myWindow.show()

    sys.exit(app.exec_())
    