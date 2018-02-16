import sys
import os
from dvrk_vision.vtk_stereo_viewer import StereoCameras
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QMainWindow, QWidget, QApplication, QGraphicsScene
from orthoWidget import Ui_Form
import rospy
import ipdb
import cv2
import numpy as np 

# def nothingCB(numIn):
#     #don't do anything
#     return 0

#Set up stereo camera struct
frameRate = 15
slop = 1.0 / frameRate
cams = StereoCameras("/stereo/left/image_rect",
                     "/stereo/right/image_rect",
                     "/stereo/left/camera_info",
                     "/stereo/right/camera_info",
                      slop = slop)
def mask(img):
    # Convert to HSV and mask colors
    # hMin = cv2.getTrackbarPos('min H',_WINDOW_NAME)
    # hMax = cv2.getTrackbarPos('max H',_WINDOW_NAME)
    # sMin = cv2.getTrackbarPos('min S',_WINDOW_NAME)
    # sMax = cv2.getTrackbarPos('max S',_WINDOW_NAME)
    # vMin = cv2.getTrackbarPos('min V',_WINDOW_NAME)
    # vMax = cv2.getTrackbarPos('max V',_WINDOW_NAME)
    colorLower = (0, 0, 0)
    colorUpper = (0, 0, 0)
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, colorLower, colorUpper )
    # Refine mask
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask

def getCentroid(maskImage):
    # With help from http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(maskImage.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # if cv2.getTrackbarPos('masked',_WINDOW_NAME) == 0:
        # ipdb.set_trace()


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

def combineImages(imageL, imageR):
    (rows,cols) = imageL.shape[0:2]
    if len(imageL.shape) == 2:
        shape = (rows, cols*2)
    elif len(imageL.shape) == 3:
        shape = (rows, cols*2, imageL.shape[2])
    doubleImage = np.zeros(shape,np.uint8)
    doubleImage[0:rows,0:cols] = imageL
    doubleImage[0:rows,cols:cols*2] = imageR
    return doubleImage

def calculate3DPoint(imageL, imageR):
    point3d = None
    # Process left image if it exists
    (rows,cols,channels) = imageL.shape
    if cols > 60 and rows > 60 :
        maskImageL = mask(imageL)
        centerL, radiusL = getCentroid(maskImageL)

    # if it doesn't exist, don't do anything
    else:
        return None, None, None

    (rows,cols,channels) = imageR.shape
    if cols > 60 and rows > 60 :
        maskImageR = mask(imageR)
        centerR, radiusR = getCentroid(maskImageR)
    else:
        return None, None, combineImages(imageL, imageR)

    if(centerL != None and centerR != None):
        # disparity = abs(centerL[0] - centerR[0])
        cv2.circle(imageL, centerL, 2,(0, 255, 0), -1)
        cv2.circle(imageR, centerR, 2,(0, 255, 0), -1)
        cv2.circle(imageL, centerL, radiusL,(0, 255, 0), 1)
        cv2.circle(imageR, centerR, radiusR,(0, 255, 0), 1)

    # if cv2.getTrackbarPos('masked',_WINDOW_NAME) == 0:
    return centerL, centerR, imageL,imageR
    # else:
    #     return centerL, centerR, combineImages(maskImageL, maskImageR)

def UpdateIm():
    if type(cams.camL.image) != type(None) and type(cams.camR.image) != type(None):
        cvRGBImg = cv2.cvtColor(cams.camL.image, cv2.COLOR_BGR2RGB)
        myIm=QImage(cvRGBImg.data,cvRGBImg.shape[1],cvRGBImg.shape[0],QtGui.QImage.Format_RGB888)
        pix=QPixmap(myIm)
        leftScn.addPixmap(pix)
        cvRGBImg = cv2.cvtColor(cams.camR.image, cv2.COLOR_BGR2RGB)
        myIm=QImage(cvRGBImg.data,cvRGBImg.shape[1],cvRGBImg.shape[0],QtGui.QImage.Format_RGB888)
        pix=QPixmap(myIm)
        rightScn.addPixmap(pix)


class OrthoWindow(QtWidgets.QMainWindow):
    def __init__(self,masterWidget=None, parent=None):
        super(OrthoWindow,self).__init__()
        functionPath = os.path.dirname(os.path.realpath(__file__))
        uic.loadUi(functionPath + "/ortho_cam_v1.ui", self)
        



if __name__ == "__main__":
    #Set up qt GUI widget
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()

    ui.runButton.clicked.connect(UpdateIm)

    rospy.init_node("stereo_cams_test")
    rate = rospy.Rate(24) # 24hz

    rightScn = QGraphicsScene()
    leftScn = QGraphicsScene()
    ui.rightView.setScene(rightScn)
    ui.leftView.setScene(leftScn)


    # while not rospy.is_shutdown():
    #     if type(cams.camL.image) != type(None) and type(cams.camR.image) != type(None):
    #         # ipdb.set_trace()
    #         centerL,centerR,combImb=calculate3DPoint(cams.camL.image,cams.camR.image)
    #         if type(combImb)!=type(None):
    #             myIm=QImage(combImb.data,combImb.shape[0],combImb.shape[1],QtGui.QImage.Format_RGB32)
    #             pix=QPixmap(myIm)
    #             scn.addPixmap(pix)

            # rate.sleep()
    sys.exit(app.exec_())

    # app = QApplication(sys.argv)
    # window = OrthoWindow(None)

    # window.show()

    # app.exec_()
    # app.deleteLater()
    # sys.exit()
    # sys.exit(app.exec_())

    

    # # Set up GUI
    # cv2.namedWindow(_WINDOW_NAME)
    # cv2.createTrackbar('min H', _WINDOW_NAME, 0, 180, nothingCB)
    # cv2.createTrackbar('max H', _WINDOW_NAME, 0, 180, nothingCB)
    # cv2.createTrackbar('min S', _WINDOW_NAME, 0, 255, nothingCB)
    # cv2.createTrackbar('max S', _WINDOW_NAME, 0, 255, nothingCB)
    # cv2.createTrackbar('min V', _WINDOW_NAME, 0, 255, nothingCB)
    # cv2.createTrackbar('max V', _WINDOW_NAME, 0, 255, nothingCB)
    # cv2.createTrackbar('masked', _WINDOW_NAME, 0, 1, nothingCB)

    