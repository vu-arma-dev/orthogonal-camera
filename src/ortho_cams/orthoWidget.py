# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'testWid.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Orthogonal_Form(object):
    def setupUi(self, Orthogonal_Form):
        Orthogonal_Form.setObjectName("Orthogonal_Form")
        Orthogonal_Form.resize(649, 540)
        self.gridLayout_2 = QtWidgets.QGridLayout(Orthogonal_Form)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.label_3 = QtWidgets.QLabel(Orthogonal_Form)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 2, 0, 1, 1)
        self.maxSBar = QtWidgets.QScrollBar(Orthogonal_Form)
        self.maxSBar.setMinimumSize(QtCore.QSize(250, 0))
        self.maxSBar.setMaximum(255)
        self.maxSBar.setOrientation(QtCore.Qt.Horizontal)
        self.maxSBar.setObjectName("maxSBar")
        self.gridLayout.addWidget(self.maxSBar, 1, 2, 1, 1)
        self.maxHBar = QtWidgets.QScrollBar(Orthogonal_Form)
        self.maxHBar.setMaximum(255)
        self.maxHBar.setOrientation(QtCore.Qt.Horizontal)
        self.maxHBar.setObjectName("maxHBar")
        self.gridLayout.addWidget(self.maxHBar, 0, 2, 1, 1)
        self.minHBar = QtWidgets.QScrollBar(Orthogonal_Form)
        self.minHBar.setMaximum(255)
        self.minHBar.setOrientation(QtCore.Qt.Horizontal)
        self.minHBar.setObjectName("minHBar")
        self.gridLayout.addWidget(self.minHBar, 0, 1, 1, 1)
        self.maxVBar = QtWidgets.QScrollBar(Orthogonal_Form)
        self.maxVBar.setMaximum(255)
        self.maxVBar.setOrientation(QtCore.Qt.Horizontal)
        self.maxVBar.setObjectName("maxVBar")
        self.gridLayout.addWidget(self.maxVBar, 2, 2, 1, 1)
        self.label_2 = QtWidgets.QLabel(Orthogonal_Form)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.label = QtWidgets.QLabel(Orthogonal_Form)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(Orthogonal_Form)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 1, 3, 1, 1)
        self.label_4 = QtWidgets.QLabel(Orthogonal_Form)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 0, 3, 1, 1)
        self.label_5 = QtWidgets.QLabel(Orthogonal_Form)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 2, 3, 1, 1)
        self.minSBar = QtWidgets.QScrollBar(Orthogonal_Form)
        self.minSBar.setMinimumSize(QtCore.QSize(250, 0))
        self.minSBar.setMaximum(255)
        self.minSBar.setOrientation(QtCore.Qt.Horizontal)
        self.minSBar.setObjectName("minSBar")
        self.gridLayout.addWidget(self.minSBar, 1, 1, 1, 1)
        self.minVBar = QtWidgets.QScrollBar(Orthogonal_Form)
        self.minVBar.setMaximum(255)
        self.minVBar.setOrientation(QtCore.Qt.Horizontal)
        self.minVBar.setObjectName("minVBar")
        self.gridLayout.addWidget(self.minVBar, 2, 1, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 3, 0, 1, 1)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.leftView = QtWidgets.QGraphicsView(Orthogonal_Form)
        self.leftView.setObjectName("leftView")
        self.horizontalLayout.addWidget(self.leftView)
        self.rightView = QtWidgets.QGraphicsView(Orthogonal_Form)
        self.rightView.setObjectName("rightView")
        self.horizontalLayout.addWidget(self.rightView)
        self.gridLayout_2.addLayout(self.horizontalLayout, 1, 0, 1, 1)
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.maskCheck = QtWidgets.QCheckBox(Orthogonal_Form)
        self.maskCheck.setObjectName("maskCheck")
        self.gridLayout_3.addWidget(self.maskCheck, 0, 4, 1, 1)
        self.label_7 = QtWidgets.QLabel(Orthogonal_Form)
        self.label_7.setObjectName("label_7")
        self.gridLayout_3.addWidget(self.label_7, 0, 2, 1, 1)
        self.spinBox = QtWidgets.QSpinBox(Orthogonal_Form)
        self.spinBox.setObjectName("spinBox")
        self.gridLayout_3.addWidget(self.spinBox, 0, 0, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(918, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem, 0, 3, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout_3, 4, 0, 1, 1)
        self.runButton = QtWidgets.QPushButton(Orthogonal_Form)
        self.runButton.setObjectName("runButton")
        self.gridLayout_2.addWidget(self.runButton, 2, 0, 1, 1)
        self.label_7.raise_()
        self.runButton.raise_()

        self.retranslateUi(Orthogonal_Form)
        QtCore.QMetaObject.connectSlotsByName(Orthogonal_Form)

    def retranslateUi(self, Orthogonal_Form):
        _translate = QtCore.QCoreApplication.translate
        Orthogonal_Form.setWindowTitle(_translate("Orthogonal_Form", "Form"))
        self.label_3.setText(_translate("Orthogonal_Form", "Min V"))
        self.label_2.setText(_translate("Orthogonal_Form", "Min S"))
        self.label.setText(_translate("Orthogonal_Form", "Min H"))
        self.label_6.setText(_translate("Orthogonal_Form", "Max S"))
        self.label_4.setText(_translate("Orthogonal_Form", "Max H"))
        self.label_5.setText(_translate("Orthogonal_Form", "Max V"))
        self.maskCheck.setText(_translate("Orthogonal_Form", "Mask"))
        self.label_7.setText(_translate("Orthogonal_Form", "ColorNumber"))
        self.runButton.setText(_translate("Orthogonal_Form", "Image"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Orthogonal_Form = QtWidgets.QWidget()
    ui = Ui_Orthogonal_Form()
    ui.setupUi(Orthogonal_Form)
    Orthogonal_Form.show()
    sys.exit(app.exec_())
