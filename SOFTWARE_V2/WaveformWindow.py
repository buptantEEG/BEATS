# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'WaveformWindow.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1043, 871)
        self.verticalLayout = QtWidgets.QVBoxLayout(Form)
        self.verticalLayout.setContentsMargins(-1, -1, -1, 9)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame_2 = QtWidgets.QFrame(Form)
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.frame_2)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.isDetrendBox = QtWidgets.QCheckBox(self.frame_2)
        self.isDetrendBox.setObjectName("isDetrendBox")
        self.horizontalLayout_2.addWidget(self.isDetrendBox)
        self.isFilterBox = QtWidgets.QCheckBox(self.frame_2)
        self.isFilterBox.setObjectName("isFilterBox")
        self.horizontalLayout_2.addWidget(self.isFilterBox)
        self.verticalLayout.addWidget(self.frame_2)
        self.channelSelectLayout = QtWidgets.QHBoxLayout()
        self.channelSelectLayout.setObjectName("channelSelectLayout")
        self.checkBox = QtWidgets.QCheckBox(Form)
        self.checkBox.setObjectName("checkBox")
        self.channelSelectLayout.addWidget(self.checkBox)
        self.verticalLayout.addLayout(self.channelSelectLayout)
        self.ChannelsLayout = QtWidgets.QVBoxLayout()
        self.ChannelsLayout.setObjectName("ChannelsLayout")
        self.widget = QtWidgets.QWidget(Form)
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.signalView = PlotWidget(self.widget)
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(20)
        self.signalView.setFont(font)
        self.signalView.setLineWidth(1)
        self.signalView.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.signalView.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.NoBrush)
        self.signalView.setBackgroundBrush(brush)
        self.signalView.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        self.signalView.setObjectName("signalView")
        self.horizontalLayout.addWidget(self.signalView)
        self.FFTView = PlotWidget(self.widget)
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(20)
        self.FFTView.setFont(font)
        self.FFTView.setLineWidth(1)
        self.FFTView.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.FFTView.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.NoBrush)
        self.FFTView.setBackgroundBrush(brush)
        self.FFTView.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        self.FFTView.setObjectName("FFTView")
        self.horizontalLayout.addWidget(self.FFTView)
        self.horizontalLayout.setStretch(0, 4)
        self.horizontalLayout.setStretch(1, 1)
        self.ChannelsLayout.addWidget(self.widget)
        self.verticalLayout.addLayout(self.ChannelsLayout)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Waveform"))
        self.isDetrendBox.setText(_translate("Form", "Detrend"))
        self.isFilterBox.setText(_translate("Form", "Filter"))
        self.checkBox.setText(_translate("Form", "Channel1"))
from pyqtgraph import PlotWidget