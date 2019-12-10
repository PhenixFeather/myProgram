# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'F:\资料\编程\python\untitled.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

import serial
import serial.tools.list_ports
from math import sin, cos, tan, pi
from PyQt5 import QtCore, QtGui, QtWidgets
from scipy.optimize import fsolve
import time
from aip import AipBodyAnalysis
import cv2
import numpy as np


class Ui_Form(object):
    def setupUi(self, Form):
        self.ser=serial.Serial()
        Form.setObjectName("Form")
        Form.resize(772, 486)
        self.Box_Setting = QtWidgets.QGroupBox(Form)
        self.Box_Setting.setGeometry(QtCore.QRect(20, 10, 391, 91))
        self.Box_Setting.setObjectName("Box_Setting")
        #串口选择下拉列表
        self.cBox_Choose = QtWidgets.QComboBox(self.Box_Setting)
        self.cBox_Choose.setGeometry(QtCore.QRect(100, 20, 69, 22))
        self.cBox_Choose.setObjectName("cBox_Choose")
        self.cBox_Choose.addItem("")
        self.cBox_Choose.addItem("")
        #打开串口按钮
        self.bt_SerialOn = QtWidgets.QPushButton(self.Box_Setting)
        self.bt_SerialOn.setGeometry(QtCore.QRect(300, 20, 75, 23))
        self.bt_SerialOn.setObjectName("bt_SerialOn")
        self.bt_SerialOn.clicked.connect(self.port_open)
        #关闭串口按钮
        self.bt_SerialOff = QtWidgets.QPushButton(self.Box_Setting)
        self.bt_SerialOff.setGeometry(QtCore.QRect(300, 60, 75, 23))
        self.bt_SerialOff.setObjectName("bt_SerialOff")
        self.bt_SerialOff.clicked.connect(self.port_close)
        #文本
        self.label = QtWidgets.QLabel(self.Box_Setting)
        self.label.setGeometry(QtCore.QRect(30, 22, 61, 20))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.Box_Setting)
        self.label_2.setGeometry(QtCore.QRect(40, 60, 54, 12))
        self.label_2.setObjectName("label_2")
        #波特率选择下拉列表
        self.cBox_Bote = QtWidgets.QComboBox(self.Box_Setting)
        self.cBox_Bote.setGeometry(QtCore.QRect(100, 60, 69, 22))
        self.cBox_Bote.setObjectName("cBox_Bote")
        self.cBox_Bote.addItem("")
        self.cBox_Bote.addItem("")
        self.cBox_Bote.addItem("")
        self.cBox_Bote.addItem("")
        #串口检测按钮
        self.bt_SerialCheck = QtWidgets.QPushButton(self.Box_Setting)
        self.bt_SerialCheck.setGeometry(QtCore.QRect(200, 20, 75, 23))
        self.bt_SerialCheck.setMouseTracking(False)
        self.bt_SerialCheck.setObjectName("bt_SerialCheck")
        self.bt_SerialCheck.clicked.connect(self.port_check)
        #框
        self.Box_Command = QtWidgets.QGroupBox(Form)
        self.Box_Command.setGeometry(QtCore.QRect(20, 140, 391, 331))
        self.Box_Command.setObjectName("Box_Command")
        #舵机1滑动条及计数器
        self.Slider_1 = QtWidgets.QSlider(self.Box_Command)
        self.Slider_1.setGeometry(QtCore.QRect(100, 40, 160, 22))
        self.Slider_1.setMinimum(500)
        self.Slider_1.setMaximum(2500)
        self.Slider_1.setProperty("value", 1500)
        self.Slider_1.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_1.setObjectName("Slider_1")
        self.Slider_1.setSingleStep(1)
        self.spinBox_1 = QtWidgets.QSpinBox(self.Box_Command)
        self.spinBox_1.setGeometry(QtCore.QRect(280, 40, 51, 22))
        self.spinBox_1.setMinimum(500)
        self.spinBox_1.setMaximum(2500)
        self.spinBox_1.setProperty("value", 1500)
        self.spinBox_1.setObjectName("spinBox_1")
        self.Slider_1.valueChanged.connect(self.slider1_pwm)
        self.spinBox_1.valueChanged.connect(self.port_send1)
        self.spinBox_1.valueChanged.connect(self.coord_update)
        #舵机2滑动条及计数器
        self.Slider_2 = QtWidgets.QSlider(self.Box_Command)
        self.Slider_2.setGeometry(QtCore.QRect(100, 80, 160, 22))
        self.Slider_2.setMinimum(500)
        self.Slider_2.setMaximum(2500)
        self.Slider_2.setProperty("value", 1250)
        self.Slider_2.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_2.setObjectName("Slider_2")
        self.Slider_2.setSingleStep(1)
        self.spinBox_2 = QtWidgets.QSpinBox(self.Box_Command)
        self.spinBox_2.setGeometry(QtCore.QRect(280, 80, 51, 22))
        self.spinBox_2.setMinimum(500)
        self.spinBox_2.setMaximum(2500)
        self.spinBox_2.setProperty("value", 1250)
        self.spinBox_2.setObjectName("spinBox_2")
        self.Slider_2.valueChanged.connect(self.slider2_pwm)
        self.spinBox_2.valueChanged.connect(self.port_send2)
        self.spinBox_2.valueChanged.connect(self.coord_update)
        #舵机3滑动条及计数器
        self.Slider_3 = QtWidgets.QSlider(self.Box_Command)
        self.Slider_3.setGeometry(QtCore.QRect(100, 120, 160, 22))
        self.Slider_3.setMinimum(500)
        self.Slider_3.setMaximum(2500)
        self.Slider_3.setProperty("value", 1500)
        self.Slider_3.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_3.setObjectName("Slider_3")
        self.Slider_3.setSingleStep(1)
        self.spinBox_3 = QtWidgets.QSpinBox(self.Box_Command)
        self.spinBox_3.setGeometry(QtCore.QRect(280, 120, 51, 22))
        self.spinBox_3.setMinimum(500)
        self.spinBox_3.setMaximum(2500)
        self.spinBox_3.setProperty("value", 1500)
        self.spinBox_3.setObjectName("spinBox_3")
        self.Slider_3.valueChanged.connect(self.slider3_pwm)
        self.spinBox_3.valueChanged.connect(self.port_send3)
        self.spinBox_3.valueChanged.connect(self.coord_update)
        #舵机4滑动条及计数器
        self.Slider_4 = QtWidgets.QSlider(self.Box_Command)
        self.Slider_4.setGeometry(QtCore.QRect(100, 160, 160, 22))
        self.Slider_4.setMinimum(500)
        self.Slider_4.setMaximum(2500)
        self.Slider_4.setProperty("value", 1500)
        self.Slider_4.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_4.setObjectName("Slider_4")
        self.Slider_4.setSingleStep(1)
        self.spinBox_4 = QtWidgets.QSpinBox(self.Box_Command)
        self.spinBox_4.setGeometry(QtCore.QRect(280, 160, 51, 22))
        self.spinBox_4.setMinimum(500)
        self.spinBox_4.setMaximum(2500)
        self.spinBox_4.setProperty("value", 1500)
        self.spinBox_4.setObjectName("spinBox_4")
        self.Slider_4.valueChanged.connect(self.slider4_pwm)
        self.spinBox_4.valueChanged.connect(self.port_send4)
        self.spinBox_4.valueChanged.connect(self.coord_update)
        #舵机5滑动条及计数器
        self.Slider_5 = QtWidgets.QSlider(self.Box_Command)
        self.Slider_5.setGeometry(QtCore.QRect(100, 200, 160, 22))
        self.Slider_5.setMinimum(500)
        self.Slider_5.setMaximum(2500)
        self.Slider_5.setProperty("value", 1500)
        self.Slider_5.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_5.setObjectName("Slider_5")
        self.Slider_5.setSingleStep(1)
        self.spinBox_5 = QtWidgets.QSpinBox(self.Box_Command)
        self.spinBox_5.setGeometry(QtCore.QRect(280, 200, 51, 22))
        self.spinBox_5.setMinimum(500)
        self.spinBox_5.setMaximum(2500)
        self.spinBox_5.setProperty("value", 1500)
        self.spinBox_5.setObjectName("spinBox_5")
        self.Slider_5.valueChanged.connect(self.slider5_pwm)
        self.spinBox_5.valueChanged.connect(self.port_send5)
        #舵机6滑动条及计数器
        self.Slider_6 = QtWidgets.QSlider(self.Box_Command)
        self.Slider_6.setGeometry(QtCore.QRect(100, 240, 160, 22))
        self.Slider_6.setMinimum(500)
        self.Slider_6.setMaximum(2500)
        self.Slider_6.setProperty("value", 1500)
        self.Slider_6.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_6.setObjectName("Slider_6")
        self.Slider_6.setSingleStep(1)
        self.spinBox_6 = QtWidgets.QSpinBox(self.Box_Command)
        self.spinBox_6.setGeometry(QtCore.QRect(280, 240, 51, 22))
        self.spinBox_6.setMinimum(500)
        self.spinBox_6.setMaximum(2500)
        self.spinBox_6.setProperty("value", 1500)
        self.spinBox_6.setObjectName("spinBox_6")
        self.Slider_6.valueChanged.connect(self.slider6_pwm)
        self.Slider_6.sliderReleased.connect(self.port_send6)
        self.spinBox_6.valueChanged.connect(self.port_send6)
        #文本
        self.label_5 = QtWidgets.QLabel(self.Box_Command)
        self.label_5.setGeometry(QtCore.QRect(20, 40, 61, 16))
        self.label_5.setTextFormat(QtCore.Qt.AutoText)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.Box_Command)
        self.label_6.setGeometry(QtCore.QRect(20, 80, 61, 16))
        self.label_6.setTextFormat(QtCore.Qt.AutoText)
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(self.Box_Command)
        self.label_7.setGeometry(QtCore.QRect(20, 120, 61, 16))
        self.label_7.setTextFormat(QtCore.Qt.AutoText)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.Box_Command)
        self.label_8.setGeometry(QtCore.QRect(20, 160, 61, 16))
        self.label_8.setTextFormat(QtCore.Qt.AutoText)
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(self.Box_Command)
        self.label_9.setGeometry(QtCore.QRect(20, 200, 61, 16))
        self.label_9.setTextFormat(QtCore.Qt.AutoText)
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.Box_Command)
        self.label_10.setGeometry(QtCore.QRect(20, 240, 61, 16))
        self.label_10.setTextFormat(QtCore.Qt.AutoText)
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.Box_Command)
        self.label_11.setGeometry(QtCore.QRect(290, 20, 54, 12))
        self.label_11.setObjectName("label_11")
        #复位键
        self.bt_Reset = QtWidgets.QPushButton(self.Box_Command)
        self.bt_Reset.setGeometry(QtCore.QRect(150, 290, 75, 23))
        self.bt_Reset.setObjectName("bt_Reset")
        self.bt_Reset.clicked.connect(self.port_reset)
        #坐标框
        self.Box_Coord = QtWidgets.QGroupBox(Form)
        self.Box_Coord.setGeometry(QtCore.QRect(439, 139, 321, 331))
        self.Box_Coord.setObjectName("Box_Coord")
        #文本
        self.label_3 = QtWidgets.QLabel(self.Box_Coord)
        self.label_3.setGeometry(QtCore.QRect(20, 30, 54, 12))
        self.label_3.setObjectName("label_3")
        #目的坐标
        self.x_end = QtWidgets.QLineEdit(self.Box_Coord)
        self.x_end.setGeometry(QtCore.QRect(20, 150, 60, 20))
        self.x_end.setObjectName("x_end")
        self.x_end.setValidator(QtGui.QDoubleValidator())
        self.y_end = QtWidgets.QLineEdit(self.Box_Coord)
        self.y_end.setGeometry(QtCore.QRect(100, 150, 60, 20))
        self.y_end.setObjectName("y_end")
        self.y_end.setValidator(QtGui.QDoubleValidator())
        self.z_end = QtWidgets.QLineEdit(self.Box_Coord)
        self.z_end.setGeometry(QtCore.QRect(180, 150, 60, 20))
        self.z_end.setObjectName("z_end")
        self.z_end.setValidator(QtGui.QDoubleValidator())
        #文本
        self.label_4 = QtWidgets.QLabel(self.Box_Coord)
        self.label_4.setGeometry(QtCore.QRect(20, 120, 54, 12))
        self.label_4.setObjectName("label_4")
        #运动键
        self.bt_Sport = QtWidgets.QPushButton(self.Box_Coord)
        self.bt_Sport.setGeometry(QtCore.QRect(250, 140, 61, 31))
        self.bt_Sport.setObjectName("bt_Sport")
        self.bt_Sport.clicked.connect(self.port_sport)
        #当前坐标
        self.x_start = QtWidgets.QLineEdit(self.Box_Coord)
        self.x_start.setGeometry(QtCore.QRect(20, 60, 60, 20))
        self.x_start.setObjectName("x_start")
        self.x_start.setReadOnly(True)
        self.x_start.setText(str(1.1))
        self.y_start = QtWidgets.QLineEdit(self.Box_Coord)
        self.y_start.setGeometry(QtCore.QRect(100, 60, 60, 20))
        self.y_start.setObjectName("y_start")
        self.y_start.setReadOnly(True)
        self.y_start.setText(str(0))
        self.z_start = QtWidgets.QLineEdit(self.Box_Coord)
        self.z_start.setGeometry(QtCore.QRect(180, 60, 60, 20))
        self.z_start.setObjectName("z_start")
        self.z_start.setReadOnly(True)
        self.z_start.setText(str(34.72))
        #抓取键
        self.bt_Track= QtWidgets.QPushButton(Form)
        self.bt_Track.setGeometry(QtCore.QRect(450, 70, 75, 23))
        self.bt_Track.setObjectName("bt_Track")
        self.bt_Track.clicked.connect(self.Track)
        #手势识别按键
        self.bt_recog = QtWidgets.QPushButton(Form)
        self.bt_recog.setGeometry(QtCore.QRect(540, 70, 75, 23))
        self.bt_recog.setObjectName("bt_recog")
        self.bt_recog.clicked.connect(self.gestureRecog)
        
        #打开程序时锁定滑动条、计数器、复位键、运动键、手势识别
        self.bt_SerialOn.setEnabled(True)
        self.bt_SerialOff.setEnabled(False)
        self.Slider_1.setEnabled(False)
        self.Slider_2.setEnabled(False)
        self.Slider_3.setEnabled(False)
        self.Slider_4.setEnabled(False)
        self.Slider_5.setEnabled(False)
        self.Slider_6.setEnabled(False)
        self.spinBox_1.setEnabled(False)
        self.spinBox_2.setEnabled(False)
        self.spinBox_3.setEnabled(False)
        self.spinBox_4.setEnabled(False)
        self.spinBox_5.setEnabled(False)
        self.spinBox_6.setEnabled(False)
        self.bt_Reset.setEnabled(False)
        self.bt_Sport.setEnabled(False)
        self.bt_Track.setEnabled(False)
        self.bt_recog.setEnabled(False)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "萌萌哒的上位机"))
        self.Box_Setting.setTitle(_translate("Form", "串口连接与设置"))
        self.cBox_Choose.setItemText(0, _translate("Form", "COM1"))
        self.cBox_Choose.setItemText(1, _translate("Form", "COM2"))
        self.bt_SerialOn.setText(_translate("Form", "打开串口"))
        self.bt_SerialOff.setText(_translate("Form", "关闭串口"))
        self.label.setText(_translate("Form", "串口选择："))
        self.label_2.setText(_translate("Form", "波特率："))
        self.cBox_Bote.setItemText(0, _translate("Form", "2400"))
        self.cBox_Bote.setItemText(1, _translate("Form", "4800"))
        self.cBox_Bote.setItemText(2, _translate("Form", "9600"))
        self.cBox_Bote.setItemText(3, _translate("Form", "14400"))
        self.bt_SerialCheck.setText(_translate("Form", "检测串口"))
        self.Box_Command.setTitle(_translate("Form", "运动区"))
        self.label_5.setText(_translate("Form", "1号舵机"))
        self.label_6.setText(_translate("Form", "2号舵机"))
        self.label_7.setText(_translate("Form", "3号舵机"))
        self.label_8.setText(_translate("Form", "4号舵机"))
        self.label_9.setText(_translate("Form", "5号舵机"))
        self.label_10.setText(_translate("Form", "6号舵机"))
        self.label_11.setText(_translate("Form", "PWM"))
        self.bt_Reset.setText(_translate("Form", "复位"))
        self.Box_Coord.setTitle(_translate("Form", "坐标区"))
        self.label_3.setText(_translate("Form", "当前位置"))
        self.label_4.setText(_translate("Form", "目标位置"))
        self.bt_Sport.setText(_translate("Form", "运动"))
        self.bt_Track.setText(_translate("Form", "跟踪"))
        self.bt_recog.setText(_translate("Form", "手势识别"))

    # 串口检测
    def port_check(self):
        # 检测所有存在的串口，将信息存储在字典中
        self.Com_Dict = {}
        port_list = list(serial.tools.list_ports.comports())
        self.cBox_Choose.clear()
        for port in port_list:
            self.Com_Dict["%s" % port[0]] = "%s" % port[1]
            self.cBox_Choose.addItem(port[0])
        if len(self.Com_Dict) == 0:
            self.state_label.setText("无串口")

    # 打开串口
    def port_open(self):
        self.ser.port = self.cBox_Choose.currentText()
        self.ser.baudrate = int(self.cBox_Bote.currentText())

#        try:
        self.ser.open()
#        except:
#            QMessageBox.critical(self, "Port Error", "此串口不能被打开！")
#            return None


        if self.ser.isOpen():
            self.bt_SerialOn.setEnabled(False)
            self.bt_SerialOff.setEnabled(True)
            self.Slider_1.setEnabled(True)
            self.Slider_2.setEnabled(True)
            self.Slider_3.setEnabled(True)
            self.Slider_4.setEnabled(True)
            self.Slider_5.setEnabled(True)
            self.Slider_6.setEnabled(True)
            self.spinBox_1.setEnabled(True)
            self.spinBox_2.setEnabled(True)
            self.spinBox_3.setEnabled(True)
            self.spinBox_4.setEnabled(True)
            self.spinBox_5.setEnabled(True)
            self.spinBox_6.setEnabled(True)
            self.bt_Reset.setEnabled(True)
            self.bt_Sport.setEnabled(True)
            self.bt_Track.setEnabled(True)
            self.bt_recog.setEnabled(True)
#            self.state_label.setText("串口已开启")

    #关闭串口
    def port_close(self):
        try:
            self.ser.close()
        except:
            pass
        self.bt_SerialOn.setEnabled(True)
        self.bt_SerialOff.setEnabled(False)
        self.Slider_1.setEnabled(False)
        self.Slider_2.setEnabled(False)
        self.Slider_3.setEnabled(False)
        self.Slider_4.setEnabled(False)
        self.Slider_5.setEnabled(False)
        self.Slider_6.setEnabled(False)
        self.spinBox_1.setEnabled(False)
        self.spinBox_2.setEnabled(False)
        self.spinBox_3.setEnabled(False)
        self.spinBox_4.setEnabled(False)
        self.spinBox_5.setEnabled(False)
        self.spinBox_6.setEnabled(False)
        self.bt_Reset.setEnabled(False)
        self.bt_Sport.setEnabled(False)
        self.bt_Track.setEnabled(False)
        self.bt_recog.setEnabled(False)
#        self.state_label.setText("串口已关闭")

    #1号舵机pwm值
    def slider1_pwm(self):
        a=self.Slider_1.value()
        self.spinBox_1.setValue(a)

    #2号舵机pwm值
    def slider2_pwm(self):
        a=self.Slider_2.value()
        self.spinBox_2.setValue(a)
    
    #3号舵机pwm值
    def slider3_pwm(self):
        a=self.Slider_3.value()
        self.spinBox_3.setValue(a)
    
    #4号舵机pwm值
    def slider4_pwm(self):
        a=self.Slider_4.value()
        self.spinBox_4.setValue(a)
    
    #5号舵机pwm值
    def slider5_pwm(self):
        a=self.Slider_5.value()
        self.spinBox_5.setValue(a)
    
    #6号舵机pwm值
    def slider6_pwm(self):
        a=self.Slider_6.value()
        self.spinBox_6.setValue(a)

    #复位
    def port_reset(self):
        self.Slider_1.setValue(1500)
        self.Slider_2.setValue(1250)
        self.Slider_3.setValue(1500)
        self.Slider_4.setValue(1500)
        self.Slider_5.setValue(1500)
        self.Slider_6.setValue(1500)
        self.x_start.setText(str(1.1))
        self.y_start.setText(str(0))
        self.z_start.setText(str(34.72))
    
    #舵机1发送运动指令
    def port_send1(self):
        #0P2000T1000!
        pwm_str=str(self.spinBox_1.value())
        sport_str="#0P"+pwm_str+"T500!"
        self.ser.write(sport_str.encode())

    #舵机2发送运动指令
    def port_send2(self):
        #0P2000T1000!
        pwm_str=str(self.spinBox_2.value())
        sport_str="#1P"+pwm_str+"T500!"
        self.ser.write(sport_str.encode())

    #舵机3发送运动指令
    def port_send3(self):
        #0P2000T1000!
        pwm_str=str(3000-self.spinBox_3.value())
        sport_str="#2P"+pwm_str+"T500!"
        self.ser.write(sport_str.encode())

    #舵机4发送运动指令
    def port_send4(self):
        #0P2000T1000!
        pwm_str=str(3000-self.spinBox_4.value())
        sport_str="#3P"+pwm_str+"T500!"
        self.ser.write(sport_str.encode())

    #舵机5发送运动指令
    def port_send5(self):
        #0P2000T1000!
        pwm_str=str(self.spinBox_5.value()-750)
        sport_str="#4P"+pwm_str+"T500!"
        self.ser.write(sport_str.encode())

    #舵机6发送运动指令
    def port_send6(self):
        #0P2000T1000!
        pwm_str=str(self.spinBox_6.value())
        sport_str="#5P"+pwm_str+"T500!"
        self.ser.write(sport_str.encode())

#    def mult_sport(self):
#        
#        # {G0#1P2000T1000!#0P2000T1000!}
##        pwm0_str=
##        pwm1_str=
##        pwm2_str=
##        pwm3_str=
##        pwm4_str=
#        pwm5_str="500"
#        open_str="#5P"+pwm5_str+"T500!"
#        self.ser.write(open_str.encode())
##        sport_str="{G0#0P"+pwm0_str+"T500!"+"#1P"+pwm1_str+"T500!"+"#2P"+pwm2_str+"T500!"+"#3P"+pwm3_str+"T500!"+"#4P"+pwm4_str+"T500!}"
##        self.ser.write(sport_str.encode())
#        delay_str="$DELAY100MS:10!"
#        self.ser.write(delay_str.encode())
#        self.port_sport()
#        pwm5_str="1500"
#        close_str="#5P"+pwm5_str+"T500!"
#        self.ser.write(close_str.encode())
#        
    #运动时更新当前坐标，正运动学
    def coord_update(self):
        xita1=0.09*(self.spinBox_1.value()-1500)*pi/180
        xita2=0.1286*(self.spinBox_2.value()-1250)*pi/180
        xita3=0.09*(self.spinBox_3.value()-1500)*pi/180
        xita4=0.1384*(self.spinBox_4.value()-1500)*pi/180
        xita5=0.09*(self.spinBox_5.value()-1500)*pi/180
        #d=1.1
        #L1=8.1
        #L2=7.5
        #L3=20.2
        x_now=11*cos(xita1)*sin(xita2+xita3+xita4+xita5)+(cos(xita1)*(92*sin(xita2+xita3+xita4)+75*sin(xita2+xita3)+81*sin(xita2)+11))/10
        y_now=11*sin(xita1)*sin(xita2+xita3+xita4+xita5)+(sin(xita1)*(92*sin(xita2+xita3+xita4)+75*sin(xita2+xita3)+81*sin(xita2)+11))/10
        z_now=(46*cos(xita2+xita3+xita4))/5+11*cos(xita2+xita3+xita4+xita5)+(15*cos(xita2+xita3))/2+(81*cos(xita2))/10
        x_now=round(x_now, 2)
        y_now=round(y_now, 2)
        z_now=round(z_now, 2)
        self.x_start.setText(str(x_now))
        self.y_start.setText(str(y_now))
        self.z_start.setText(str(z_now))


#    def msg(self):  
#        reply = QtWidgets.QMessageBox.information(self,                         #使用infomation信息框  
#                                    "标题",  
#                                    "消息",  
#                                    QMessageBox.Yes | QMessageBox.No)

    #根据坐标进行运动，调用逆运动学函数
    def port_sport(self):
        x0 = [0, 0, 0, 0, 0]
        result = fsolve(self.f, x0)
        k=0
        while(abs(result[0]) > pi/2 or abs(result[1]) > pi/2 or abs(result[2]) > pi/2 or abs(result[3]) > pi/2 or abs(result[4]) > pi/2):
            x0[0] = x0[0] + 0.1
            x0[1] = x0[1] + 0.05
            result = fsolve(self.f, x0)
            k += 1
            if k>100:
                errorMessage=QtWidgets.QMessageBox.information(Form,"提示", "无法求出对应的解", QtWidgets.QMessageBox.Yes)
                print(errorMessage)
                return
        #弧度制转角度制
        #result=result/pi*180
#        Message=QtWidgets.QMessageBox.information(Form,"结果", str(result), QtWidgets.QMessageBox.Yes)
#        print(Message)
        #角度转PWM
        
        PWM=[0, 0, 0, 0,0]
        PWM[0]=((result[0]/pi)*180)/0.09+1500
        PWM[1]=((result[1]/pi)*180)/0.1286+1250
        PWM[2]=((result[2]/pi)*180)/0.09+1500
        PWM[3]=((result[3]/pi)*180)/0.1384+1500
        PWM[4]=((result[4]/pi)*180)/0.09+1500
        self.Slider_1.setValue(int(PWM[0]))
        self.Slider_2.setValue(int(PWM[1]))
        self.Slider_3.setValue(int(PWM[2]))
        self.Slider_4.setValue(int(PWM[3]))
        self.Slider_5.setValue(int(PWM[4]))

    #逆运动学求解函数
    def f(self, x):
        xw=float(self.x_end.text())
        yw=float(self.y_end.text())
        zw=float(self.z_end.text())
        x0 = float(x[0])
        x1 = float(x[1])
        x2 = float(x[2])
        x3 = float(x[3])
        x4 = float(x[4])

        return [(cos(x0)*(92*sin(x1 + x2 + x3) + 75*sin(x1 + x2) + 81*sin(x1) + 11))/10 + 11*sin(x1 + x2 + x3 + x4)*cos(x0) - xw,
                (sin(x0)*(92*sin(x1 + x2 + x3) + 75*sin(x1 + x2) + 81*sin(x1) + 11))/10 + 11*sin(x1 + x2 + x3 + x4)*sin(x0) - yw,
                (46*cos(x1 + x2 + x3))/5 + 11*cos(x1 + x2 + x3 + x4) + (15*cos(x1 + x2))/2 + (81*cos(x1))/10 - zw,
#                tan(x0) - yw/xw,
                x2 + x3 + x4 - pi/2, 
                x4
                ]
                
    def gestureRecog(self):
        APP_ID = '16534881'
        API_KEY = 'DSG6hUBgBVt61Ms1XXnYuEPm'
        SECRET_KEY = 'W4nkl6xYmOnRjntg53fDMuxQXvqemNI0'
        a = AipBodyAnalysis(APP_ID, API_KEY, SECRET_KEY)

        cap=cv2.VideoCapture(0)
        while 1:
            ret,image=cap.read()
            cv2.imshow("capture", image)
            if cv2.waitKey(100)&0xff==ord("q"):
                cv2.imwrite("test.jpg", image)
                break

        cap.release()
        cv2.destroyAllWindows()

        filePath = "test.jpg"



        options = {}

        options["top_num"] = 5 

        result=a.gesture(self.get_file_content(filePath),options)

        print(result)

        k=result["result"][0]["classname"]
        
        if k=="Insult": 
                self.port_reset()
        if  k=="ILY":
                self.Slider_3.setValue(500)
                time.sleep(0.5)
                self.Slider_3.setValue(1500)
        else:
            return

    def get_file_content(self, filePath):

        with open(filePath, 'rb') as fp:

            return fp.read()
            
    
    #追踪功能
    def Track(self):
        cap = cv2.VideoCapture(1)
        while(1):
            u_center = 320
            v_center = 240
            x = 30
            self.x_end.setText(str(x))
            y = 0
            self.y_end.setText(str(y))
            z = 10 
            self.z_end.setText(str(z))
            self.port_sport()
            time.sleep(1)
            delta = 2

            ret, img = cap.read()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            height = gray.shape[0]
            width = gray.shape[1]
            sumx = 0
            sumy = 0
            num = 0
            black = np.ones([height,width])
            for i in range(height):
                for j in range(width):
                    if gray[i, j] < 20:
                        sumx += i
                        sumy += j
                        num += 1
                        black[i, j] = 0
                        #edges= filters.median(black,disk(9))
                        #print(i, j)

            if num != 0:
                
                v = sumx/num
                u = sumy/num
                eu = u - u_center
                ev = v - v_center
                break
                
            #cv.imshow('img', black)
            #cv.waitKey(1)
            #return [eu, ev]

        while((abs(eu) > 50) or (abs(eu) > 50)):
            if (eu > 0) and (ev > 0):
                y = y - delta
                z = z - delta
                #eu, ev = get_dis()
            elif (eu > 0) and (ev < 0):
                y = y - delta
                z = z + delta
                #eu, ev = get_dis()
            elif (eu < 0) and (ev > 0):
                y = y + delta
                z = z - delta
                #eu, ev = get_dis()
            elif (eu < 0) and (ev < 0):
                y = y + delta
                z = z + delta
                #eu, ev = get_dis()
            while(1):
                ret, img = cap.read()
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                height = gray.shape[0]
                width = gray.shape[1]
                sumx = 0
                sumy = 0
                num = 0
                black = np.ones([height,width])
                for i in range(height):
                    for j in range(width):
                        if gray[i, j] < 20:
                            sumx += i
                            sumy += j
                            num += 1
                            black[i, j] = 0
                            #edges= filters.median(black,disk(9))
                            #print(i, j)
                
                if num != 0:
                    
                    v = sumx/num
                    u = sumy/num
                    eu = u - u_center
                    ev = v - v_center
                    break
            
            cv2.imshow('img', black)
            cv2.waitKey(1)
#            cap.release()
#            cv2.destroyAllWindows()

#            print("eu:",eu,"ev:", ev)
#            print("y:",y,"z:", z)
            self.y_end.setText(str(y))
            self.z_end.setText(str(z))
            self.port_sport()
    
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

