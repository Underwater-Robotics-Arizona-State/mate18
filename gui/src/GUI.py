#!/usr/bin/env python
import wx
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16, Float32, Int16MultiArray, String
 
class DataDisplay(wx.Frame):
    def __init__(self):
        super(DataDisplay, self).__init__(None, size=(600, 400))    #parent=None
        self.InitUI()
        self.Centre()
        self.Show()
         
        self._motor_fl_max = self._motor_fl_min = 1500
        self._motor_fr_max = self._motor_fl_min = 1500
        self._motor_bl_max = self._motor_fl_min = 1500
        self._motor_br_max = self._motor_fl_min = 1500
        self._motor_ft_max = self._motor_fl_min = 1500
        self._motor_bt_max = self._motor_fl_min = 1500

    def InitUI(self):
        #setup menubar and add a menu called 'File' that can be accessed with shortcut ctrl+f
        menubar = wx.MenuBar()
        self.fileMenu = wx.Menu()
        self.launch_ros = self.fileMenu.Append(wx.ID_ANY, 'launch nodes', kind=wx.ITEM_CHECK)   #first menu item
        self.shutdown_ros = self.fileMenu.Append(wx.ID_ANY, 'shutdown roscore')                 #second menu item
        self.fileMenu.Check(self.launch_ros.GetId(), False)                                     #first menu item initially unchecked
        menubar.Append(self.fileMenu, '&File')
        #menubar.Append(wx.ID_EXIT, '&Quit \tCtrl+Q')
        self.SetMenuBar(menubar)
         
        #Do ROS stuff if items in menu are clicked
        #self.Bind(EVENT_BINDER, eventHandlerMethod, id=target_object)
        self.Bind(wx.EVT_MENU, self.launchRos, self.launch_ros)
        self.Bind(wx.EVT_MENU, self.shutdownRos, self.shutdown_ros)
         
        #statusbar at bottom of GUI indicates that ROS is running
        self.statusbar = self.CreateStatusBar()
        self.statusbar.Show()
         
        #setup main panel in GUI, and initialize vertical box sizer
        mainPanel = wx.Panel(self)
        vbox = wx.BoxSizer(wx.VERTICAL)
         
         
        #setup StaticBox to visually separate different categories of data
        accelBox = wx.StaticBox(mainPanel, label = "Accelerometer Data")
        gyroBox = wx.StaticBox(mainPanel, label = "Gyroscope Data")
        #magBox = wx.StaticBox(mainPanel, label = "Magnetometer Data")
        processedBox = wx.StaticBox(mainPanel, label = "Processed Data") #bluetooth, pressure sensor, compass calculated from magnetometer
        motorBox = wx.StaticBox(mainPanel, label = "Motor Values")
         
        #proportion=1 allows box to be stretched with window
        #flag=wx.EXPAND tells the text panel to take up the whole box allocated by StaticBoxSizer
        #flab=wx.LEFT/RIGHT/TOP/BOTTOM indicates which borders the border spacing border=4 applies to
        #setup accelBox with 6 panels oriented horizontally
        accelBoxSizer = wx.StaticBoxSizer(accelBox, wx.HORIZONTAL)  #child of accelBox
        accel_x = wx.StaticText(mainPanel, label="accel_x: ", style=wx.ALIGN_RIGHT)
        accel_y = wx.StaticText(mainPanel, label="accel_y: ", style=wx.ALIGN_RIGHT)
        accel_z = wx.StaticText(mainPanel, label="accel_z: ", style=wx.ALIGN_RIGHT)
        self.accel_x_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self.accel_y_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self.accel_z_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        accelBoxSizer.Add(accel_x, flag=wx.EXPAND|wx.LEFT, border=4)
        accelBoxSizer.Add(self.accel_x_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        accelBoxSizer.Add(accel_y, flag=wx.EXPAND|wx.LEFT, border=4)
        accelBoxSizer.Add(self.accel_y_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        accelBoxSizer.Add(accel_z, flag=wx.EXPAND|wx.LEFT, border=4)
        accelBoxSizer.Add(self.accel_z_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
         
        #setup gyroBox with 6 panels oriented horizontally
        gyroBoxSizer = wx.StaticBoxSizer(gyroBox, wx.HORIZONTAL)  #child of gyroBox
        gyro_yaw = wx.StaticText(mainPanel, label="yaw: ", style=wx.ALIGN_RIGHT)
        gyro_pitch = wx.StaticText(mainPanel, label="pitch: ", style=wx.ALIGN_RIGHT)
        gyro_roll = wx.StaticText(mainPanel, label="roll: ", style=wx.ALIGN_RIGHT)
        self.gyro_yaw_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self.gyro_pitch_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self.gyro_roll_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        gyroBoxSizer.Add(gyro_yaw, flag=wx.EXPAND|wx.LEFT, border=4)
        gyroBoxSizer.Add(self.gyro_yaw_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        gyroBoxSizer.Add(gyro_pitch, flag=wx.EXPAND|wx.LEFT, border=4)
        gyroBoxSizer.Add(self.gyro_pitch_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        gyroBoxSizer.Add(gyro_roll, flag=wx.EXPAND|wx.LEFT, border=4)
        gyroBoxSizer.Add(self.gyro_roll_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
         
        ##########################uncomment for mag raw values################################
        #setup magBox with 6 panels oriented horizontally
        #magBoxSizer = wx.StaticBoxSizer(magBox, wx.HORIZONTAL)  #child of gyroBox
        #mag_x = wx.StaticText(mainPanel, label="mag_x: ", style=wx.ALIGN_RIGHT)
        #mag_y = wx.StaticText(mainPanel, label="mag_y: ", style=wx.ALIGN_RIGHT)
        #mag_z = wx.StaticText(mainPanel, label="mag_z: ", style=wx.ALIGN_RIGHT)
        #self.mag_x_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        #self.mag_y_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        #self.mag_z_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        #magBoxSizer.Add(mag_x, flag=wx.EXPAND|wx.LEFT, border=4)
        #magBoxSizer.Add(self.mag_x_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        #magBoxSizer.Add(mag_y, flag=wx.EXPAND|wx.LEFT, border=4)
        #magBoxSizer.Add(self.mag_y_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        #magBoxSizer.Add(mag_z, flag=wx.EXPAND|wx.LEFT, border=4)
        #magBoxSizer.Add(self.mag_z_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        ########################################################################################
         
        #setup processedBox with 6 panels oriented horizontally
        processedBoxSizer = wx.StaticBoxSizer(processedBox, wx.HORIZONTAL)  #child of processedBox
        bt = wx.StaticText(mainPanel, label="bt code: ", style=wx.ALIGN_RIGHT)
        depth = wx.StaticText(mainPanel, label="depth: ", style=wx.ALIGN_RIGHT)
        direction = wx.StaticText(mainPanel, label="direction: ", style=wx.ALIGN_RIGHT)
        self.bt_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self.depth_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self.direction_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        processedBoxSizer.Add(bt, flag=wx.EXPAND|wx.LEFT, border=4)
        processedBoxSizer.Add(self.bt_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        processedBoxSizer.Add(depth, flag=wx.EXPAND|wx.LEFT, border=4)
        processedBoxSizer.Add(self.depth_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        processedBoxSizer.Add(direction, flag=wx.EXPAND|wx.LEFT, border=4)
        processedBoxSizer.Add(self.direction_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        
        motorBoxSizer = wx.StaticBoxSizer(motorBox, wx.VERTICAL)  #child of accelBox
        motor_line1 = wx.BoxSizer(wx.HORIZONTAL)
        motor_line2 = wx.BoxSizer(wx.HORIZONTAL)
        motor_fl = wx.StaticText(mainPanel, label="motor_fl: ", style=wx.ALIGN_RIGHT)
        motor_fr = wx.StaticText(mainPanel, label="motor_fr: ", style=wx.ALIGN_RIGHT)
        motor_bl = wx.StaticText(mainPanel, label="motor_bl: ", style=wx.ALIGN_RIGHT)
        motor_br = wx.StaticText(mainPanel, label="motor_br: ", style=wx.ALIGN_RIGHT)
        motor_ft = wx.StaticText(mainPanel, label="motor_ft: ", style=wx.ALIGN_RIGHT)
        motor_bt = wx.StaticText(mainPanel, label="motor_bt: ", style=wx.ALIGN_RIGHT)
        self._motor_fl_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self._motor_fr_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self._motor_bl_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self._motor_br_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self._motor_ft_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        self._motor_bt_data = wx.TextCtrl(mainPanel, style=wx.TE_READONLY)
        motor_line1.Add(motor_fl, flag=wx.EXPAND|wx.LEFT, border=4)
        motor_line1.Add(self._motor_fl_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        motor_line1.Add(motor_ft, flag=wx.EXPAND|wx.LEFT, border=4)
        motor_line1.Add(self._motor_ft_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        motor_line1.Add(motor_fr, flag=wx.EXPAND|wx.LEFT, border=4)
        motor_line1.Add(self._motor_fr_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        motor_line2.Add(motor_bl, flag=wx.EXPAND|wx.LEFT, border=4)
        motor_line2.Add(self._motor_bl_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        motor_line2.Add(motor_bt, flag=wx.EXPAND|wx.LEFT, border=4)
        motor_line2.Add(self._motor_bt_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        motor_line2.Add(motor_br, flag=wx.EXPAND|wx.LEFT, border=4)
        motor_line2.Add(self._motor_br_data, flag=wx.EXPAND|wx.RIGHT, border=4, proportion=1)
        motorBoxSizer.Add(motor_line1, flag=wx.EXPAND|wx.BOTTOM, border=4, proportion=1)
        motorBoxSizer.Add(motor_line2, flag=wx.EXPAND|wx.BOTTOM, border=4, proportion=1) 
         
        #only BoxSizers need to be added, because their respective parents will be included by default
        vbox.Add(accelBoxSizer, flag=wx.EXPAND|wx.TOP|wx.BOTTOM, border=5)
        vbox.Add(gyroBoxSizer, flag=wx.EXPAND|wx.TOP|wx.BOTTOM, border=5)
        #vbox.Add(magBoxSizer, flag=wx.EXPAND|wx.TOP|wx.BOTTOM, border=5)
        vbox.Add(processedBoxSizer, flag=wx.EXPAND|wx.TOP|wx.BOTTOM, border=5)
        vbox.Add(motorBoxSizer, flag=wx.EXPAND|wx.TOP|wx.BOTTOM, border=5)
         
        #setup main panel to use the vertical box sizer
        mainPanel.SetSizerAndFit(vbox)
     
    def dispAccel(self, data):
        self.accel_x_data.SetValue(str(data[0])+" g")
        self.accel_y_data.SetValue(str(data[1])+" g")
        self.accel_z_data.SetValue(str(data[2])+" g")
     
    def dispGyro(self, data):
        self.gyro_yaw_data.SetValue(str(data[0])+" deg/s")
        self.gyro_pitch_data.SetValue(str(data[1])+" deg/s")
        self.gyro_roll_data.SetValue(str(data[2])+" deg/s")
     
    def dispMag(self, data):
        self.direction_data.SetValue(str(data)+" degrees")
        #############Uncomment for mag raw values###################
        #self.mag_x_data.SetValue(str(data[6])+" m/s")
        #self.mag_y_data.SetValue(str(data[7])+" m/s")
        #self.mag_z_data.SetValue(str(data[8])+" m/s")
        ############################################################
         
    def dispBt(self, data):
        self.bt_data.SetValue(data)
     
    def launchRos(self, e):
        if self.launch_ros.IsChecked():
            self.statusbar.SetStatusText('ROS running')
     
    def shutdownRos(self, e):
        self.statusbar.SetStatusText('roscore shutdown')
        self.fileMenu.Check(self.launch_ros.GetId(), False)

    def motorChecker(self, motorValues):
        if motorValues[0] < self._motor_fl_min: self._motor_fl_min = motorValues[0]
        if motorValues[0] > self._motor_fl_max: self._motor_fl_max = motorValues[0]
        if motorValues[1] < self._motor_fr_min: self._motor_fr_min = motorValues[1]
        if motorValues[1] > self._motor_fr_max: self._motor_fr_max = motorValues[1]
        if motorValues[2] < self._motor_bl_min: self._motor_bl_min = motorValues[2]
        if motorValues[2] > self._motor_bl_max: self._motor_bl_max = motorValues[2]
        if motorValues[3] < self._motor_br_min: self._motor_br_min = motorValues[3]
        if motorValues[3] > self._motor_br_max: self._motor_br_max = motorValues[3]
        if motorValues[4] < self._motor_ft_min: self._motor_ft_min = motorValues[4]
        if motorValues[4] > self._motor_ft_max: self._motor_ft_max = motorValues[4]
        if motorValues[5] < self._motor_bt_min: self._motor_bt_min = motorValues[5]
        if motorValues[5] > self._motor_bt_max: self._motor_bt_max = motorValues[5]
 
        self._motor_fl_data.SetValue(str(self._motor_fl_min) + "/" + str(self._motor_fl_max))
        self._motor_fr_data.SetValue(str(self._motor_fr_min) + "/" + str(self._motor_fr_max))
        self._motor_bl_data.SetValue(str(self._motor_bl_min) + "/" + str(self._motor_bl_max))
        self._motor_br_data.SetValue(str(self._motor_br_min) + "/" + str(self._motor_br_max))
        self._motor_ft_data.SetValue(str(self._motor_ft_min) + "/" + str(self._motor_ft_max))
        self._motor_bt_data.SetValue(str(self._motor_bt_min) + "/" + str(self._motor_bt_max))
         
         
 
class Video(object):
    def __init__(self):
        self._frame = None
        self._bridge = CvBridge()
        cv2.startWindowThread()
        #cv2.namedWindow('ROV Video')
     
    def vidCallback(self, data):
        try:
            self._frame = self._bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow('ROV Video', self._frame)
            cv2.waitKey(1)
        except CvBridgeError, e:
            print e


def main():
    app = wx.App()
     
    rospy.init_node('gui')  #creates a node
    gui = DataDisplay()     #initialize GUI class to be used below for callbacks
 
    #subscribes to imu topics, and passes data to relevant callbacks
    rospy.Subscriber("/magnetometer", Float32, gui.dispMag)
    rospy.Subscriber("/accelerometer", Int16MultiArray, gui.dispAccel)
    rospy.Subscriber("/gyroscope", Int16MultiArray, gui.dispGyro)
    #rospy.Subscriber("/motorValues", Int16MultiArray, gui.motorChecker)

    camStream1 = Video()
    rospy.Subscriber("front_cam/left", Image, camStream1.vidCallback)
    camStream2 = Video()
    rospy.Subscriber("front_cam/right", Image, camStream2.vidCallback)
    camStream3 = Video()
    rospy.Subscriber("/front_cam/bottom", Image, camStream3.vidCallback)
     
    app.MainLoop()
    rospy.spin()
     
if __name__ == '__main__':
    main()
