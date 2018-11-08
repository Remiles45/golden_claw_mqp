import os
import sys
import rospkg
import rospy

from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from PyQt5.QtCore import QEvent, Qt, QTimeLine, QTimer
from std_msgs.msg import Int16MultiArray
from reflex_msgs.msg import PoseCommand, Command, VelocityCommand
from std_msgs.msg import UInt16
from os import listdir
from os.path import isfile, join
from calibration_widget import CalibrationWidget
from glove_cntrl_widget import GloveWidget
from file_wr import FileWriteRead


file = FileWriteRead()
rospack = rospkg.RosPack()
FILE_DIR = rospack.get_path('rqt_gui_control') + '/data'
class ManualHandControlWidget(QWidget):


    def __init__(self):
        self.rate = rospy.Rate(2)
        super(ManualHandControlWidget, self).__init__()
        #Reflex SF hand control
        self.command_pub = rospy.Publisher('/reflex_sf/command', Command, queue_size=1) #_position
        self.currentGrasp = []
        self.initUI()
        self.delete_waypoint = None
        #soft hand control
        self.command_pub_softhand_1 = rospy.Publisher('UbirosGentlePro1', UInt16, queue_size=1)
        self.command_pub_softhand_2 = rospy.Publisher('UbirosGentlePro2', UInt16, queue_size=1)
        self.command_pub_softhand_3 = rospy.Publisher('UbirosGentlePro3', UInt16, queue_size=1)
        self.command_pub_softhand_4 = rospy.Publisher('UbirosGentlePro4', UInt16, queue_size=1)

        # self.filename = []
        # self.fileListWidget = QListWidget()

    def initUI(self):

################## Position Control GUI ########################################################################
######### Automatic Hand Move ###################################################################################
        #TODO: this may happen too fast? need to test on hand, make smooth like buddah
        #Synchronize selected fingers
        self.hbox_update_tick = QHBoxLayout()
        self.live_update = QCheckBox("Turn on live update")
        self.hbox_update_tick.addWidget(self.live_update)
        self.hbox_update_tick.addStretch()
        self.tick_update_state = 0
        self.live_update.stateChanged.connect(lambda:self.tickchange(self.live_update))


#Fingers: slider range 0 -> 200
        #Finger 1 row (F1)
        self.finger_label_1 = QLabel("Goal for f1")
        self.finger_slider_1 = QSlider(1)
        self.finger_slider_1.setMinimum(0)
        self.finger_slider_1.setMaximum(200)
        self.finger_slider_1.setValue(0)

        self.value_slider_1 = QLabel("0.00")
        self.value_slider_1.setMaximumSize(80,20)
        self.f1_speed = QDoubleSpinBox()
        self.f1_speed.setSingleStep(0.05)
        self.f1_speed.setValue(0.05)
        self.hbox_f1 = QHBoxLayout()
        self.hbox_f1.addWidget(self.finger_slider_1)
        self.hbox_f1.addWidget(self.value_slider_1)
        self.hbox_f1.addWidget(self.f1_speed)
        self.hbox_f1.addWidget( QLabel("Rad/Sec"))

        #Finger 2 row (F2)
        self.finger_label_2 = QLabel("Goal for f2")
        self.finger_slider_2 = QSlider(1)
        self.finger_slider_2.setMinimum(0)
        self.finger_slider_2.setMaximum(200)
        self.finger_slider_2.setValue(0)

        self.value_slider_2 = QLabel("0.00")
        self.value_slider_2.setMaximumSize(80,20)
        self.f2_speed = QDoubleSpinBox()
        self.f2_speed.setSingleStep(0.05)
        self.f2_speed.setValue(0.05)
        self.hbox_f2 = QHBoxLayout()
        self.hbox_f2.addWidget(self.finger_slider_2)
        self.hbox_f2.addWidget(self.value_slider_2)
        self.hbox_f2.addWidget(self.f2_speed)
        self.hbox_f2.addWidget( QLabel("Rad/Sec"))

        #Finger 3 row (F3)
        self.finger_label_3 = QLabel("Goal for f3")
        self.finger_slider_3 = QSlider(1)
        self.finger_slider_3.setMinimum(0)
        self.finger_slider_3.setMaximum(200)
        self.finger_slider_3.setValue(0)

        self.value_slider_3 = QLabel("0.00")
        self.value_slider_3.setMaximumSize(80,20)
        self.f3_speed = QDoubleSpinBox()
        self.f3_speed.setSingleStep(0.05)
        self.f3_speed.setValue(0.05)
        self.hbox_f3 = QHBoxLayout()
        self.hbox_f3.addWidget(self.finger_slider_3)
        self.hbox_f3.addWidget(self.value_slider_3)
        self.hbox_f3.addWidget(self.f3_speed)
        self.hbox_f3.addWidget( QLabel("Rad/Sec"))


        #Reflex SF hand distance between fingers (k1)
        self.finger_label_4 = QLabel("Distance between fingers 1 and 2")
        self.finger_slider_4 = QSlider(1)
        self.finger_slider_4.setMinimum(0)
        self.finger_slider_4.setMaximum(200)
        self.finger_slider_4.setValue(0)
        #initialize at position 0
        self.value_slider_4 = QLabel("0.00")
        self.value_slider_4.setMaximumSize(80,20)
        self.f4_speed = QDoubleSpinBox()
        self.f4_speed.setSingleStep(0.05)
        self.f4_speed.setValue(0.05)
        self.hbox_f4 = QHBoxLayout()
        self.hbox_f4.addWidget(self.finger_slider_4)
        self.hbox_f4.addWidget(self.value_slider_4)
        self.hbox_f4.addWidget(self.f4_speed)
        self.hbox_f4.addWidget( QLabel("Rad/Sec"))


        #Reflex SF hand thumb rotation (k2)
        #set range to -100 to 100 since it is easier for user to conceptualize
        self.finger_label_5 = QLabel("Thumb Rotation")
        self.finger_slider_5 = QSlider(1)
        self.finger_slider_5.setMinimum(-100)
        self.finger_slider_5.setMaximum(100)
        self.finger_slider_5.setValue(0)
        #set initial value to center position
        self.value_slider_5 = QLabel("0.00")
        self.value_slider_5.setMaximumSize(80,20)
        self.f5_speed = QDoubleSpinBox()
        self.f5_speed.setSingleStep(0.05)
        self.f5_speed.setValue(0.05)
        self.hbox_f5 = QHBoxLayout()
        self.hbox_f5.addWidget(self.finger_slider_5)
        self.hbox_f5.addWidget(self.value_slider_5)
        self.hbox_f5.addWidget(self.f5_speed)
        self.hbox_f5.addWidget( QLabel("Rad/Sec"))


########## Coupling Row ###################################################################################
        #Synchronize selected fingers
        self.coupling_label = QLabel("Coupling")
        self.hbox_tick = QHBoxLayout()

        self.tick_f1 = QCheckBox("F1")
        self.tick_f2 = QCheckBox("F2")
        self.tick_f3 = QCheckBox("F3")
        #only display finger 4 for the Reflex SF hand
        self.tick_f4 = QCheckBox("F4")
        self.tick_f4.setHidden(True)

        self.hbox_tick.addWidget(self.tick_f1)
        self.hbox_tick.addWidget(self.tick_f2)
        self.hbox_tick.addWidget(self.tick_f3)
        self.hbox_tick.addWidget(self.tick_f4)
        self.hbox_tick.addStretch()

        self.tick_f1_state = 0
        self.tick_f2_state = 0
        self.tick_f3_state = 0
        self.tick_f4_state = 0
        self.tick_f1.stateChanged.connect(lambda:self.tickchange(self.tick_f1))
        self.tick_f2.stateChanged.connect(lambda:self.tickchange(self.tick_f2))
        self.tick_f3.stateChanged.connect(lambda:self.tickchange(self.tick_f3))
        self.tick_f4.stateChanged.connect(lambda:self.tickchange(self.tick_f4))

########### Command Row Button ############################################################################
        self.command_label = QLabel("Manual Command Hand")
        self.go_button = QPushButton("Go to set values")
        self.re_button = QPushButton("Reset Goal Values")
        self.home_button = QPushButton("Finger Home")

        self.hbox_command = QHBoxLayout()
        self.hbox_command.addWidget(self.go_button)
        self.hbox_command.addWidget(self.re_button)
        self.hbox_command.addWidget(self.home_button)
########### Select Active Hand ############################################################################
        self.combo_label = QLabel("Targeted Device")
        self.combo = QComboBox(self)
        self.combo.addItem("ReflexSF")
        self.combo.addItem("Soft Hand")

############ Manage Waypoint List ##############################################################################################
        self.listCommand = []
        self.grasplist = []
        self.filename = []
        #initial point at conceptual home position (thumb in center position)
        self.pose0 = PoseCommand(f1=0.0,f2=0.0,f3=0.0,k1=0.0,k2=1.0)
        self.velocity0 = VelocityCommand(f1=1,f2=1,f3=1,k1=1,k2=1)
        command0 = Command(pose=self.pose0, velocity=self.velocity0)
        self.listCommand.append(command0)
        #Display waypoints and files
        self.listWidget = QListWidget()
        self.fileManagement = QListWidget()#FileWriteRead()#QListWidget()
        self.listWidget.installEventFilter(self)#######################################################3
        self.populate_filelist()
        self.populate_commandlist()

        self.fileslabel = QLabel("Grasp Files")
        self.listlabel = QLabel("List waypoint")

        # Options for File List
        self.file_control = QHBoxLayout()
        self.save_file_button = QPushButton("Save to File")
        self.file_load_button = QPushButton("Load File into Waypoint List")
        self.file_execute_button = QPushButton("Execute File")
        self.file_control.addWidget(self.file_load_button)
        self.file_control.addWidget(self.save_file_button)
        self.file_control.addWidget(self.file_execute_button)

        #List Control
        self.list_control_label = QLabel("Waypoint Control")
        self.list_control_save_button = QPushButton("Add Waypoint")
        self.list_control_delay_button = QPushButton("Add Delay")
        self.list_control_execute_waypoints = QPushButton("Execute Waypoints")
        self.list_control_save_grasp = QPushButton("Save Grasp")
        self.list_control_delete_all = QPushButton("Delete All Waypoints")
        self.list_control = QHBoxLayout()
        self.list_control.addWidget(self.list_control_save_button)
        self.list_control.addWidget(self.list_control_delay_button)
        self.list_control.addWidget(self.list_control_execute_waypoints)
        self.list_control.addWidget(self.list_control_delete_all)

        #calibration
        self.calibration = QHBoxLayout()
        self.calibrate_button = QPushButton("Calibrate Hand")
        self.calibration.addWidget(self.calibrate_button)

############ Adding Sections to GUI ####################################################
#using the buttons defined above to create the GUI itself
        self.fbox = QFormLayout()
        self.fbox.addRow(QLabel(""),self.hbox_update_tick)
        self.fbox.addRow(self.finger_label_1,self.hbox_f1)
        self.fbox.addRow(self.finger_label_2,self.hbox_f2)
        self.fbox.addRow(self.finger_label_3,self.hbox_f3)
        self.fbox.addRow(self.finger_label_4,self.hbox_f4)
        self.fbox.addRow(self.finger_label_5,self.hbox_f5)
        self.fbox.addRow(self.coupling_label,self.hbox_tick)
        self.fbox.addRow(self.command_label,self.hbox_command)
        self.fbox.addRow(self.listlabel, self.listWidget)
        self.fbox.addRow(self.list_control_label,self.list_control)
        self.fbox.addRow(self.fileslabel, self.fileManagement)
        self.fbox.addRow(QLabel(""), self.file_control)
        self.fbox.addRow(QLabel(""), self.calibration)
        self.fbox.addRow(self.combo_label,self.combo)
        # self.fbox.addRow(self.glove_label,self.hbox_glove)

        # Connect signal when slider change to function respectively to change value of label
        self.finger_slider_1.valueChanged.connect(self.valuechange1)
        self.finger_slider_2.valueChanged.connect(self.valuechange2)
        self.finger_slider_3.valueChanged.connect(self.valuechange3)
        self.finger_slider_4.valueChanged.connect(self.valuechange4)
        self.finger_slider_5.valueChanged.connect(self.valuechange5)

        # connect slider to measure distance scrolled
        self.finger_slider_1.sliderPressed.connect(self.storeValue)
        self.finger_slider_2.sliderPressed.connect(self.storeValue)
        self.finger_slider_3.sliderPressed.connect(self.storeValue)
        self.finger_slider_4.sliderPressed.connect(self.storeValue)
        self.finger_slider_5.sliderPressed.connect(self.storeValue)

        # Connect signal when slider has been released
        self.finger_slider_1.sliderReleased.connect(self.sliderRelease)
        self.finger_slider_2.sliderReleased.connect(self.sliderRelease)
        self.finger_slider_3.sliderReleased.connect(self.sliderRelease)
        self.finger_slider_4.sliderReleased.connect(self.sliderRelease)
        self.finger_slider_5.sliderReleased.connect(self.sliderRelease)

        # Add connect signal to Button Go, Cancel and Reset
        self.go_button.clicked.connect(self.handleButtonGo)
        self.home_button.clicked.connect(self.handleButtonHome)
        self.re_button.clicked.connect(self.handleButtonReset)
        #Add connect signal when combo box changes
        self.combo.currentIndexChanged.connect(self.handleHandSelectChange)

        self.list_control_save_button.clicked.connect(self.handle_list_control_save_button)
        self.list_control_delay_button.clicked.connect(self.handle_add_delay)
        self.list_control_execute_waypoints.clicked.connect(self.handle_execute_waypoints)
        self.list_control_save_grasp.clicked.connect(self.handle_grasp_save_button)
        self.list_control_delete_all.clicked.connect(self.handle_delete_all)
        self.file_execute_button.clicked.connect(self.handle_run_existing_grasp_button)
        self.file_load_button.clicked.connect(self.handle_add_file_waypoints)
        self.save_file_button.clicked.connect(self.handle_grasp_save_button)

        #incorporate the calibration widget if button is clicked
        self.calibrate_button.clicked.connect(self.handle_calibrate_widget)




######### Set up window ###################################################################################
        #Set the widget to layout and show the widget
        self.setLayout(self.fbox)

#############################################################################################################
#Actions for when you right click on a waypoint in the list
    def eventFilter(self,source,event):
        if (event.type() == QEvent.ContextMenu):
            menu = QMenu(self)
            #waypoint actions
            waypointMenu = menu.addMenu("Add Waypoint")
            addWaypointAbove = waypointMenu.addAction("Add Above")
            addWaypointBelow = waypointMenu.addAction("Add Below")
            #delay actions
            delayMenu = menu.addMenu("Add Delay")
            addDelayAbove = delayMenu.addAction("Add Above")
            addDelayBelow = delayMenu.addAction("Add Below")
            #general actions
            deleteWaypoint = menu.addAction("Delete")
            #execute actions
            action = menu.exec_(event.globalPos())
            slider_pose = self.getSliderPose()
            vel = self.getVelocity()
            des_cmd = Command(pose=slider_pose,velocity=vel)
            if action == addWaypointAbove:
                self.addWaypoint(is_below=False,cmd=des_cmd)
            if action == addWaypointBelow:
                self.addWaypoint(is_below=True,cmd=des_cmd)
            if action == addDelayAbove:
                delay = self.getDelay()
                if delay != False: #if the delay was not canceled during prompt
                    self.addWaypoint(is_below=False,cmd=delay)
            if action == addDelayBelow:
                delay = self.getDelay()
                if delay != False:
                    self.addWaypoint(is_below=True,cmd=delay)
            if action == deleteWaypoint:
                self.deleteWaypoint(self.listWidget.currentRow())

        return QWidget.eventFilter(self, source, event)

    #Popup window with calibration GUI for Reflex SF hand
    def handle_calibrate_widget(self):
        self.calibrate_window = CalibrationWidget()



#delete all stored waypoints
    def handle_delete_all(self):
        confirm_msg = "Are you sure you want to delete all waypoints?"
        response = QMessageBox.question(self,'Confirm Action', confirm_msg, QMessageBox.Yes, QMessageBox.No)
        if response == QMessageBox.Yes:
            while len(self.listCommand) > 0:
                self.deleteWaypoint(0)

    #Save Current slider pose to waypoint list
    def handle_list_control_save_button(self):
        slider_pose = self.getSliderPose()
        velocity0 = self.getVelocity()
        self.addWaypoint(is_below=-1, cmd=Command(pose=slider_pose,velocity=velocity0))#desired_pose=slider_pose)

    #Send waypoints from existing waypoint list to the robotic hand
    def handle_execute_waypoints(self):
        for command in self.listCommand:
            self.moveHandtoPose(command)

#############################################################################################################
    #Read waypoints from grasp file and send pose messages to robotic hand
    def handle_run_existing_grasp_button(self):
        fileData = self.readFile()
        for pose in fileData:
            self.moveHandtoPose(pose)

######### valuechange for updating goal label ###############################################################

    def sliderRelease(self):
        if(self.tick_f1_state == 1):
            self.handleButtonGo()

    def storeValue(self):
        self.f1_ref = self.finger_slider_1.value()
        self.f2_ref = self.finger_slider_2.value()
        self.f3_ref = self.finger_slider_3.value()
        self.f4_ref = self.finger_slider_4.value()
        self.f5_ref = self.finger_slider_5.value()


#Coupling of fingers, updating finger slider values
    def valuechange1(self):
        curr_value = float(self.finger_slider_1.value())
        float_value = curr_value/100.0
        diff = abs(self.f1_ref - curr_value)
        if((self.tick_update_state == 1) and (diff >= 10)):
            self.f1_ref = curr_value
            self.handleButtonGo()
        self.value_slider_1.setText("%2.2f" % float_value)
        if self.tick_f1_state:
            if self.tick_f2_state:
                self.value_slider_2.setText("%2.2f" % float_value)
                self.finger_slider_2.setValue(self.finger_slider_1.value())
            if self.tick_f3_state:
                self.value_slider_3.setText("%2.2f" % float_value)
                self.finger_slider_3.setValue(self.finger_slider_1.value())
            if self.tick_f4_state and self.tick_f4.isVisible():
                self.value_slider_4.setText("%2.2f" % float_value)
                self.finger_slider_4.setValue(self.finger_slider_1.value())


    def valuechange2(self):
        curr_value = float(self.finger_slider_2.value())
        float_value = curr_value/100.0
        diff = abs(self.f2_ref - curr_value)
        if((self.tick_update_state == 1) and (diff >= 10)):
            self.f2_ref = curr_value
            self.handleButtonGo()
        self.value_slider_2.setText("%2.2f" % float_value)
        if self.tick_f2_state:
            if self.tick_f1_state:
                self.value_slider_1.setText("%2.2f" % float_value)
                self.finger_slider_1.setValue(self.finger_slider_2.value())
            if self.tick_f3_state:
                self.value_slider_3.setText("%2.2f" % float_value)
                self.finger_slider_3.setValue(self.finger_slider_2.value())
            if self.tick_f4_state and self.tick_f4.isVisible():
                self.value_slider_4.setText("%2.2f" % float_value)
                self.finger_slider_4.setValue(self.finger_slider_2.value())


    def valuechange3(self):
        curr_value = float(self.finger_slider_3.value())
        float_value = curr_value/100.0
        diff = abs(self.f3_ref - curr_value)
        if((self.tick_update_state == 1) and (diff >= 10)):
            self.f3_ref = curr_value
            self.handleButtonGo()
        self.value_slider_3.setText("%2.2f" % float_value)
        if self.tick_f3_state:
            if self.tick_f1_state:
                self.value_slider_1.setText("%2.2f" % float_value)
                self.finger_slider_1.setValue(self.finger_slider_3.value())
            if self.tick_f2_state:
                self.value_slider_2.setText("%2.2f" % float_value)
                self.finger_slider_2.setValue(self.finger_slider_3.value())
            if self.tick_f4_state and self.tick_f4.isVisible():
                self.value_slider_4.setText("%2.2f" % float_value)
                self.finger_slider_4.setValue(self.finger_slider_3.value())

    def valuechange4(self):
        """This should only work for the softhand
        """
        curr_value = float(self.finger_slider_4.value())
        float_value = curr_value/100.0
        diff = abs(self.f4_ref - curr_value)
        if((self.tick_update_state == 1) and (diff >= 10)):
            self.f4_ref = curr_value
            self.handleButtonGo()
        self.value_slider_4.setText("%2.2f" % float_value)
        if self.tick_f4_state and self.tick_f4.isVisible():
            if self.tick_f1_state:
                self.value_slider_1.setText("%2.2f" % float_value)
                self.finger_slider_1.setValue(self.finger_slider_4.value())
            if self.tick_f2_state:
                self.value_slider_2.setText("%2.2f" % float_value)
                self.finger_slider_2.setValue(self.finger_slider_4.value())
            if self.tick_f3_state:
                self.value_slider_3.setText("%2.2f" % float_value)
                self.finger_slider_3.setValue(self.finger_slider_4.value())

    def valuechange5(self):
        curr_value = float(self.finger_slider_5.value())
        float_value = curr_value/100.0
        diff = abs(self.f5_ref - curr_value)
        if((self.tick_update_state == 1) and (diff >= 10)):
            self.f5_ref = curr_value
            self.handleButtonGo()
        self.value_slider_5.setText("%2.2f" % float_value)

#########################################1####################################################################
    def tickchange(self,b):
        if b.text() == "F1":
            if b.isChecked():
                self.tick_f1_state = 1
            else:
                self.tick_f1_state = 0
        if b.text() == "F2":
            if b.isChecked():
                self.tick_f2_state = 1
            else:
                self.tick_f2_state = 0
        if b.text() == "F3":
            if b.isChecked():
                self.tick_f3_state = 1
            else:
                self.tick_f3_state = 0
        if b.text() == "F4":
            if b.isChecked():
                self.tick_f4_state = 1
            else:
                self.tick_f4_state = 0
        if b.text() == "Turn on live update":
            if b.isChecked():
                self.tick_update_state = 1
            else:
                self.tick_update_state = 0
######### Command Button handler ############################################################################
    #Send current slider position to the robotic hand
    def handleButtonGo(self):
        #send current slider values to hand
        curr_command = self.getCurrCommand()
        self.moveHandtoPose(curr_command)


    def handleHandSelectChange(self):
        """Change the UI labels accordingly with the selected robot hand.

        """
        if self.combo.currentText() == "ReflexSF":
            self.finger_label_4.setText("Distance between fingers 1 and 2")
            self.finger_label_5.setText("Thumb Rotation")
            self.tick_f4.setHidden(True)
            self.finger_label_5.setHidden(False)
            self.finger_slider_5.setHidden(False)
            self.value_slider_5.setHidden(False)

            self.calibrate_button.setHidden(False)
        elif self.combo.currentText() == "Soft Hand":
            self.finger_label_4.setText("Goal for F4")
            self.tick_f4.setHidden(False)
            self.finger_label_5.setHidden(True)
            self.finger_slider_5.setHidden(True)
            self.value_slider_5.setHidden(True)

            self.calibrate_button.setHidden(True)

    #Reset fingers to home positions
    def handleButtonHome(self):
        pose = PoseCommand(f1=0.0,f2=0.0,f3=0.0,k1=0.0,k2=1.0)
        vel = VelocityCommand(f1=0.05,f2=0.05,f3=0.05,k1=0.05,k2=0.05)
        home_command = Command(pose=pose,velocity=vel)
        self.moveHandtoPose(home_command)


    #Reset slider values to home positions
    def handleButtonReset(self):
        self.finger_slider_1.setValue(0)
        self.value_slider_1.setText("0.00")
        self.finger_slider_2.setValue(0)
        self.value_slider_2.setText("0.00")
        self.finger_slider_3.setValue(0)
        self.value_slider_3.setText("0.00")
        self.finger_slider_4.setValue(0)
        self.value_slider_4.setText("0.00")
        self.finger_slider_5.setValue(1)
        self.value_slider_5.setText("0.00")

    #Read waypoints from file and add to current waypoint list
    def handle_add_file_waypoints(self):
        # read selected file
        file_name = self.fileListWidget.currentItem().text()
        fileData = file.readFile(file_name)
        if(fileData is not None):
            for pose in fileData:
                self.addWaypoint(is_below=-1,cmd=pose)

    #Add Delay to waypoint list
    def handle_add_delay(self):
        delay = self.getDelay()
        if delay == False:
            return
        else:
            self.addWaypoint(is_below=-1,cmd=delay)


    def handle_grasp_save_button(self):
        file.save_file(self.listCommand)
        # #prompt to edit filename/path
        # filepath = QFileDialog.getSaveFileName(self, 'Save File', FILE_DIR)[0]
        # name = os.path.basename(filepath)
        # #write waypoint list to file
        # if len(self.listCommand) > 0:
        #     print 'saved ' + str(len(self.listCommand)) + ' waypoints to ' + name
        #     with open(filepath, 'w') as file:
        #         for point in self.listCommand:
        #             #add indicator for each chunk of data
        #             data = '//' + str(point.pose) + "***" + str(point.velocity)
        #             file.write(data)
        # else:
        #     print "No waypoints to save"
        #
        # self.filename.append(name)
        # item = QListWidgetItem(name)
        # self.fileListWidget.addItem(item)

    #Send desired pose to the soft robotic hand
    def softHand_pose(self,f1,f2,f3,f4):
        f1 = 100 if int(f1) > 100 else int(f1)
        f2 = 100 if int(f2) > 100 else int(f2)
        f3 = 100 if int(f3) > 100 else int(f3)
        f4 = 100 if int(f4) > 100 else int(f4)
        self.command_pub_softhand_1.publish(f1)
        self.command_pub_softhand_2.publish(f2)
        self.command_pub_softhand_3.publish(f3)
        self.command_pub_softhand_4.publish(f4)




########### Load File   ############################################################################
    #check data directory and display available grasp files to run
    def populate_filelist(self):
        file.populate_filelist()
        # all_files = [f for f in listdir(FILE_DIR) if isfile(join(FILE_DIR, f))]
        # for f in all_files:
        #     self.fileListWidget.addItem(QListWidgetItem(f))
        #     self.filename.append(f)

    def is_delay(self, pose):
        f1 = pose.f1 == 999
        f2 = pose.f2 == 999
        f3 = pose.f3 == 999
        k1 = pose.k1 == 999
        return f1 and f2 and f3 and k1

    def populate_commandlist(self):
        count = 1
        self.listWidget.clear()
        for command in self.listCommand:
            if not self.is_delay(command.pose):
                pose = command.pose
                vel = command.velocity
                pose_str = "Pose: [  %2.2f,  %2.2f,  %2.2f,  %2.2f,  %2.2f  ]"%(pose.f1,pose.f2,pose.f3,pose.k1,pose.k2-1)
                velocity_str = "Velocity: [  %2.2f,  %2.2f,  %2.2f,  %2.2f,  %2.2f  ]"%(vel.f1,vel.f2,vel.f3,vel.k1,vel.k2)
                item = QListWidgetItem("%d.  %s %s"%(count, pose_str, velocity_str))
            else:
                item = QListWidgetItem("%d. %.2f second delay" % (count, command.pose.k2))
            self.listWidget.addItem(item)
            count += 1


    def deleteWaypoint(self, point_sel):
        if (self.listCommand != []):
            if (point_sel < 0):
                error_msg1 = QErrorMessage(self)
                error_msg1.setWindowTitle("Waypoint Error")
                error_msg1.showMessage("Please select a valid waypoint to remove")
            else:
                self.listCommand.pop(point_sel)
                self.populate_commandlist()
        else:
            error_msg2 = QErrorMessage(self)
            error_msg2.setWindowTitle("Waypoint Error")
            error_msg2.showMessage("Could not remove waypoint: \nNo waypoints found")



    def addWaypoint(self, is_below, cmd):
        # is below = true if waypoint is desired to be below selected row, above if desired to be above, -1 if desired at end of list
        if is_below < 0:
            self.listCommand.append(cmd)
            self.populate_commandlist()
        else:
            list_location = int(self.listWidget.currentRow()) + is_below
            self.listCommand.insert(list_location, cmd)
            self.populate_commandlist()




    #Read a Grasp File and return list of poses
    def readFile(self):
        return file.readFile()
        pose_list = []
        try:
            # read selected file
            file_name = self.fileListWidget.currentItem().text()
            file_path = "{}/{}".format(FILE_DIR, file_name)
            file = open(file_path,'r').read()
            # Divide file into poses
            data_chunks = file.split('//')
            data_chunks.pop(0) #the file gets saved with an extra // trigger
            for command in data_chunks:
                p_or_v = command.split('***')
                cnt = 0
                for test in p_or_v:
                    # divide each pose up by commands per finger
                    f1,f2,f3,k1,k2 = test.split('\n')
                    # choose only the numerical chunk of command and convert to float
                    tar_f1 = float(f1.split(': ')[1])
                    tar_f2 = float(f2.split(': ')[1])
                    tar_f3 = float(f3.split(': ')[1])
                    tar_k1 = float(k1.split(': ')[1])
                    tar_k2 = float(k2.split(': ')[1])
                    if cnt == 0:
                        pose0 = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_k1,k2=tar_k2)
                        cnt = cnt + 1
                    else:
                        velocity0 = VelocityCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_k1,k2=tar_k2)

                cmd = Command(pose=pose0,velocity=velocity0)
                pose_list.append(cmd)
            return pose_list
        except AttributeError:
            error_msg = QErrorMessage(self)
            error_msg.setWindowTitle("File Error")
            error_msg.showMessage("Please Select A File to Execute")

        except Exception:
            error_msg2 = QErrorMessage(self)
            error_msg2.setWindowTitle("File Error")
            error_msg2.showMessage("Could not load file")


#Send robot hand to desired position
    def moveHandtoPose(self, command):
        poseTarget = command.pose
        if self.is_delay(poseTarget):
            rospy.sleep(poseTarget.k2)
        else:
            if self.combo.currentText() == "ReflexSF":
                self.command_pub.publish(command)
            elif self.combo.currentText() == "Soft Hand":
                self.softHand_pose(f1=poseTarget.f1*50,f2=poseTarget.f2*50,f3=poseTarget.f3*50,f4=poseTarget.k1*50)
            rospy.sleep(0.2)

    def getCurrCommand(self):
        pose = self.getSliderPose()
        vel = self.getVelocity()
        return Command(pose=pose,velocity=vel)
    #return current pose with respect to slider values
    def getSliderPose(self):
        # record slider values
        float_value_1 = float(self.value_slider_1.text())
        float_value_2 = float(self.value_slider_2.text())
        float_value_3 = float(self.value_slider_3.text())
        float_value_4 = float(self.value_slider_4.text())
        float_value_5 = float(self.value_slider_5.text())
        pose0 = PoseCommand(f1=float_value_1,f2=float_value_2,f3=float_value_3,k1=float_value_4,k2=1+float_value_5)
        return pose0

    def getVelocity(self):
        val1 = float(self.f1_speed.value())
        val2 = float(self.f2_speed.value())
        val3 = float(self.f3_speed.value())
        val4 = float(self.f4_speed.value())
        val5 = float(self.f5_speed.value())
        velocity = VelocityCommand(f1=val1,f2=val2,f3=val3,k1=val4,k2=val5)
        return velocity

    #prompt for delay input
    def getDelay(self):
        num,ok = QInputDialog.getDouble(self,"Delay", "Delay in seconds")
        if ok:
          pose0 = PoseCommand(f1=999,f2=999,f3=999,k1=999,k2=num)
          vel0 = VelocityCommand(f1=999,f2=999,f3=999,k1=999,k2=num)
          return Command(pose=pose0,velocity=vel0)
        else:
          return False
