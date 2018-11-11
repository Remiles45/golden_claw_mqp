import os
import sys
import rospkg
import rospy

from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from PyQt5.QtCore import QEvent, Qt, QTimeLine, QTimer
from reflex_msgs.msg import PoseCommand, Command, VelocityCommand
from std_msgs.msg import UInt16
from os import listdir
from os.path import isfile, join
from calibration_widget import CalibrationWidget
from file_wr import FileWriteRead
from slider import Slider


file = FileWriteRead()
rospack = rospkg.RosPack()
FILE_DIR = rospack.get_path('rqt_gui_control') + '/data'
class ManualHandControlWidget(QWidget):


    def __init__(self):
        self.rate = rospy.Rate(2)
        super(ManualHandControlWidget, self).__init__()
        self.filename = []
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



    def initUI(self):

################## Position Control GUI ########################################################################
######### Automatic Hand Move ###################################################################################
        #TODO: this may happen too fast? need to test on hand, make smooth like buddah
        #Synchronize selected fingers
        self.hbox_update_tick = QHBoxLayout()
        self.live_update = QCheckBox("Turn on live update")
        self.hbox_update_tick.addWidget(self.live_update)
        self.hbox_update_tick.addStretch()

#Fingers: slider range 0 -> 200
        #Finger 1 row (F1)
        self.finger_label_1 = QLabel("Goal for f1")
        self.finger_slider_1 = QSlider(1)
        self.finger_slider_1.setMinimum(0)
        self.finger_slider_1.setMaximum(200)
        self.f1_ref = 0.00
        self.finger_slider_1.setValue(self.f1_ref)

        self.value_slider_1 = QLabel(str(self.f1_ref))
        self.value_slider_1.setMaximumSize(80,20)
        self.f1_speed = QDoubleSpinBox()
        self.f1_speed.setSingleStep(0.1)
        self.f1_speed.setValue(0.5)
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
        self.f2_speed.setSingleStep(0.1)
        self.f2_speed.setValue(0.5)
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
        self.f3_speed.setSingleStep(0.1)
        self.f3_speed.setValue(0.5)
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
        self.f4_speed.setSingleStep(0.1)
        self.f4_speed.setValue(0.5)
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
        self.f5_speed_lbl = QLabel("Rad/Sec")
        self.f5_speed = QDoubleSpinBox()
        self.f5_speed.setSingleStep(0.1)
        self.f5_speed.setValue(0.5)
        self.hbox_f5 = QHBoxLayout()
        self.hbox_f5.addWidget(self.finger_slider_5)
        self.hbox_f5.addWidget(self.value_slider_5)
        self.hbox_f5.addWidget(self.f5_speed)
        self.hbox_f5.addWidget(self.f5_speed_lbl)


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
        self.fileListWidget = QListWidget()
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
        self.fbox.addRow(self.fileslabel, self.fileListWidget)
        self.fbox.addRow(QLabel(""), self.file_control)
        self.fbox.addRow(QLabel(""), self.calibration)
        self.fbox.addRow(self.combo_label,self.combo)

        # Connect signal when slider change to function respectively to change value of label
        self.finger_slider_1.valueChanged.connect(lambda:self.sliderMoved(1))
        self.finger_slider_2.valueChanged.connect(lambda:self.sliderMoved(2))
        self.finger_slider_3.valueChanged.connect(lambda:self.sliderMoved(3))
        self.finger_slider_4.valueChanged.connect(lambda:self.sliderMoved(4))
        self.finger_slider_5.valueChanged.connect(lambda:self.sliderMoved(5))

        # connect slider to measure distance scrolled
        self.finger_slider_1.sliderPressed.connect(self.updateRefs)
        self.finger_slider_2.sliderPressed.connect(self.updateRefs)
        self.finger_slider_3.sliderPressed.connect(self.updateRefs)
        self.finger_slider_4.sliderPressed.connect(self.updateRefs)
        self.finger_slider_5.sliderPressed.connect(self.updateRefs)

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
        self.calibrate_button.clicked.connect(self.handle_calibrate_widget)

######### Set up window ###################################################################################
        #Set the widget to layout and show the widget
        self.setLayout(self.fbox)

######### Event Filter ###################################################################################
    def eventFilter(self,source,event):
        """
            Add actions to context menu on waypoint items
        """
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

########## UI Controllers ############################################################################
    def handle_add_delay(self):
        """
            Add a delay to the waypoint list
        """
        delay = self.getDelay()
        if delay == False:
            return
        else:
            self.addWaypoint(is_below=-1,cmd=delay)

    def handle_add_file_waypoints(self):
        """
            Read a selected file and add its contents to the waypoint list
        """
        file_name = self.fileListWidget.currentItem().text()
        fileData = file.readFile(file_name)
        if(fileData is not None):
            for pose in fileData:
                self.addWaypoint(is_below=-1,cmd=pose)

    def handleButtonGo(self):
        """
            Send current slider position to the robotic hand
        """
        curr_command = self.getCurrCommand()
        self.moveHandtoPose(curr_command)

    def handleButtonHome(self):
        """
            Reset fingers to home positions
        """
        pose = PoseCommand(f1=0.0,f2=0.0,f3=0.0,k1=0.0,k2=1.0)
        vel = VelocityCommand(f1=1,f2=1,f3=1,k1=1,k2=1)
        home_command = Command(pose=pose,velocity=vel)
        self.moveHandtoPose(home_command)

    def handleButtonReset(self):
        """
            Reset slider values to home positions
        """
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

    def handle_calibrate_widget(self):
        """
            Popup window with calibration GUI for Reflex SF hand
        """
        self.calibrate_window = CalibrationWidget()

    def handle_delete_all(self):
        """
            delete all stored waypoints
        """
        confirm_msg = "Are you sure you want to delete all waypoints?"
        response = QMessageBox.question(self,'Confirm Action', confirm_msg, QMessageBox.Yes, QMessageBox.No)
        if response == QMessageBox.Yes:
            while len(self.listCommand) > 0:
                self.deleteWaypoint(0)

    def handle_execute_waypoints(self):
        """
            Send waypoints from existing waypoint list to the robotic hand
        """
        for command in self.listCommand:
            self.moveHandtoPose(command)

    def handle_grasp_save_button(self):
        """
            save waypoint list to a new file
        """
        name = file.save_file(self.listCommand)
        if name is not False:
            self.filename.append(name)
            item = QListWidgetItem(name)
            self.fileListWidget.addItem(item)

    def handleHandSelectChange(self):
        """
            Change the UI labels accordingly with the selected robot hand.
        """
        if self.combo.currentText() == "ReflexSF":
            self.finger_label_4.setText("Distance between fingers 1 and 2")
            self.finger_label_5.setText("Thumb Rotation")
            self.tick_f4.setHidden(True)
            self.finger_label_5.setHidden(False)
            self.finger_slider_5.setHidden(False)
            self.value_slider_5.setHidden(False)
            self.f5_speed.setHidden(False)
            self.f5_speed_lbl.setHidden(False)
            self.calibrate_button.setHidden(False)
        elif self.combo.currentText() == "Soft Hand":
            self.finger_label_4.setText("Goal for F4")
            self.tick_f4.setHidden(False)
            self.finger_label_5.setHidden(True)
            self.finger_slider_5.setHidden(True)
            self.value_slider_5.setHidden(True)
            self.calibrate_button.setHidden(True)
            self.f5_speed.setHidden(True)
            self.f5_speed_lbl.setHidden(True)

    def handle_list_control_save_button(self):
        """
            Save Current slider pose to waypoint list
        """
        slider_pose = self.getSliderPose()
        velocity0 = self.getVelocity()
        self.addWaypoint(is_below=-1, cmd=Command(pose=slider_pose,velocity=velocity0))

    def handle_run_existing_grasp_button(self):
        """
            Read waypoints from grasp file and send pose messages to robotic hand
        """
        fileData = self.readFile()
        for pose in fileData:
            self.moveHandtoPose(pose)

######### Finger Slider Handlers ###############################################################
    def sliderRelease(self):
        """
            When live update is activated, send the hand to the current slider
            pose when the user releases the click on the slider
        """
        if(self.live_update.isChecked()):
            self.handleButtonGo()

    # def storeValue(self):
    #     """
    #         Record what the last slider value was for reference
    #     """
    #     self.f1_ref = self.finger_slider_1.value()
    #     self.f2_ref = self.finger_slider_2.value()
    #     self.f3_ref = self.finger_slider_3.value()
    #     self.f4_ref = self.finger_slider_4.value()
    #     self.f5_ref = self.finger_slider_5.value()

    def sliderMoved(self,active_slider):
        """
            update slider values and label texts according to slider coupling
        """
        alt_slider_list = []
        slide1 = Slider(slider=self.finger_slider_1,label_val=self.value_slider_1,ref=self.f1_ref,tick=self.tick_f1.isChecked())
        slide2 = Slider(slider=self.finger_slider_2,label_val=self.value_slider_2,ref=self.f2_ref,tick=self.tick_f2.isChecked())
        slide3 = Slider(slider=self.finger_slider_3,label_val=self.value_slider_3,ref=self.f3_ref,tick=self.tick_f3.isChecked())
        slide4_tick = self.tick_f4.isChecked() if self.tick_f4.isVisible() else False
        slide4 = Slider(slider=self.finger_slider_4,label_val=self.value_slider_4,ref=self.f4_ref,tick=slide4_tick)
        slide5 = Slider(slider=self.finger_slider_5,label_val=self.value_slider_5,ref=self.f5_ref,tick=False)
        if active_slider == 1:
            this_slider = slide1
            alt_slider_list = [slide2,slide3,slide4]#slider 5 cant be coupled
        elif active_slider == 2:
            this_slider = slide2
            alt_slider_list = [slide1,slide3,slide4]#slider 5 cant be coupled
        elif active_slider == 3:
            this_slider = slide3
            alt_slider_list = [slide1,slide2,slide4]#slider 5 cant be coupled
        elif active_slider == 4:
            this_slider = slide4
            alt_slider_list = [slide1,slide2,slide3]#slider 5 cant be coupled
        elif active_slider == 5:
            this_slider = slide5

        curr_value = float(this_slider.getVal())
        float_value = curr_value/100.0
        diff = abs(this_slider.getRef() - curr_value)
        this_slider.setLabel("%2.2f" % float_value)
        if this_slider.getTickCheck():
            for slide in alt_slider_list:
                if slide.getTickCheck():
                    slide.setLabel("%2.2f" % float_value)
                    slide.setSlider(this_slider.getVal())

        if(self.live_update.isChecked() and (diff >= 10)):
            self.handleButtonGo()
            self.updateRefs()


    def updateRefs(self):
        self.f1_ref = self.finger_slider_1.value()
        self.f2_ref = self.finger_slider_2.value()
        self.f3_ref = self.finger_slider_3.value()
        self.f4_ref = self.finger_slider_4.value()
        self.f5_ref = self.finger_slider_5.value()

######## Manage Waypoint List ##################################################################################
    def populate_commandlist(self):
        """
            display all of the commands in the waypoint list
        """
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
        """
            delete waypoint from memory and from displayed list
        """
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
        """
            Add the current slider and velocity values to the waypoint list
        """
        # is below = true if waypoint is desired to be below selected row, above if desired to be above, -1 if desired at end of list
        if is_below < 0:
            self.listCommand.append(cmd)
            self.populate_commandlist()
        else:
            list_location = int(self.listWidget.currentRow()) + is_below
            self.listCommand.insert(list_location, cmd)
            self.populate_commandlist()

    def is_delay(self, pose):
        """
            check if a waypoint is an actual pose or a delay
        """
        f1 = pose.f1 == 999
        f2 = pose.f2 == 999
        f3 = pose.f3 == 999
        k1 = pose.k1 == 999
        return f1 and f2 and f3 and k1

######## Manage file write read ##################################################################################
    def populate_filelist(self):
        """
            check data directory and display available grasp files to run
        """
        all_files = [f for f in listdir(FILE_DIR) if isfile(join(FILE_DIR, f))]
        for f in all_files:
            self.fileListWidget.addItem(QListWidgetItem(f))
            self.filename.append(f)

    def readFile(self):
        """
            Read a Grasp File and return list of poses
        """
        pose_list = []
        file_name = self.fileListWidget.currentItem().text()
        pose_list = file.readFile(file_name)
        if pose_list is not False:
            return pose_list

######## Physically move Robot Hand ##################################################################################
    def moveHandtoPose(self, command):
        """
            Publish command message to physically move fingers to position
        """
        poseTarget = command.pose
        if self.is_delay(poseTarget):
            rospy.sleep(poseTarget.k2)
        else:
            if self.combo.currentText() == "ReflexSF":
                self.command_pub.publish(command)
            elif self.combo.currentText() == "Soft Hand":
                self.softHand_pose(f1=poseTarget.f1*50,f2=poseTarget.f2*50,f3=poseTarget.f3*50,f4=poseTarget.k1*50)
            rospy.sleep(0.2)

    def softHand_pose(self,f1,f2,f3,f4):
        """
            Send desired pose to the soft robotic hand
        """
        f1 = 100 if int(f1) > 100 else int(f1)
        f2 = 100 if int(f2) > 100 else int(f2)
        f3 = 100 if int(f3) > 100 else int(f3)
        f4 = 100 if int(f4) > 100 else int(f4)
        self.command_pub_softhand_1.publish(f1)
        self.command_pub_softhand_2.publish(f2)
        self.command_pub_softhand_3.publish(f3)
        self.command_pub_softhand_4.publish(f4)
######## Retrieve user inputs from UI ##################################################################################
    def getCurrCommand(self):
        """
            Combine the pose and velocity inputs into a single command
        """
        pose = self.getSliderPose()
        vel = self.getVelocity()
        return Command(pose=pose,velocity=vel)

    def getSliderPose(self):
        """
            return the current pose of the slider bars
        """
        float_value_1 = float(self.value_slider_1.text())
        float_value_2 = float(self.value_slider_2.text())
        float_value_3 = float(self.value_slider_3.text())
        float_value_4 = float(self.value_slider_4.text())
        float_value_5 = float(self.value_slider_5.text())
        pose0 = PoseCommand(f1=float_value_1,f2=float_value_2,f3=float_value_3,k1=float_value_4,k2=1+float_value_5)
        return pose0

    def getVelocity(self):
        """
            Return the current value of the velocity input box
        """
        val1 = float(self.f1_speed.value())
        val2 = float(self.f2_speed.value())
        val3 = float(self.f3_speed.value())
        val4 = float(self.f4_speed.value())
        val5 = float(self.f5_speed.value())
        velocity = VelocityCommand(f1=val1,f2=val2,f3=val3,k1=val4,k2=val5)
        return velocity

    def getDelay(self):
        """
            Prompt for user input to set a delay
        """
        num,ok = QInputDialog.getDouble(self,"Delay", "Delay in seconds")
        if ok:
          pose0 = PoseCommand(f1=999,f2=999,f3=999,k1=999,k2=num)
          vel0 = VelocityCommand(f1=999,f2=999,f3=999,k1=999,k2=num)
          return Command(pose=pose0,velocity=vel0)
        else:
          return False
