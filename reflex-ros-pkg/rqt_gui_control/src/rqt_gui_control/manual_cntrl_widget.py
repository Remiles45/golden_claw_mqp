import os
import rospkg
import rospy
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from PyQt5.QtCore import QEvent, Qt, QTimeLine, QTimer
from std_msgs.msg import Int16MultiArray
from reflex_msgs.msg import PoseCommand
from std_msgs.msg import UInt16
from os import listdir
from os.path import isfile, join



rospack = rospkg.RosPack()
FILE_DIR = rospack.get_path('rqt_gui_control') + '/data'
class ManualHandControlWidget(QWidget):


    def __init__(self):
        super(ManualHandControlWidget, self).__init__()
        #Reflex SF hand control
        self.command_pub = rospy.Publisher('/reflex_sf/command_position', PoseCommand, queue_size=1)
        #Data glove
        rospy.Subscriber('/chatter',Int16MultiArray, self.received_int)####FIXME
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
#Fingers: slider range 0 -> 200
        #Finger 1 row (F1)
        self.finger_label_1 = QLabel("Goal for f1")
        self.finger_slider_1 = QSlider(1)
        self.finger_slider_1.setMinimum(0)
        self.finger_slider_1.setMaximum(200)
        self.finger_slider_1.setValue(0)

        self.value_slider_1 = QTextEdit("0.00")
        self.value_slider_1.setMaximumSize(80,20)
        self.hbox_f1 = QHBoxLayout()
        self.hbox_f1.addWidget(self.finger_slider_1)
        self.hbox_f1.addWidget(self.value_slider_1)

        #Finger 2 row (F2)
        self.finger_label_2 = QLabel("Goal for f2")
        self.finger_slider_2 = QSlider(1)
        self.finger_slider_2.setMinimum(0)
        self.finger_slider_2.setMaximum(200)
        self.finger_slider_2.setValue(0)

        self.value_slider_2 = QTextEdit("0.00")
        self.value_slider_2.setMaximumSize(80,20)
        self.hbox_f2 = QHBoxLayout()
        self.hbox_f2.addWidget(self.finger_slider_2)
        self.hbox_f2.addWidget(self.value_slider_2)

        #Finger 3 row (F3)
        self.finger_label_3 = QLabel("Goal for f3")
        self.finger_slider_3 = QSlider(1)
        self.finger_slider_3.setMinimum(0)
        self.finger_slider_3.setMaximum(200)
        self.finger_slider_3.setValue(0)

        self.value_slider_3 = QTextEdit("0.00")
        self.value_slider_3.setMaximumSize(80,20)
        self.hbox_f3 = QHBoxLayout()
        self.hbox_f3.addWidget(self.finger_slider_3)
        self.hbox_f3.addWidget(self.value_slider_3)


        #Reflex SF hand distance between fingers (k1)
        self.finger_label_4 = QLabel("Distance between fingers 1 and 2")
        self.finger_slider_4 = QSlider(1)
        self.finger_slider_4.setMinimum(0)
        self.finger_slider_4.setMaximum(200)
        self.finger_slider_4.setValue(0)
        #initialize at position 0
        self.value_slider_4 = QTextEdit("0.00")
        self.value_slider_4.setMaximumSize(80,20)
        self.hbox_f4 = QHBoxLayout()
        self.hbox_f4.addWidget(self.finger_slider_4)
        self.hbox_f4.addWidget(self.value_slider_4)


        #Reflex SF hand thumb rotation (k2)
        #set range to -100 to 100 since it is easier for user to conceptualize
        self.finger_label_5 = QLabel("Thumb Rotation")
        self.finger_slider_5 = QSlider(1)
        self.finger_slider_5.setMinimum(-100)
        self.finger_slider_5.setMaximum(100)
        self.finger_slider_5.setValue(0)
        #set initial value to center position
        self.value_slider_5 = QTextEdit("0.00")
        self.value_slider_5.setMaximumSize(80,20)
        self.hbox_f5 = QHBoxLayout()
        self.hbox_f5.addWidget(self.finger_slider_5)
        self.hbox_f5.addWidget(self.value_slider_5)

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
############ Glove Section #############################################################################
#TODO: fix basically everything related to the glove
        self.glove_label = QLabel("Glove Interface")
        self.tick_glove = QCheckBox("ON/OFF")
        self.tick_glove_state = 0
        self.tick_glove.stateChanged.connect(lambda:self.tickchange(self.tick_glove))

        self.value_glove_1 = QLabel("x")
        self.value_glove_1.setMaximumSize(80,20)

        self.value_glove_2 = QLabel("x")
        self.value_glove_2.setMaximumSize(80,20)

        self.value_glove_3 = QLabel("x")
        self.value_glove_3.setMaximumSize(80,20)

        self.value_glove_4 = QLabel("x")
        self.value_glove_4.setMaximumSize(80,20)

        self.hbox_glove = QHBoxLayout()
        self.hbox_glove.addWidget(self.tick_glove)
        self.hbox_glove.addWidget(self.value_glove_1)
        self.hbox_glove.addWidget(self.value_glove_2)
        self.hbox_glove.addWidget(self.value_glove_3)
        self.hbox_glove.addWidget(self.value_glove_4)

############ Manage Waypoint List ##############################################################################################
        self.listPose = []
        self.grasplist = []
        self.filename = []
        #initial point at conceptual home position (thumb in center position)
        pose0 = PoseCommand(f1=0.0,f2=0.0,f3=0.0,k1=0.0,k2=1.0)
        self.listPose.append(pose0)
        #Display waypoints and files
        self.listWidget = QListWidget()
        self.fileListWidget = QListWidget()
        self.populate_poselist()
        self.listWidget.installEventFilter(self)#######################################################3
        self.populate_filelist()

        self.fileslabel = QLabel("Grasp Files")
        self.listlabel = QLabel("List waypoint")

        # Options for File List
        self.file_control = QHBoxLayout()
        self.file_load_button = QPushButton("Load File into Waypoint List")
        self.file_execute_button = QPushButton("Execute File")
        self.file_control.addWidget(self.file_load_button)
        self.file_control.addWidget(self.file_execute_button)

        #List Control
        self.list_control_label = QLabel("Waypoint Control")
        self.list_control_save_button = QPushButton("Add Waypoint")
        self.list_control_delay_button = QPushButton("Add Delay")
        self.list_control_execute_waypoints = QPushButton("Execute Waypoints")
        self.list_control_save_grasp = QPushButton("Save Grasp")
        self.list_control = QHBoxLayout()
        self.list_control.addWidget(self.list_control_save_button)
        self.list_control.addWidget(self.list_control_delay_button)
        self.list_control.addWidget(self.list_control_execute_waypoints)
        self.list_control.addWidget(self.list_control_save_grasp)

############ Adding Sections to GUI ####################################################
#using the buttons defined above to create the GUI itself
        self.fbox = QFormLayout()
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
        self.fbox.addRow(self.combo_label,self.combo)
        self.fbox.addRow(self.glove_label,self.hbox_glove)

        # Connect signal when slider change to function respectively to change value of label
        self.finger_slider_1.valueChanged.connect(self.valuechange1)
        self.finger_slider_2.valueChanged.connect(self.valuechange2)
        self.finger_slider_3.valueChanged.connect(self.valuechange3)
        self.finger_slider_4.valueChanged.connect(self.valuechange4)
        self.finger_slider_5.valueChanged.connect(self.valuechange5)
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
        self.file_execute_button.clicked.connect(self.handle_run_existing_grasp_button)
        self.file_load_button.clicked.connect(self.handle_add_file_waypoints)

######### Set up window ###################################################################################
        #Set the widget to layout and show the widget
        self.setLayout(self.fbox)

        self.setWindowTitle("Manual Hand Controller")
        self.resize(640,480)
        self.dumbnum = 0
        self.show()
        self.current_angle = [0.0,0.0,0.0,0.0]

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
            if action == addWaypointAbove:
                self.addWaypoint(is_below=False,desired_pose=slider_pose)
            if action == addWaypointBelow:
                self.addWaypoint(is_below=True,desired_pose=slider_pose)
            if action == addDelayAbove:
                delay = self.getDelay()
                if delay != False: #if the delay was not canceled during prompt
                    self.addWaypoint(is_below=False,desired_pose=delay)
            if action == addDelayBelow:
                delay = self.getDelay()
                if delay != False:
                    self.addWaypoint(is_below=True,desired_pose=delay)
            if action == deleteWaypoint:
                self.deleteWaypoint()

        return QWidget.eventFilter(self, source, event)



    #Save Current slider pose to waypoint list
    def handle_list_control_save_button(self):
        slider_pose = self.getSliderPose()
        self.addWaypoint(is_below=-1, desired_pose=slider_pose)

    #Send waypoints from existing waypoint list to the robotic hand
    #TODO: rework glove interface -- add record/stop recording button to save live grasp data to file
    #wait..... why is glove control in execute waypoints????? shouldnt it be in save? or totally separate?
    def handle_execute_waypoints(self):
        scaled_float_1 = 1.0
        scaled_float_2 = 1.0
        scaled_float_3 = 1.0

        if (self.tick_glove_state == 1):
            #needs to be reworked
            self.value_glove_1.setText("%2.2f" % scaled_float_1)
            self.value_glove_2.setText("%2.2f" % scaled_float_2)
            self.value_glove_3.setText("%2.2f" % scaled_float_3)
            data = str(scaled_float_1) + ";" +str(scaled_float_2) + ";" + str(scaled_float_3) + "\n"
            filename = "data/grasp1.txt"# I dont think this should be here?
            file = open(filename, "a")
            file.write(data)
            file.close()
        else:
            for pose in self.listPose:
                self.moveHandtoPose(f1=pose.f1, f2=pose.f2, f3=pose.f3, k1=pose.k1, k2=pose.k2)

#############################################################################################################
    #Save waypoint list to a grasp file
    def handle_grasp_save_button(self):
        filepath = QFileDialog.getSaveFileName(self, 'Save File', FILE_DIR)[0]
        name = os.path.basename(filepath)
        #write waypoint list to file
        if not filepath:
            return
        if self.listPose:
            print 'saved ' + str(len(self.listPose)) + ' waypoints to ' + name
            with open(filepath, 'w') as file:
                for point in self.listPose:
                    #add indicator for each chunk of data
                    data = "//" + str(point)
                    file.write(data)
        else:
            error_msg = QErrorMessage(self)
            error_msg.setWindowTitle("File Save Error")
            error_msg.showMessage("No waypoints to save")

        self.filename.append(name)
        item = QListWidgetItem(name)
        self.fileListWidget.addItem(item)

    #Read waypoints from grasp file and send pose messages to robotic hand
    def handle_run_existing_grasp_button(self):
        fileData = self.readFile()
        for pose in fileData:
            self.moveHandtoPose(f1=pose.f1,f2=pose.f2,f3=pose.f3,k1=pose.k1,k2=pose.k2)


######### valuechange for updating goal label ###############################################################
#Coupling of fingers, updating finger slider values
    def valuechange1(self):
        float_value = float(self.finger_slider_1.value())/100.0
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
        float_value = float(self.finger_slider_2.value())/100.0
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
        float_value = float(self.finger_slider_3.value())/100.0
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
        float_value = float(self.finger_slider_4.value())/100.0
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
        float_value = float(self.finger_slider_5.value())/100.0
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
        if b.text() == "ON/OFF":
            if b.isChecked() == True:
                self.tick_glove_state = 1
            else:
                self.tick_glove_state = 0
                self.value_glove_1.setText("x")
                self.value_glove_2.setText("x")
                self.value_glove_3.setText("x")

######### Command Button handler ############################################################################
    #Send current slider position to the robotic hand
    def handleButtonGo(self):
        #send current slider values to hand
        tar_f1 = float(self.value_slider_1.toPlainText())
        tar_f2 = float(self.value_slider_2.toPlainText())
        tar_f3 = float(self.value_slider_3.toPlainText())
        tar_k1 = float(self.value_slider_4.toPlainText())
        tar_k2 = 1 + float(self.value_slider_5.toPlainText())
        self.moveHandtoPose(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_k1,k2=tar_k2)


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
        elif self.combo.currentText() == "Soft Hand":
            self.finger_label_4.setText("Goal for F4")
            self.tick_f4.setHidden(False)
            self.finger_label_5.setHidden(True)
            self.finger_slider_5.setHidden(True)
            self.value_slider_5.setHidden(True)

    #Reset fingers to home positions
    def handleButtonHome(self):
        self.moveHandtoPose(f1=0.0,f2=0.0,f3=0.0,k1=0.0,k2=1.0)


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
        fileData = self.readFile()
        for pose in fileData:
            self.addWaypoint(is_below=-1,desired_pose=pose)

    #Add Delay to waypoint list
    def handle_add_delay(self):
        delay = self.getDelay()
        if delay == False:
            return
        else:
            self.addWaypoint(is_below=-1,desired_pose=delay)


    ## Update Value of the hand for checking for waypoint
    #but bruh why? what is the point. why you do this.
    # def hand_state_cb(self, hand):
    #     self.current_angle[0] = hand.motor[0].joint_angle
    #     self.current_angle[1] = hand.motor[1].joint_angle
    #     self.current_angle[2] = hand.motor[2].joint_angle
    #     self.current_angle[3] = hand.motor[3].joint_angle

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

    # Receive messages from data glove
    def received_int(self, value_received):
        #Callback for Glove MSGs
        #TODO figure out wtf is going on here
        scaled_float_1 = 2.0-(float(value_received.data[0])-450.0)/100.0
        scaled_float_2 = 2.0-(float(value_received.data[1])-500.0)/80.0
        scaled_float_3 = 2.0 - (float(value_received.data[2])-440)/80.0


        if (self.tick_glove_state == 1):
            # Scale raw value into readable value
            self.value_glove_1.setText("%2.2f" % scaled_float_1)
            self.value_glove_2.setText("%2.2f" % scaled_float_2)
            self.value_glove_3.setText("%2.2f" % scaled_float_3)
            data = str(scaled_float_1) + ";" +str(scaled_float_2) + ";" + str(scaled_float_3) + "\n"
            #why are they writing to a file????????? wtf is going on here. Im so confused
            filename = "data/grasp1.txt"
            file = open(filename, "a")
            file.write(data)
            file.close()
            tar_f1 = scaled_float_2
            tar_f2 = scaled_float_3
            tar_f3 = scaled_float_1
            tar_k1 = float(self.value_slider_4.toPlainText())
            tar_k2 = float(self.value_slider_5.toPlainText())
            self.moveHandtoPose(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_f4,k2=tar_k2)



########### Load File   ############################################################################
    #check data directory and display available grasp files to run
    def populate_filelist(self):
        all_files = [f for f in listdir(FILE_DIR) if isfile(join(FILE_DIR, f))]
        for f in all_files:
            self.fileListWidget.addItem(QListWidgetItem(f))
            self.filename.append(f)



    def deleteWaypoint(self):
        if (self.listPose != []):
            if (self.listWidget.currentRow() < 0):
                error_msg1 = QErrorMessage(self)
                error_msg1.setWindowTitle("Waypoint Error")
                error_msg1.showMessage("Please select a valid waypoint to remove")
            else:
                print self.listWidget.currentRow()
                dummy = self.listPose.pop(self.listWidget.currentRow())
                dummyItem = self.listWidget.takeItem(self.listWidget.currentRow())
                self.listWidget.removeItemWidget(dummyItem)
                print "Removed Waypoint \n", dummy
        else:
            error_msg2 = QErrorMessage(self)
            error_msg2.setWindowTitle("Waypoint Error")
            error_msg2.showMessage("Could not remove waypoint: \nNo waypoints found")



    def addWaypoint(self, is_below, desired_pose):
    #is below = true if waypoint is desired to be below selected row, above if desired to be above, -1 if desired at end of list
        if desired_pose.f1 == 999:
           item = QListWidgetItem("%.2f second delay" % desired_pose.k2)
        else:
            item = QListWidgetItem("[  '%2.2f'  ,  '%2.2f'  ,  '%2.2f'  ,  '%2.2f',  '%2.2f'  ]" % (desired_pose.f1, desired_pose.f2, desired_pose.f3, desired_pose.k1, desired_pose.k2-1))
        #place waypoint in desired location in the list
        if is_below < 0:
            self.listPose.append(desired_pose)
            self.listWidget.addItem(item)
        else:
            list_location = int(self.listWidget.currentRow()) + is_below
            self.listPose.insert(list_location, desired_pose)
            self.listWidget.insertItem(list_location, item)




    #Read a Grasp File and return list of poses
    def readFile(self):
        pose_list = []
        try:
            #read selected file
            file_name = self.fileListWidget.currentItem().text()
            file_path = "{}/{}".format(FILE_DIR, file_name)
            file = open(file_path,'r').read()
            #Divide file into poses
            data_chunks = file.split('//')
            for pose in data_chunks:
                if len(pose) > 1:
                    #divide each pose up by commands per finger
                    f1,f2,f3,k1,k2 = pose.split('\n')
                    #choose only the numerical chunk of command and convert to float
                    tar_f1 = float(f1.split(': ')[1])
                    tar_f2 = float(f2.split(': ')[1])
                    tar_f3 = float(f3.split(': ')[1])
                    tar_k1 = float(k1.split(': ')[1])
                    tar_k2 = float(k2.split(': ')[1])
                    pose0 = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_k1,k2=tar_k2)
                    pose_list.append(pose0)
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
    def moveHandtoPose(self, f1, f2, f3, k1, k2):
        if f1 == 999:
            rospy.sleep(k2)
        else:
            if self.combo.currentText() == "ReflexSF":
                poseTarget = PoseCommand(f1=f1,f2=f2,f3=f3,k1=k1,k2=k2)
                self.command_pub.publish(poseTarget)
            elif self.combo.currentText() == "Soft Hand":
                self.softHand_pose(f1=f1*50,f2=f2*50,f3=f3*50,f4=k1*50)
            rospy.sleep(0.2)

    #return current pose with respect to slider values
    def getSliderPose(self):
        # record slider values
        float_value_1 = float(self.value_slider_1.toPlainText())
        float_value_2 = float(self.value_slider_2.toPlainText())
        float_value_3 = float(self.value_slider_3.toPlainText())
        float_value_4 = float(self.value_slider_4.toPlainText())
        float_value_5 = float(self.value_slider_5.toPlainText())
        pose0 = PoseCommand(f1=float_value_1,f2=float_value_2,f3=float_value_3,k1=float_value_4,k2=1+float_value_5)
        return pose0

    #prompt for delay input
    def getDelay(self):
        num,ok = QInputDialog.getDouble(self,"Delay", "Delay in seconds")
        if ok:
          pose0 = PoseCommand(f1=999,f2=999,f3=999,k1=999,k2=num)
          return pose0
        else:
          return False
