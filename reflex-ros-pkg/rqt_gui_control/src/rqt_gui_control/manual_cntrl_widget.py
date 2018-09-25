import os
import rospkg
import rospy
from python_qt_binding.QtWidgets import * #QWidget, QToolTip,QPushButton,QLabel,QGridLayout,QLineEdit
from python_qt_binding.QtGui import * #QFont,QPalette, QColor

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from reflex_msgs.msg import PoseCommand
# from rqt_service.srv import SendTwoInt
from reflex_msgs.msg import Hand
import socket
from std_msgs.msg import UInt16

class ManualHandControlWidget(QWidget):

    def __init__(self):
        super(ManualHandControlWidget, self).__init__()
        self.command_pub = rospy.Publisher('/reflex_sf/command_position', PoseCommand, queue_size=1)
        # Constantly capture the current hand state
        # rospy.Subscriber('/reflex_sf/hand_state', Hand, self.hand_state_cb)
        rospy.Subscriber('/chatter',Int16MultiArray, self.received_int)####FIXME
        #rospy.init_node('listener', anonymous=True)
        self.currentGrasp = []
        self.initUI()
        #soft hand
        self.command_pub_softhand_1 = rospy.Publisher('UbirosGentlePro1', UInt16, queue_size=1)
        self.command_pub_softhand_2 = rospy.Publisher('UbirosGentlePro2', UInt16, queue_size=1)
        self.command_pub_softhand_3 = rospy.Publisher('UbirosGentlePro3', UInt16, queue_size=1)
        self.command_pub_softhand_4 = rospy.Publisher('UbirosGentlePro4', UInt16, queue_size=1)


    def initUI(self):

################## Position Control GUI ########################################################################
#Fingers: slider range 0 -> 400
        #Finger 1 row
        self.finger_label_1 = QLabel("Goal for f1")
        self.finger_slider_1 = QSlider(1)
        self.finger_slider_1.setMinimum(0)
        self.finger_slider_1.setMaximum(400)
        self.finger_slider_1.setValue(200)

        self.value_slider_1 = QTextEdit("2.00")
        self.value_slider_1.setMaximumSize(80,20)
        self.hbox_f1 = QHBoxLayout()
        self.hbox_f1.addWidget(self.finger_slider_1)
        self.hbox_f1.addWidget(self.value_slider_1)

        #Finger 2 row
        self.finger_label_2 = QLabel("Goal for f2")
        self.finger_slider_2 = QSlider(1)
        self.finger_slider_2.setMinimum(0)
        self.finger_slider_2.setMaximum(400)
        self.finger_slider_2.setValue(300)

        self.value_slider_2 = QTextEdit("3.00")
        self.value_slider_2.setMaximumSize(80,20)
        self.hbox_f2 = QHBoxLayout()
        self.hbox_f2.addWidget(self.finger_slider_2)
        self.hbox_f2.addWidget(self.value_slider_2)

        #Finger 3 row
        self.finger_label_3 = QLabel("Goal for f3")
        self.finger_slider_3 = QSlider(1)
        self.finger_slider_3.setMinimum(0)
        self.finger_slider_3.setMaximum(400)
        self.finger_slider_3.setValue(100)

        self.value_slider_3 = QTextEdit("1.00")
        self.value_slider_3.setMaximumSize(80,20)
        self.hbox_f3 = QHBoxLayout()
        self.hbox_f3.addWidget(self.finger_slider_3)
        self.hbox_f3.addWidget(self.value_slider_3)
#Preshape: slider range 0 -> 400
        #Preshape k1 (index/middle fingers)
        self.finger_label_4 = QLabel("Distance between fingers 1 and 2 (Soft Hand F4)") #actually check that im not lying
        self.finger_slider_4 = QSlider(1)
        self.finger_slider_4.setMinimum(0)
        self.finger_slider_4.setMaximum(400)
        self.finger_slider_4.setValue(0)

        self.value_slider_4 = QTextEdit("0.00")
        self.value_slider_4.setMaximumSize(80,20)
        self.hbox_f4 = QHBoxLayout()
        self.hbox_f4.addWidget(self.finger_slider_4)
        self.hbox_f4.addWidget(self.value_slider_4)

        #Preshape k2
        self.finger_label_5 = QLabel("Thumb Rotation (Soft Hand: N/A)")#acutally check this one is the thumb
        self.finger_slider_5 = QSlider(1)
        self.finger_slider_5.setMinimum(0)
        self.finger_slider_5.setMaximum(400)
        self.finger_slider_5.setValue(0)

        self.value_slider_5 = QTextEdit("0.00")
        self.value_slider_5.setMaximumSize(80,20)
        self.hbox_f5 = QHBoxLayout()
        self.hbox_f5.addWidget(self.finger_slider_5)
        self.hbox_f5.addWidget(self.value_slider_5)

########## Coupling Row ###################################################################################
        #Tick line row, choosing coupling motor
        self.coupling_label = QLabel("Coupling")
        self.hbox_tick = QHBoxLayout()

        self.tick_f1 = QCheckBox("F1")
        self.tick_f2 = QCheckBox("F2")
        self.tick_f3 = QCheckBox("F3")
        # self.tick_f4 = QCheckBox("Preshape")

        self.hbox_tick.addWidget(self.tick_f1)
        self.hbox_tick.addWidget(self.tick_f2)
        self.hbox_tick.addWidget(self.tick_f3)
        # self.hbox_tick.addWidget(self.tick_f4)
        self.hbox_tick.addStretch()

        self.tick_f1_state = 0
        self.tick_f2_state = 0
        self.tick_f3_state = 0
        self.tick_f4_state = 0
        self.tick_f1.stateChanged.connect(lambda:self.tickchange(self.tick_f1))
        self.tick_f2.stateChanged.connect(lambda:self.tickchange(self.tick_f2))
        self.tick_f3.stateChanged.connect(lambda:self.tickchange(self.tick_f3))
        ######is this actually controlling the hand? or JUST coupling
        # self.tick_f4.stateChanged.connect(lambda:self.tickchange(self.tick_f4))

########### Command Row Button ############################################################################
        self.command_label = QLabel("Manual Command Hand")
        self.go_button = QPushButton("Go to set values")
        self.re_button = QPushButton("Reset Goal Values")
        self.home_button = QPushButton("Reset Finger Positions")

        self.hbox_command = QHBoxLayout()
        self.hbox_command.addWidget(self.go_button)
        self.hbox_command.addWidget(self.re_button)
        self.hbox_command.addWidget(self.home_button)
########### Combo section ############################################################################
        self.combo_label = QLabel("Targeted Device")
        self.combo = QComboBox(self)
        self.combo.addItem("ReflexSF")
        self.combo.addItem("Soft Hand")
############ Glove Section #############################################################################
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

        self.hbox_glove = QHBoxLayout()
        self.hbox_glove.addWidget(self.tick_glove)
        self.hbox_glove.addWidget(self.value_glove_1)
        self.hbox_glove.addWidget(self.value_glove_2)
        self.hbox_glove.addWidget(self.value_glove_3)

##########################################################################################################
        self.listPose = []
        self.grasplist = []
        self.filename = []

        pose0 = PoseCommand(f1=0.0,f2=0.0,f3=0.0,k1=0.0,k2=0.0)
        self.listPose.append(pose0)
        #Test List view
        self.listWidget = QListWidget()

        item = QListWidgetItem("Pos(  '%2.2f'  ,  '%2.2f'  ,  '%2.2f'  ,  '%2.2f',  '%2.2f'  )" % (pose0.f1,pose0.f2,pose0.f3,pose0.k1,pose0.k2))
        self.listWidget.addItem(item)

        self.listlabel = QLabel("List waypoint")

        #List Control
        self.list_control_label = QLabel("Waypoint Control")
        self.list_control_save_button = QPushButton("Add Waypoint")
        self.list_control_delete_button = QPushButton("Remove Waypoint")
        self.list_control_execute_waypoints = QPushButton("Execute Waypoints")#TODO does not seem to send messages
        self.list_control_save_grasp = QPushButton("Save Grasp")
        self.list_control_execute_existing_grasp = QPushButton("Execute Grasp File")
        self.list_control = QHBoxLayout()
        self.list_control.addWidget(self.list_control_save_button)
        self.list_control.addWidget(self.list_control_delete_button)
        self.list_control.addWidget(self.list_control_execute_waypoints)
        self.list_control.addWidget(self.list_control_save_grasp)
        self.list_control.addWidget(self.list_control_execute_existing_grasp)
############ Adding rows and set up singal for button ####################################################
        #QFormLayout similar to HBox but you know it look like form, add everything to FormLayout
        self.fbox = QFormLayout()
        self.fbox.addRow(self.finger_label_1,self.hbox_f1)
        self.fbox.addRow(self.finger_label_2,self.hbox_f2)
        self.fbox.addRow(self.finger_label_3,self.hbox_f3)
        self.fbox.addRow(self.finger_label_4,self.hbox_f4)
        self.fbox.addRow(self.finger_label_5,self.hbox_f5)
        self.fbox.addRow(self.coupling_label,self.hbox_tick)
        self.fbox.addRow(self.command_label,self.hbox_command)
        self.fbox.addRow(self.listlabel,self.listWidget)
        self.fbox.addRow(self.list_control_label,self.list_control)
        self.fbox.addRow(self.combo_label,self.combo)
        self.fbox.addRow(self.glove_label,self.hbox_glove)

        # Connect singal when slider change to function respectively to change value of label
        self.finger_slider_1.valueChanged.connect(self.valuechange1)
        self.finger_slider_2.valueChanged.connect(self.valuechange2)
        self.finger_slider_3.valueChanged.connect(self.valuechange3)
        self.finger_slider_4.valueChanged.connect(self.valuechange4)
        self.finger_slider_5.valueChanged.connect(self.valuechange5)
        # Add connect signal to Button Go, Cancel and Reset
        self.go_button.clicked.connect(self.handleButtonGo)
        self.home_button.clicked.connect(self.handleButtonHome)
        self.re_button.clicked.connect(self.handleButtonReset)

        self.list_control_save_button.clicked.connect(self.handle_list_control_save_button)
        self.list_control_delete_button.clicked.connect(self.handle_list_control_delete_button)
        self.list_control_execute_waypoints.clicked.connect(self.handle_execute_waypoints)
        self.list_control_save_grasp.clicked.connect(self.handle_grasp_save_button)
        self.list_control_execute_existing_grasp.clicked.connect(self.handle_run_existing_grasp_button)
######### Set up window ###################################################################################
        #Set the widget to layout and show the widget
        self.setLayout(self.fbox)

        self.setWindowTitle("Manual Hand Controller")
        self.resize(640,480)
        self.dumbnum = 0
        self.show()
        self.current_angle = [0.0,0.0,0.0,0.0]

#############################################################################################################
    def handle_list_control_save_button(self):
        float_value_1 = float(self.value_slider_1.toPlainText())
        float_value_2 = float(self.value_slider_2.toPlainText())
        float_value_3 = float(self.value_slider_3.toPlainText())
        float_value_4 = float(self.value_slider_4.toPlainText())
        float_value_5 = float(self.value_slider_5.toPlainText())
        pose0 = PoseCommand(f1=float_value_1,f2=float_value_2,f3=float_value_3,k1=float_value_4,k2=float_value_5)
        self.listPose.append(pose0)

        item = QListWidgetItem("Pos(  '%2.2f'  ,  '%2.2f'  ,  '%2.2f'  ,  '%2.2f',  '%2.2f'  )" % (pose0.f1, pose0.f2, pose0.f3, pose0.k1, pose0.k2))
        self.listWidget.addItem(item)

    def handle_list_control_delete_button(self):
        #TODO remove the item from the displayed list
        #i.e. have the window be just the list of waypoints

        if (self.listPose != []):
            dummy = self.listPose.pop(self.listWidget.currentRow())
            dummyItem = self.listWidget.takeItem(self.listWidget.currentRow())
            print "Removed Waypoint ", dummy
        else:
            print "Could not remove waypoint: No waypoint found"


    def handle_execute_waypoints(self):
        #TODO bring up menu to select file from directory
        #actually on second thought not sure what this is doing. just reading from glove?
        #no clue why it is writing to a file that doesnt exist
        scaled_float_1 = 1.0
        scaled_float_2 = 1.0
        scaled_float_3 = 1.0


        if (self.tick_glove_state == 1):
            self.value_glove_1.setText("%2.2f" % scaled_float_1)
            self.value_glove_2.setText("%2.2f" % scaled_float_2)
            self.value_glove_3.setText("%2.2f" % scaled_float_3)
            data = str(scaled_float_1) + ";" +str(scaled_float_2) + ";" + str(scaled_float_3) + "\n"
            filename = "data/grasp1.txt"# I dont think this should be here?
            file = open(filename, "a")
            file.write(data)
            file.close()

#############################################################################################################
    def handle_grasp_save_button(self):
        #TODO prompt to edit filename/path
        #replace next 4 lines with prompt to select save directory+ rename file
        abspath = os.path.abspath(__file__)
        folderdatapath = abspath[:-len('/src/rqt_gui_control/manual_cntrl_widget.py')] + '/data'
        name = 'grasp'+str(len(self.filename))+'.txt'
        filename = folderdatapath + '/' + name

        #write waypoint list to file
        if len(self.listPose) > 0:
            print 'saved ' + str(len(self.listPose)) + ' waypoints to ' + name
            count = 0
            for point in self.listPose:
                #add indicator for each chunk of data
                data = "//" + str(point)
                file = open(filename, "a")
                file.write(data)
                file.close()
        else:
            print "No waypoints to save"

        self.filename.append(name)
        self.grasplist.append(filename)
        item = QListWidgetItem(name)
        self.listWidget.addItem(item)


    def handle_run_existing_grasp_button(self):
        #TODO I would like a button to open a window where the default
        #     path is to /data and you select the file and click "open"
        #     and have it execute from there

        #replace next 4 lines  with prompt to choose destination file (default /data)
        # and rename file.
        currentChoicepath = self.grasplist[self.listWidget.currentRow()]
        currentChoicename = self.filename[self.listWidget.currentRow()]
        print("Execute grasp: " + currentChoicename)
        file = open(currentChoicepath,'r').read()

        #Divide file by pose commands
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

                poseTarget = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_k1) #preshape=tar_f4)
                self.command_pub.publish(poseTarget)
        rospy.sleep(0.2)

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

#Update preshape slider values
    def valuechange4(self):
        float_value = float(self.finger_slider_4.value())/100.0
        self.value_slider_4.setText("%2.2f" % float_value)

    def valuechange5(self):
        float_value = float(self.finger_slider_5.value())/100.0 - 2.0
        self.value_slider_5.setText("%2.2f" % float_value)

#############################################################################################################
    def tickchange(self,b):
        if b.text() == "F1":
            if b.isChecked() == True:
                self.tick_f1_state = 1
            else:
                self.tick_f1_state = 0
        if b.text() == "F2":
            if b.isChecked() == True:
                self.tick_f2_state = 1
            else:
                self.tick_f2_state = 0
        if b.text() == "F3":
            if b.isChecked() == True:
                self.tick_f3_state = 1
            else:
                self.tick_f3_state = 0
        if b.text() == "ON/OFF":
            if b.isChecked() == True:
                self.tick_glove_state = 1
            else:
                self.tick_glove_state = 0
                self.value_glove_1.setText("x")
                self.value_glove_2.setText("x")
                self.value_glove_3.setText("x")

######### Command Button handler ############################################################################
    def handleButtonGo(self):
        #send current slider values to hand
        if self.combo.currentText() == "ReflexSF":
            tar_f1 = float(self.value_slider_1.toPlainText())
            tar_f2 = float(self.value_slider_2.toPlainText())
            tar_f3 = float(self.value_slider_3.toPlainText())
            tar_k1 = float(self.value_slider_4.toPlainText())
            tar_k2 = float(self.value_slider_5.toPlainText())

            print "Sending Hand to: "
            print(tar_f1,tar_f2,tar_f3,tar_k1,tar_k2)
            poseTarget = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_k1,k2=tar_k2)
            self.command_pub.publish(poseTarget)
        elif self.combo.currentText() == "Soft Hand":
            tar_f1 = int(float(self.value_slider_1.toPlainText())*25)
            tar_f2 = int(float(self.value_slider_2.toPlainText())*25)
            tar_f3 = int(float(self.value_slider_3.toPlainText())*25)
            tar_k1 = int(float(self.value_slider_4.toPlainText())*25)

            print "Sending Hand to: "
            print(tar_f1,tar_f2,tar_f3,tar_k1)
            self.softHand_pose(f1=tar_f1,f2=tar_f2,f3=tar_f3,f4=tar_k1)


    def handleButtonHome(self):
        #send the fingers to home positions
        poseTarget = PoseCommand(f1=0.0,f2=0.0,f3=0.0,k1=0.0,k2=0.0)
        self.command_pub.publish(poseTarget)
        self.softHand_pose(f1=30,f2=30,f3=30,f4=30)

    def handleButtonReset(self):
        #set slider values to 0
        self.finger_slider_1.setValue(0)
        self.value_slider_1.setText("0.00")
        self.finger_slider_2.setValue(0)
        self.value_slider_2.setText("0.00")
        self.finger_slider_3.setValue(0)
        self.value_slider_3.setText("0.00")
        self.finger_slider_4.setValue(0)
        self.value_slider_4.setText("0.00")
        self.finger_slider_5.setValue(0)
        self.value_slider_5.setText("0.00")


    ## Update Value of the hand for checking for waypoint
    #but bruh why? what is the point. why you do this.
    # def hand_state_cb(self, hand):
    #     self.current_angle[0] = hand.motor[0].joint_angle
    #     self.current_angle[1] = hand.motor[1].joint_angle
    #     self.current_angle[2] = hand.motor[2].joint_angle
    #     self.current_angle[3] = hand.motor[3].joint_angle

    def softHand_pose(self,f1,f2,f3,f4):
        f1 = 100 if int(f1) > 100 else int(f1)
        f2 = 100 if int(f2) > 100 else int(f2)
        f3 = 100 if int(f3) > 100 else int(f3)
        f4 = 100 if int(f4) > 100 else int(f4)
        self.command_pub_softhand_1.publish(f1)
        self.command_pub_softhand_2.publish(f2)
        self.command_pub_softhand_3.publish(f3)
        self.command_pub_softhand_4.publish(f4)

    # Receive messages from
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
            tar_f4 = float(self.value_slider_4.toPlainText())
            # poseTarget = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,preshape=tar_f4)
            # self.command_pub.publish(poseTarget)

            if self.combo.currentText() == "ReflexSF":
                poseTarget = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,preshape=tar_f4)
                self.command_pub.publish(poseTarget)
            elif self.combo.currentText() == "Soft Hand":
                self.softHand_pose(f1=tar_f1,f2=tar_f2,f3=tar_f3,f4=tar_f4)
