import rospy
from python_qt_binding.QtWidgets import * #QWidget, QToolTip,QPushButton,QLabel,QGridLayout,QLineEdit
from python_qt_binding.QtGui import * #QFont,QPalette, QColor

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from reflex_msgs.msg import PoseCommand
from rqt_service.srv import SendTwoInt
from reflex_msgs.msg import Hand
import os
import sys

class GraspControlWidget(QWidget):

    def __init__(self):
        super(GraspControlWidget, self).__init__()
        self.command_pub = rospy.Publisher('/reflex_sf/command_position', PoseCommand, queue_size=1)
        # self.command_pub_sim = rospy.Publisher('/reflex_sf/hand_state', Hand, queue_size=1)
        # Constantly capture the current hand state
        # rospy.Subscriber('/reflex_sf/hand_state', Hand, self.hand_state_cb)
        rospy.Subscriber('/chatter',Int16MultiArray, self.received_int)
        self.currentGrasp = []
        self.initUI()

    def initUI(self):


################## Position Control GUI ########################################################################
        # #Finger 1 row
        # self.finger_label_1 = QLabel("Goal for f1")
        # self.finger_slider_1 = QSlider(1)
        # self.finger_slider_1.setMinimum(0)
        # self.finger_slider_1.setMaximum(400)
        # self.finger_slider_1.setValue(200)
        # self.value_slider_1 = QTextEdit("2.00")
        # self.value_slider_1.setMaximumSize(80,20)
        # self.hbox_f1 = QHBoxLayout()
        # self.hbox_f1.addWidget(self.finger_slider_1)
        # self.hbox_f1.addWidget(self.value_slider_1)
        #
        # #Finger 2 row
        # self.finger_label_2 = QLabel("Goal for f2")
        # self.finger_slider_2 = QSlider(1)
        # self.finger_slider_2.setMinimum(0)
        # self.finger_slider_2.setMaximum(400)
        # self.finger_slider_2.setValue(300)
        #
        # self.value_slider_2 = QTextEdit("3.00")
        # self.value_slider_2.setMaximumSize(80,20)
        # self.hbox_f2 = QHBoxLayout()
        # self.hbox_f2.addWidget(self.finger_slider_2)
        # self.hbox_f2.addWidget(self.value_slider_2)
        #
        # #Finger 3 row
        # self.finger_label_3 = QLabel("Goal for f3")
        # self.finger_slider_3 = QSlider(1)
        # self.finger_slider_3.setMinimum(0)
        # self.finger_slider_3.setMaximum(400)
        # self.finger_slider_3.setValue(100)
        #
        # self.value_slider_3 = QTextEdit("1.00")
        # self.value_slider_3.setMaximumSize(80,20)
        # self.hbox_f3 = QHBoxLayout()
        # self.hbox_f3.addWidget(self.finger_slider_3)
        # self.hbox_f3.addWidget(self.value_slider_3)
        #
        # #Finger 4
        # self.finger_label_4 = QLabel("Goal for f_preshape")
        # self.finger_slider_4 = QSlider(1)
        # self.finger_slider_4.setMinimum(0)
        # self.finger_slider_4.setMaximum(400)
        # self.finger_slider_4.setValue(0)
        #
        # self.value_slider_4 = QTextEdit("0.00")
        # self.value_slider_4.setMaximumSize(80,20)
        # self.hbox_f4 = QHBoxLayout()
        # self.hbox_f4.addWidget(self.finger_slider_4)
        # self.hbox_f4.addWidget(self.value_slider_4)
########## Coupling Row ###################################################################################
        #Tick line row, choosing coupling motor
#         self.coupling_label = QLabel("Coupling")
#         self.hbox_tick = QHBoxLayout()
#
#         self.tick_f1 = QCheckBox("F1")
#         self.tick_f2 = QCheckBox("F2")
#         self.tick_f3 = QCheckBox("F3")
#         self.tick_f4 = QCheckBox("Preshape")
#
#         self.hbox_tick.addWidget(self.tick_f1)
#         self.hbox_tick.addWidget(self.tick_f2)
#         self.hbox_tick.addWidget(self.tick_f3)
#         self.hbox_tick.addWidget(self.tick_f4)
#         self.hbox_tick.addStretch()
#
#         self.tick_f1_state = 0
#         self.tick_f2_state = 0
#         self.tick_f3_state = 0
#         self.tick_f4_state = 0
#         self.tick_f1.stateChanged.connect(lambda:self.tickchange(self.tick_f1))
#         self.tick_f2.stateChanged.connect(lambda:self.tickchange(self.tick_f2))
#         self.tick_f3.stateChanged.connect(lambda:self.tickchange(self.tick_f3))
#         self.tick_f4.stateChanged.connect(lambda:self.tickchange(self.tick_f4))
#
# ########### Command Row Button ############################################################################
#         # Command row
#         self.command_label = QLabel("Command")
#         self.go_button = QPushButton("Go Go Go !!!")
#         self.home_button = QPushButton("Go Home boys !!!")
#         self.re_button = QPushButton("Reset Goal")
#
#         self.hbox_command = QHBoxLayout()
#         self.hbox_command.addWidget(self.go_button)
#         self.hbox_command.addWidget(self.home_button)
#         self.hbox_command.addWidget(self.re_button)
########### Combo section ############################################################################
#         self.combo_label = QLabel("Targeted Device")
#         self.combo = QComboBox(self)
#         self.combo.addItem("ReflexSF")
#         self.combo.addItem("MQP Hand")
#         self.combo.addItem("Rviz/Gazebo Simulation")
# ############ Glove Section #############################################################################
#         self.glove_label = QLabel("Glove Interface")
#
#
#         self.tick_glove = QCheckBox("ON/OFF")
#         self.tick_glove_state = 0
#         self.tick_glove.stateChanged.connect(lambda:self.tickchange(self.tick_glove))
#
#         self.value_glove_1 = QLabel("x")
#         self.value_glove_1.setMaximumSize(80,20)
#
#         self.value_glove_2 = QLabel("x")
#         self.value_glove_2.setMaximumSize(80,20)
#
#         self.value_glove_3 = QLabel("x")
#         self.value_glove_3.setMaximumSize(80,20)
#
#         self.hbox_glove = QHBoxLayout()
#         self.hbox_glove.addWidget(self.tick_glove)
#         self.hbox_glove.addWidget(self.value_glove_1)
#         self.hbox_glove.addWidget(self.value_glove_2)
#         self.hbox_glove.addWidget(self.value_glove_3)
# ##########################################################################################################
        #List of grasp in data folder
        self.grasplist = []
        self.filename = []
        #Test List view

        self.readGrasp()


        self.listlabel = QLabel("List waypoint")

        #List Control

        self.list_control_label = QLabel("Grasp Control")
        self.list_control_save_button = QPushButton("Save Grasp")
        self.list_control_delete_button = QPushButton("Remove")
        self.list_control_go_button = QPushButton("Execution")
        self.list_control = QHBoxLayout()
        self.list_control.addWidget(self.list_control_save_button)
        self.list_control.addWidget(self.list_control_delete_button)
        self.list_control.addWidget(self.list_control_go_button)
############ Adding rows and set up singal for button ####################################################
        #QFormLayout similar to HBox but you know it look like form, add everything to FormLayout
        self.fbox = QFormLayout()
        # self.fbox.addRow(self.combo_label,self.combo)
        # self.fbox.addRow(self.glove_label,self.hbox_glove)
        # self.fbox.addRow(self.finger_label_4,self.hbox_f4)
        self.fbox.addRow(self.listlabel,self.listWidget)
        self.fbox.addRow(self.list_control_label,self.list_control)


        # # Connect singal when slider change to function respectively to change value of label
        # self.finger_slider_1.valueChanged.connect(self.valuechange1)
        # self.finger_slider_2.valueChanged.connect(self.valuechange2)
        # self.finger_slider_3.valueChanged.connect(self.valuechange3)
        # self.finger_slider_4.valueChanged.connect(self.valuechange4)
        #
        # # Add connect signal to Button Go, Cancel and Reset
        # self.go_button.clicked.connect(self.handleButtonGo)
        # self.home_button.clicked.connect(self.handleButtonHome)
        # self.re_button.clicked.connect(self.handleButtonReset)
        #
        self.list_control_save_button.clicked.connect(self.handle_list_control_save_button)
        self.list_control_delete_button.clicked.connect(self.handle_list_control_delete_button)
        self.list_control_go_button.clicked.connect(self.handle_list_control_go_button)

######### Set up window ###################################################################################
        #Set the widget to layout and show the widget
        self.setLayout(self.fbox)

        self.setWindowTitle("Grasp Control")
        self.resize(640,480)
        self.dumbnum = 0
        self.show()
        self.current_angle = [0.0,0.0,0.0,0.0]
#############################################################################################################
    def handle_list_control_save_button(self):
        #currentGrasp = [[1.0,1.0,1.0],[2.0,2.0,2.0],[3.0,3.0,3.0]]
        abspath = os.path.abspath(__file__)

        folderdatapath = abspath[:-len('/src/rqt_gui_control/glove_gui.py')] + '/data'
        #print(folderdatapath)
        name = 'grasp'+str(len(self.filename))+'.txt'
        filename = folderdatapath + '/' + name
        print(filename)
        file = open(filename, "w")
        data = "0.0;0.0;0.0" + "\n"
        file.write(data)
        file.close()

        for data in self.currentGrasp:
            data = str(data[0]) + ";" +str(data[1]) + ";" + str(data[2]) + "\n"
            file = open(filename, "a")
            file.write(data)
            file.close()

        self.filename.append(name)
        self.grasplist.append(filename)
        item = QListWidgetItem(name)
        self.listWidget.addItem(item)

    def handle_list_control_delete_button(self):
        print "Remove Grasp"
        dummy = self.filename.pop(self.listWidget.currentRow())
        dummy2 = self.grasplist.pop(self.listWidget.currentRow())

        dummyItem = self.listWidget.takeItem(self.listWidget.currentRow())


    def handle_list_control_go_button(self):
        currentChoicepath = self.grasplist[self.listWidget.currentRow()]
        currentChoicename = self.filename[self.listWidget.currentRow()]
        print("Execute grapsh: " + currentChoicename)
        file = open(currentChoicepath,'r').read()

        lines = file.split('\n')
        executionPose = []
        for line in lines:
            if len(line) > 1:
                s1,s2,s3 = line.split(';')
                executionPose.append([float(s1),float(s2),float(s3)])

        for pose in executionPose:
            scaled_float_1 = pose[0]
            scaled_float_2 = pose[1]
            scaled_float_3 = pose[2]


            # Scale raw value into readable value
            self.value_glove_1.setText("%2.2f" % scaled_float_1)
            self.value_glove_2.setText("%2.2f" % scaled_float_2)
            self.value_glove_3.setText("%2.2f" % scaled_float_3)

            tar_f1 = scaled_float_2
            tar_f2 = scaled_float_3
            tar_f3 = scaled_float_1
            tar_f4 = float(self.value_slider_4.toPlainText())
            poseTarget = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_f4)
            self.command_pub.publish(poseTarget)

            rospy.sleep(0.2)
        print "Finish Grasp"
######### tickchange for updating tick ###############################################################

######### valuechange for updating goal label ###############################################################
    def valuechange1(self):
        #self.dumbnum = self.dumbnum + 1;
        float_value = float(self.finger_slider_1.value())/100.0
        self.value_slider_1.setText("%2.2f" % float_value)
        if self.tick_f1_state:
            if self.tick_f2_state:
                self.value_slider_2.setText("%2.2f" % float_value)
                self.finger_slider_2.setValue(self.finger_slider_1.value())
            if self.tick_f3_state:
                self.value_slider_3.setText("%2.2f" % float_value)
                self.finger_slider_3.setValue(self.finger_slider_1.value())
            if self.tick_f4_state:
                self.value_slider_4.setText("%2.2f" % float_value)
                self.finger_slider_4.setValue(self.finger_slider_1.value())

        #print "test time" + str(self.dumbnum)

    def valuechange2(self):
        float_value = float(self.finger_slider_2.value())/100.0
        self.value_slider_2.setText("%2.2f" % float_value)

    def valuechange3(self):
        float_value = float(self.finger_slider_3.value())/100.0
        self.value_slider_3.setText("%2.2f" % float_value)

    def valuechange4(self):
        float_value = float(self.finger_slider_4.value())/100.0
        self.value_slider_4.setText("%2.2f" % float_value)

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
        if b.text() == "Preshape":
            if b.isChecked() == True:
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
        #print(self.tick_f1_state)

######### Command Button handler ############################################################################
    def handleButtonGo(self):
        if self.combo.currentText() == "Rviz/Gazebo Simulation":
            tar_f1 = float(self.value_slider_1.toPlainText())
            tar_f2 = float(self.value_slider_2.toPlainText())
            tar_f3 = float(self.value_slider_3.toPlainText())
            tar_f4 = float(self.value_slider_4.toPlainText())
            print "Go Button Click Simluation"
            print(tar_f1,tar_f2,tar_f3,tar_f4)
            hand = Hand()
            hand.motor[0].joint_angle = tar_f1
            hand.motor[1].joint_angle = tar_f2
            hand.motor[2].joint_angle = tar_f3
            hand.motor[3].joint_angle = tar_f4
            self.command_pub_sim.publish(hand)


        else:
            tar_f1 = float(self.value_slider_1.toPlainText())
            tar_f2 = float(self.value_slider_2.toPlainText())
            tar_f3 = float(self.value_slider_3.toPlainText())
            tar_f4 = float(self.value_slider_4.toPlainText())
            print "Go Button Click with target"
            print(tar_f1,tar_f2,tar_f3,tar_f4)
            poseTarget = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_f4)
            self.command_pub.publish(poseTarget)
            #print "Go Button Click"

    def handleButtonHome(self):
        poseTarget = PoseCommand(f1=0.0,f2=0.0,f3=0.0,k1=0.0)
        self.command_pub.publish(poseTarget)
        print "Home Button Click"

    def handleButtonReset(self):
        self.finger_slider_1.setValue(0)
        self.value_slider_1.setText("0.00")
        self.finger_slider_2.setValue(0)
        self.value_slider_2.setText("0.00")
        self.finger_slider_3.setValue(0)
        self.value_slider_3.setText("0.00")
        self.finger_slider_4.setValue(0)
        self.value_slider_4.setText("0.00")
        print "Reset Button Click"
    ## Update Value of the hand for checking for waypoint
    def hand_state_cb(self, hand):
        pass

    # Receive messages from
    def received_int(self, value_received):

        # Simulation
        scaled_float_1 = 3.0-((float(value_received.data[0])-450.0)/100.0+2.0)
        scaled_float_2 = 3.0-((float(value_received.data[1])-500.0)/80.0  + 0.5)
        scaled_float_3 = 3.0 - ((float(value_received.data[2])-440)/80.0  + 0.5)

        if (self.tick_glove_state == 1):
            # Scale raw value into readable value

            self.value_glove_1.setText("%2.2f" % scaled_float_1)
            self.value_glove_2.setText("%2.2f" % scaled_float_2)
            self.value_glove_3.setText("%2.2f" % scaled_float_3)
            self.currentGrasp.append([scaled_float_1,scaled_float_2,scaled_float_3])


            # # Based on the Combo box decided what to do with the value
            if (self.combo.currentText() == "Rviz/Gazebo Simulation"):
                #print(scaled_float_1)
                tar_f1 = scaled_float_2
                tar_f2 = scaled_float_3
                tar_f3 = scaled_float_1
                tar_f4 = float(self.value_slider_4.toPlainText())

                hand = Hand()
                hand.motor[0].joint_angle = tar_f1
                hand.motor[1].joint_angle = tar_f2
                hand.motor[2].joint_angle = tar_f3
                hand.motor[3].joint_angle = tar_f4
                self.command_pub_sim.publish(hand)
            else:
                tar_f1 = scaled_float_2
                tar_f2 = scaled_float_3
                tar_f3 = scaled_float_1
                tar_f4 = float(self.value_slider_4.toPlainText())
                poseTarget = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,preshape=tar_f4)
                self.command_pub.publish(poseTarget)
        else:
            self.currentGrasp = []

    def readGrasp(self):

        abspath = os.path.abspath(__file__)

        folderdatapath = abspath[:-len('/src/rqt_gui_control/grasp_cntrl_widget.py')] + '/data'

        for file in os.listdir(folderdatapath):
            if file.endswith(".txt"):
                self.grasplist.append(os.path.join(folderdatapath, file))
                self.filename.append(file)
        #print(self.grasplist)

        self.listWidget = QListWidget()
        for name in self.filename:
            item = QListWidgetItem(name)
            self.listWidget.addItem(item)
