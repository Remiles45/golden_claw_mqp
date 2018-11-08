import rospy
from python_qt_binding.QtWidgets import *
from std_msgs.msg import Int16MultiArray
from reflex_msgs.msg import PoseCommand, Command, VelocityCommand
from file_wr import FileWriteRead

file = FileWriteRead()
class GloveWidget(QWidget):
    def __init__(self):
        super(GloveWidget, self).__init__()
        rospy.Subscriber('/glove_data',Int16MultiArray, self.processData)
        self.command_pub = rospy.Publisher('/reflex_sf/command', Command, queue_size=1) #_position

        self.initUI()


    def initUI(self):
        self.listCommand = []
        self.record_button = QPushButton("Start Recording")
        self.calibrate_glove = QPushButton("Calibrate Glove")
        self.record = QVBoxLayout()
        self.calibrate = QVBoxLayout()
        self.record.addWidget(self.record_button)
        self.calibrate.addWidget(self.calibrate_glove)

        self.hbox_update_tick = QHBoxLayout()
        self.live_update = QCheckBox("Turn on live update")
        self.hbox_update_tick.addWidget(self.live_update)
        self.hbox_update_tick.addStretch()
        self.tick_update_state = 0
        self.live_update.stateChanged.connect(lambda:self.tickchange(self.live_update))




        self.fbox = QFormLayout()
        self.fbox.addRow(QLabel(""),self.hbox_update_tick)
        self.fbox.addRow(QLabel(""), self.record)
        self.fbox.addRow(QLabel("\n\n\nFor best results, calibrate glove before use"))
        self.fbox.addRow(QLabel(""), self.calibrate)
        self.setLayout(self.fbox)
        self.record_button.clicked.connect(self.recordGloveStart)
        self.calibrate_glove.clicked.connect(self.calibrateGlove)
    def tickchange(self,b):
        if b.text() == "Turn on live update":
            if b.isChecked():
                self.tick_update_state = 1
            else:
                self.tick_update_state = 0
    def recordGloveStart(self):
        print "record glove movements"
        if(self.record_button.text() == "Start Recording"):
            self.listCommand = []
            self.record_button.setText("Stop Recording")
        else:
            file.save_file(self.listCommand)
            self.record_button.setText("Start Recording")

    def calibrateGlove(self):
        print "calibrate glove"

    def processData(self,values):
        tar_f1 = float(values.data[0])/100
        tar_f2 = float(values.data[1])/100
        tar_f3 = float(values.data[2])/100
        pose = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=0,k2=1)
        vel = VelocityCommand(f1=1.5,f2=1.5,f3=1.5,k1=0,k2=0)
        command = Command(pose=pose,velocity=vel)
        if self.tick_update_state:
            self.command_pub.publish(command)
        if self.record_button.text() == "Stop Recording":
            self.listCommand.append(command)
