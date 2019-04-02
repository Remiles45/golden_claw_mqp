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
        self.command_pub = rospy.Publisher('/reflex_sf/command', Command, queue_size=1)
        self.initUI()

    def initUI(self):
        self.listCommand = []
        self.record_button = QPushButton("Start Recording")
        self.record = QVBoxLayout()
        self.record.addWidget(self.record_button)

        self.hbox_update_tick = QHBoxLayout()
        self.live_update = QCheckBox("Turn on live update")
        self.hbox_update_tick.addWidget(self.live_update)
        self.hbox_update_tick.addStretch()

        self.fbox = QFormLayout()
        self.fbox.addRow(QLabel("\n\n\nFor best results, calibrate glove before use \n\nTo calibrate: open hand flat then clench into a fist"))
        self.fbox.addRow(QLabel(""),self.hbox_update_tick)
        self.fbox.addRow(QLabel(""), self.record)
        self.setLayout(self.fbox)
        self.record_button.clicked.connect(self.recordGloveStart)

    def recordGloveStart(self):
        """
            Update record button text to enable waypoint recording
        """
        if(self.record_button.text() == "Start Recording"):
            self.listCommand = []
            self.record_button.setText("Stop Recording")
        else:
            file.save_file(self.listCommand)
            self.record_button.setText("Start Recording")

    def processData(self,values):
        """
            Callback for glove data msg.
            convert message to Command() and add to list
        """
        tar_f1 = float(values.data[0])/100
        tar_f2 = float(values.data[1])/100
        tar_f3 = float(values.data[2])/100
        tar_k1 = float(values.data[3])/100
        tar_k2 = (float(values.data[4])/100) - 1
        pose = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_k1,k2=tar_k2)
        vel = VelocityCommand(f1=1.5,f2=1.5,f3=1.5,k1=1.5,k2=1.5)
        command = Command(pose=pose,velocity=vel)
        if self.live_update.isChecked() and self.isVisible():

            self.command_pub.publish(command)
        if self.record_button.text() == "Stop Recording":
            self.listCommand.append(command)
