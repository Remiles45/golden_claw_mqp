import rospy
from python_qt_binding.QtWidgets import *
from std_msgs.msg import Int16MultiArray

class GloveWidget(QWidget):
    def __init__(self):
        super(GloveWidget, self).__init__()
        rospy.Subscriber('/glove_data',Int16MultiArray, self.processData())
        self.initUI()


    def initUI(self):
        self.record_button = QPushButton("Start Recording")
        self.calibrate_glove = QPushButton("Calibrate Glove")
        self.record = QVBoxLayout()
        self.calibrate = QVBoxLayout()
        self.record.addWidget(self.record_button)
        self.calibrate.addWidget(self.calibrate_glove)
        self.fbox = QFormLayout()
        self.fbox.addRow(QLabel(""), self.record)
        self.fbox.addRow(QLabel("\n\n\nFor best results, calibrate glove before use"))
        self.fbox.addRow(QLabel(""), self.calibrate)
        self.setLayout(self.fbox)

        self.record_button.clicked.connect(self.recordGloveStart)
        self.calibrate_glove.clicked.connect(self.calibrateGlove)

    def recordGloveStart(self):
        print "record glove movements"
        if(self.record_button.text() == "Start Recording"):
            self.record_button.setText("Stop Recording")
        else:
            self.record_button.setText("Start Recording")
    def calibrateGlove(self):
        print "calibrate glove"

    def processData(self):
        print "received data"
