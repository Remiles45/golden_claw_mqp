import rospy
from python_qt_binding.QtWidgets import * #QWidget, QToolTip,QPushButton,QLabel,QGridLayout,QLineEdit


class GloveWidget(QWidget):
    def __init__(self):
        super(GloveWidget, self).__init__()
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
        self.fbox.addRow(QLabel(""), self.calibrate)
        self.fbox.setVerticalSpacing(50)
        self.setLayout(self.fbox)
