import rospy
import socket
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *

from std_msgs.msg import String
from reflex_msgs.msg import PoseCommand
from std_srvs.srv import Empty
from rqt_service.srv import SendTwoInt

global calibrate_hand
class CalibrationWidget(QWidget):
    def __init__(self):
        super(CalibrationWidget, self).__init__()

        self.initUI()

    def initUI(self):
########### Calibrate section ############################################################################
        # Calibrate f1 row
        self.cali_f1_label = QLabel("Calibrate f1")
        self.cali_f1_tight_button = QPushButton("Tightening f1")
        self.cali_f1_loosen_button = QPushButton("Loosensing f1")
        self.hbox_cali_f1 = QVBoxLayout()
        self.hbox_cali_f1.addWidget(self.cali_f1_tight_button)
        self.hbox_cali_f1.addWidget(self.cali_f1_loosen_button)

        # Calibrate f2 row
        self.cali_f2_label = QLabel("Calibrate f2")
        self.cali_f2_tight_button = QPushButton("Tightening f2")
        self.cali_f2_loosen_button = QPushButton("Loosensing f2")
        self.hbox_cali_f2 = QVBoxLayout()
        self.hbox_cali_f2.addWidget(self.cali_f2_tight_button)
        self.hbox_cali_f2.addWidget(self.cali_f2_loosen_button)

        # Calibrate f3 row
        self.cali_f3_label = QLabel("Calibrate f3")
        self.cali_f3_tight_button = QPushButton("Tightening f3")
        self.cali_f3_loosen_button = QPushButton("Loosensing f3")
        self.hbox_cali_f3 = QVBoxLayout()
        self.hbox_cali_f3.addWidget(self.cali_f3_tight_button)
        self.hbox_cali_f3.addWidget(self.cali_f3_loosen_button)

        # Calibrate preshape row
        self.cali_k1_label = QLabel("Calibrate preshape")
        self.cali_k1_tight_button = QPushButton("Separate Fingers")
        self.cali_k1_loosen_button = QPushButton("Bring Fingers Together")
        self.hbox_cali_k1 = QVBoxLayout()
        self.hbox_cali_k1.addWidget(self.cali_k1_tight_button)
        self.hbox_cali_k1.addWidget(self.cali_k1_loosen_button)

        # Calibrate Thumb rotation
        # Calibrate preshape row
        self.cali_k2_label = QLabel("Calibrate Thumb")
        self.cali_k2_clock_button = QPushButton("Rotate Clockwise")
        self.cali_k2_counterclock_button = QPushButton("Rotate Counterclockwise")
        self.hbox_cali_k2 = QVBoxLayout()
        self.hbox_cali_k2.addWidget(self.cali_k2_clock_button)
        self.hbox_cali_k2.addWidget(self.cali_k2_counterclock_button)


        #Instructions
        self.finger_instructions = QLabel("\nAlign Fingers to Appropriate Zero Points\n")
        self.thumb_instruction = QLabel("\n\nRotate thumb to approximately -60 degrees with respect to the desired center position")


############ Adding rows and set up singal for button ####################################################
        #QFormLayout similar to HBox but you know it look like form, add everything to FormLayout
        self.fbox = QFormLayout()
        self.fbox.addRow(self.finger_instructions)
        self.fbox.addRow(self.cali_f1_label,self.hbox_cali_f1)
        self.fbox.addRow(self.cali_f2_label,self.hbox_cali_f2)
        self.fbox.addRow(self.cali_f3_label,self.hbox_cali_f3)
        self.fbox.addRow(self.cali_k1_label,self.hbox_cali_k1)
        self.fbox.addRow(self.thumb_instruction)
        self.fbox.addRow(self.cali_k2_label,self.hbox_cali_k2)


        # Add connect signal to f1 tight and loosen button
        self.cali_f1_tight_button.clicked.connect(self.handle_cali_f1_tight)
        self.cali_f1_loosen_button.clicked.connect(self.handle_cali_f1_loosen)

        # Add connect signal to f1 tight and loosen button
        self.cali_f2_tight_button.clicked.connect(self.handle_cali_f2_tight)
        self.cali_f2_loosen_button.clicked.connect(self.handle_cali_f2_loosen)

        # Add connect signal to f1 tight and loosen button
        self.cali_f3_tight_button.clicked.connect(self.handle_cali_f3_tight)
        self.cali_f3_loosen_button.clicked.connect(self.handle_cali_f3_loosen)

        # Add connect signal to f1 tight and loosen button
        self.cali_k1_tight_button.clicked.connect(self.handle_cali_k1_tight)
        self.cali_k1_loosen_button.clicked.connect(self.handle_cali_k1_loosen)

        self.cali_k2_clock_button.clicked.connect(self.handle_cali_k2_clock)
        self.cali_k2_counterclock_button.clicked.connect(self.handle_cali_k2_counterclock)

######### Set up window ###################################################################################
        #Set the widget to layout and show the widget
        self.setLayout(self.fbox)
        self.setWindowTitle("Calibration")
        self.dumbnum = 0
        self.show()

########## Tighten and Loosen Button Function for all four motor ##########################################
########## Tighten and Loosen for f1 ######################################################################

    def handle_cali_f1_tight(self):
        finger_sel = 1 # 1 is motor f1
        b = 0 # 0 is tight, 1 is loosen
        calibrate_hand(self, finger_sel, b)

    def handle_cali_f1_loosen(self):
        finger_sel = 1 # 1 is motor f1
        b = 1 # 0 is tight, 1 is loosen
        calibrate_hand(self, finger_sel, b)

########## Tighten and Loosen for f2 ######################################################################
    def handle_cali_f2_tight(self):
        finger_sel = 2 # 1 is motor f1
        b = 0 # 0 is tight, 1 is loosen
        calibrate_hand(self, finger_sel, b)


    def handle_cali_f2_loosen(self):
        finger_sel = 2 # 1 is motor f1
        b = 1 # 0 is tight, 1 is loosen
        calibrate_hand(self, finger_sel, b)

########## Tighten and Loosen for f3 ######################################################################
    def handle_cali_f3_tight(self):
        finger_sel = 3 # 1 is motor f1
        b = 0 # 0 is tight, 1 is loosen
        calibrate_hand(self, finger_sel, b)

    def handle_cali_f3_loosen(self):
        finger_sel = 3 # 1 is motor f1
        b = 1 # 0 is tight, 1 is loosen
        calibrate_hand(self, finger_sel, b)


########## Tighten and Loosen for k1 ######################################################################
    def handle_cali_k1_tight(self):
        finger_sel = 4 # 1 is motor f1
        b = 0 # 0 is tight, 1 is loosen
        calibrate_hand(self, finger_sel, b)

    def handle_cali_k1_loosen(self):
        finger_sel = 4 # 1 is motor f1
        b = 1 # 0 is tight, 1 is loosen
        calibrate_hand(self, finger_sel, b)

#############################################################################################################
#to calibrate thumb rotation, set to an angle of approximately -45 degrees: negative extreme of all movement
    def handle_cali_k2_clock(self):
        finger_sel = 5 # 1 is motor f1
        b = 0 # 0 is tight, 1 is loosen
        calibrate_hand(self, finger_sel, b)
    def handle_cali_k2_counterclock(self):
        finger_sel = 5 # 1 is motor f1
        b = 1 # 0 is tight, 1 is loosen
        calibrate_hand(self, finger_sel, b)



def calibrate_hand(self, finger_sel, loosen):
    try:
        rospy.wait_for_service('/send_two_int', timeout=2)
        send_two_int = rospy.ServiceProxy('/send_two_int', SendTwoInt)
        resp1 = send_two_int(finger_sel, loosen)

    except Exception:
        print("\nERROR: Could not communicate with hand. Check cable connection")
