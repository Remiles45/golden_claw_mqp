import rospy
from python_qt_binding.QtWidgets import * #QWidget, QToolTip,QPushButton,QLabel,QGridLayout,QLineEdit
from python_qt_binding.QtGui import * #QFont,QPalette, QColor

from std_msgs.msg import String
from reflex_msgs.msg import PoseCommand
from std_srvs.srv import Empty
from rqt_service.srv import SendTwoInt
class CalibrationWidget(QWidget):

    def __init__(self):
        super(CalibrationWidget, self).__init__()
        #self.command_pub = rospy.Publisher('/reflex_sf/command_position', PoseCommand, queue_size=1)
        #rospy.init_node('listener', anonymous=True)

        self.initUI()

    def initUI(self):


########### Calibrate section ############################################################################
        # Calibrate f1 row
        self.cali_f1_label = QLabel("Calibrate f1")
        self.cali_f1_tight_button = QPushButton("Tightening f1")
        self.cali_f1_loosen_button = QPushButton("Loosensing f1")
        self.hbox_cali_f1 = QHBoxLayout()
        self.hbox_cali_f1.addWidget(self.cali_f1_tight_button)
        self.hbox_cali_f1.addWidget(self.cali_f1_loosen_button)

        # Calibrate f2 row
        self.cali_f2_label = QLabel("Calibrate f2")
        self.cali_f2_tight_button = QPushButton("Tightening f2")
        self.cali_f2_loosen_button = QPushButton("Loosensing f2")
        self.hbox_cali_f2 = QHBoxLayout()
        self.hbox_cali_f2.addWidget(self.cali_f2_tight_button)
        self.hbox_cali_f2.addWidget(self.cali_f2_loosen_button)

        # Calibrate f3 row
        self.cali_f3_label = QLabel("Calibrate f3")
        self.cali_f3_tight_button = QPushButton("Tightening f3")
        self.cali_f3_loosen_button = QPushButton("Loosensing f3")
        self.hbox_cali_f3 = QHBoxLayout()
        self.hbox_cali_f3.addWidget(self.cali_f3_tight_button)
        self.hbox_cali_f3.addWidget(self.cali_f3_loosen_button)

        # Calibrate preshape row
        self.cali_f4_label = QLabel("Calibrate preshape")
        self.cali_f4_tight_button = QPushButton("Tightening preshape")
        self.cali_f4_loosen_button = QPushButton("Loosensing preshape")
        self.hbox_cali_f4 = QHBoxLayout()
        self.hbox_cali_f4.addWidget(self.cali_f4_tight_button)
        self.hbox_cali_f4.addWidget(self.cali_f4_loosen_button)

        # Auto Calibration
        # self.cali_f5_label = QLabel("Auto-Calibration")
        # self.cali_f5_button = QPushButton("Auto-Calibration")

############ Adding rows and set up singal for button ####################################################
        #QFormLayout similar to HBox but you know it look like form, add everything to FormLayout
        self.fbox = QFormLayout()

        self.fbox.addRow(self.cali_f1_label,self.hbox_cali_f1)
        self.fbox.addRow(self.cali_f2_label,self.hbox_cali_f2)
        self.fbox.addRow(self.cali_f3_label,self.hbox_cali_f3)
        self.fbox.addRow(self.cali_f4_label,self.hbox_cali_f4)
       # self.fbox.addRow(self.cali_f5_label,self.cali_f5_button)

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
        self.cali_f4_tight_button.clicked.connect(self.handle_cali_f4_tight)
        self.cali_f4_loosen_button.clicked.connect(self.handle_cali_f4_loosen)

        # self.cali_f5_button.clicked.connect(self.handle_cali_f5)

        # self.list_control_save_button.clicked.connect(self.handle_list_control_save_button)
        # self.list_control_delete_button.clicked.connect(self.handle_list_control_delete_button)
        # self.list_control_go_button.clicked.connect(self.handle_list_control_go_button)

######### Set up window ###################################################################################
        #Set the widget to layout and show the widget
        self.setLayout(self.fbox)

        self.setWindowTitle("Test QT Layout")
        self.resize(640,480)
        self.dumbnum = 0
        self.show()

########## Tighten and Loosen Button Function for all four motor ##########################################
######## These handler function does not let me have any other input !!!!!!!!!!!! so i cant change
######## a, b when calling the function, so I have to make each handler for each button, need some refine
########## Tighten and Loosen for f1 ######################################################################
    def handle_cali_f1_tight(self):
        rospy.wait_for_service('/send_two_int')
        finger_sel = 1 # 1 is motor f1
        b = 0 # 0 is tight, 1 is loosen
        try:
            send_two_int = rospy.ServiceProxy('/send_two_int', SendTwoInt)
            resp1 = send_two_int(finger_sel, b)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def handle_cali_f1_loosen(self):
        rospy.wait_for_service('/send_two_int')
        finger_sel = 1 # 1 is motor f1
        b = 1 # 0 is tight, 1 is loosen
        try:
            send_two_int = rospy.ServiceProxy('/send_two_int', SendTwoInt)
            resp1 = send_two_int(finger_sel, b)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
########## Tighten and Loosen for f2 ######################################################################
    def handle_cali_f2_tight(self):
        rospy.wait_for_service('/send_two_int')
        finger_sel = 2 # 1 is motor f1
        b = 0 # 0 is tight, 1 is loosen
        try:
            send_two_int = rospy.ServiceProxy('/send_two_int', SendTwoInt)
            resp1 = send_two_int(finger_sel, b)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def handle_cali_f2_loosen(self):
        rospy.wait_for_service('/send_two_int')
        finger_sel = 2 # 1 is motor f1
        b = 1 # 0 is tight, 1 is loosen
        try:
            send_two_int = rospy.ServiceProxy('/send_two_int', SendTwoInt)
            resp1 = send_two_int(finger_sel, b)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

########## Tighten and Loosen for f3 ######################################################################
    def handle_cali_f3_tight(self):
        rospy.wait_for_service('/send_two_int')
        finger_sel = 3 # 1 is motor f1
        b = 0 # 0 is tight, 1 is loosen
        try:
            send_two_int = rospy.ServiceProxy('/send_two_int', SendTwoInt)
            resp1 = send_two_int(finger_sel, b)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def handle_cali_f3_loosen(self):
        rospy.wait_for_service('/send_two_int')
        finger_sel = 3 # 1 is motor f1
        b = 1 # 0 is tight, 1 is loosen
        try:
            send_two_int = rospy.ServiceProxy('/send_two_int', SendTwoInt)
            resp1 = send_two_int(finger_sel, b)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

########## Tighten and Loosen for f4 ######################################################################
###F4 is preshape? k1?
    def handle_cali_f4_tight(self):
        rospy.wait_for_service('/send_two_int')
        finger_sel = 4 # 1 is motor f1
        b = 0 # 0 is tight, 1 is loosen
        try:
            send_two_int = rospy.ServiceProxy('/send_two_int', SendTwoInt)
            resp1 = send_two_int(finger_sel, b)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def handle_cali_f4_loosen(self):
        rospy.wait_for_service('/send_two_int')
        finger_sel = 4 # 1 is motor f1
        b = 1 # 0 is tight, 1 is loosen
        try:
            send_two_int = rospy.ServiceProxy('/send_two_int', SendTwoInt)
            resp1 = send_two_int(finger_sel, b)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
#############################################################################################################
    def handle_cali_f5(self):
        try:
            auto_calibrate = rospy.ServiceProxy('/reflex_sf/auto_calibrate', Empty)
            resp1 = auto_calibrate()
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
