import rospy
from python_qt_binding.QtWidgets import * #QWidget, QToolTip,QPushButton,QLabel,QGridLayout,QLineEdit
from python_qt_binding.QtGui import * #QFont,QPalette, QColor

from std_msgs.msg import String
from reflex_msgs.msg import PoseCommand
from std_srvs.srv import Empty
from rqt_service.srv import SendTwoInt

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import socket               # Import socket module

fig = plt.figure()
ax1 = fig.add_subplot(6,1,1)
ax11 = fig.add_subplot(6,1,2)
ax12 = fig.add_subplot(6,1,3)
ax13 = fig.add_subplot(6,1,4)
ax14 = fig.add_subplot(6,1,5)
ax15 = fig.add_subplot(6,1,6)

class ServerGui(QWidget):
    
    def __init__(self):
        super(ServerGui, self).__init__()
        #self.command_pub = rospy.Publisher('/reflex_sf/command_position', PoseCommand, queue_size=1)
        #rospy.init_node('listener', anonymous=True)

        self.initUI()
        
    def initUI(self):

########### Calibrate section ############################################################################
        # Server row
        self.gcali_label = QLabel("Glove Calibrate")
        self.gcali_start_button = QPushButton("Unbend all finger")
        self.gcali_close_button = QPushButton("Bend all finger")
        self.hbox_cali_f1 = QHBoxLayout()
        self.hbox_cali_f1.addWidget(self.gcali_start_button)
        self.hbox_cali_f1.addWidget(self.gcali_close_button)

##########################################################################################################
        self.lp_label = QLabel("Live Plot")
        self.lp_start_button = QPushButton("Show")
        self.lp_close_button = QPushButton("Close")
        self.hbox_cali_f2 = QHBoxLayout()
        self.hbox_cali_f2.addWidget(self.lp_start_button)
        self.hbox_cali_f2.addWidget(self.lp_close_button)

##########################################################################################################
        self.cal_label = QLabel("Calculation")
        self.cal_start_button = QPushButton("Calculation")
        self.cal_close_button = QPushButton("Close")
        self.hbox_cali_f3 = QHBoxLayout()
        self.hbox_cali_f3.addWidget(self.cal_start_button)
        self.hbox_cali_f3.addWidget(self.cal_close_button)

##########################################################################################################
        # self.listPose = []
        # pose0 = PoseCommand(f1=0.0,f2=0.0,f3=0.0,preshape=0.0)
        # self.listPose.append(pose0)
        # #Test List view
        # self.listWidget = QListWidget()
        
        # item = QListWidgetItem("Pos('%2.2f','%2.2f','%2.2f','%2.2f')" % (pose0.f1,pose0.f2,pose0.f3,pose0.preshape))
        # self.listWidget.addItem(item)

        # self.listlabel = QLabel("List waypoint")

        # #List Control
        # self.list_control_label = QLabel("Waypoint Control")
        # self.list_control_save_button = QPushButton("Save")
        # self.list_control_delete_button = QPushButton("Remove")
        # self.list_control_go_button = QPushButton("Go waypoints")
        # self.list_control = QHBoxLayout()
        # self.list_control.addWidget(self.list_control_save_button)
        # self.list_control.addWidget(self.list_control_delete_button)
        # self.list_control.addWidget(self.list_control_go_button)
############ Adding rows and set up singal for button ####################################################
        #QFormLayout similar to HBox but you know it look like form, add everything to FormLayout
        self.fbox = QFormLayout()
        # self.fbox.addRow(self.finger_label_1,self.hbox_f1)
        # self.fbox.addRow(self.finger_label_2,self.hbox_f2)
        # self.fbox.addRow(self.finger_label_3,self.hbox_f3)
        # self.fbox.addRow(self.finger_label_4,self.hbox_f4)
        # self.fbox.addRow(self.coupling_label,self.hbox_tick)
        # self.fbox.addRow(self.command_label,self.hbox_command)
        self.fbox.addRow(self.gcali_label,self.hbox_cali_f1)
        self.fbox.addRow(self.lp_label,self.hbox_cali_f2)
        self.fbox.addRow(self.cal_label,self.hbox_cali_f3)
       
        # self.fbox.addRow(self.listlabel,self.listWidget)
        # self.fbox.addRow(self.list_control_label,self.list_control)

        # # Connect singal when slider change to function respectively to change value of label
        # self.finger_slider_1.valueChanged.connect(self.valuechange1)
        # self.finger_slider_2.valueChanged.connect(self.valuechange2)
        # self.finger_slider_3.valueChanged.connect(self.valuechange3)
        # self.finger_slider_4.valueChanged.connect(self.valuechange4)
        
        # # Add connect signal to Button Go, Cancel and Reset
        # self.go_button.clicked.connect(self.handleButtonGo)
        # self.home_button.clicked.connect(self.handleButtonHome)
        # self.re_button.clicked.connect(self.handleButtonReset)
        self.isServerOn = 0;
        # Add connect signal to f1 tight and loosen button
        self.gcali_start_button.clicked.connect(self.gcali_start_button_handle)
        self.gcali_close_button.clicked.connect(self.gcali_close_button_handle)
        
        self.lp_start_button.clicked.connect(self.lp_start_button_handle)
        self.lp_close_button.clicked.connect(self.lp_close_button_handle)
       

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
    def gcali_start_button_handle(self):
        print("In here: Open Server")
        self.isServerOn = 1
        print("Start listening to data")
        s = None
        s = socket.socket()         # Create a socket object
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host = '130.215.206.158' # Get local machine name
        port = 12345                # Reserve a port for your service.
        s.bind((host, port))        # Bind to the port
        print(host, "  and  ", port)
        with open('/home/mqp-team/hand_mqp_script/unbend.txt', 'w') as file:
            file.write('\n')
        s.listen(5)                 # Now wait for client connection.
        i = 0
        while (i<20):
            c, addr = s.accept()
            #print("Debug: Acceped connection with " + addr[0] + ":" + str(addr[1]))
            #print("Debug: Server is listening")
            data = c.recv(1024).decode()
            print("Client sends: ", data)
            with open('/home/mqp-team/hand_mqp_script/unbend.txt', 'a') as file:
                file.write(data)
            mess = 'Thank you for connecting'
            c.send(mess.encode())
            c.close()                # Close the connection
            i = i+1
        s.shutdown(socket.SHUT_RDWR)
        s.close()
        s = None

    def gcali_close_button_handle(self):
        print("In here: Close Server")
        self.isServerOn = 0
        print("Start listening to data")
        s = None
        s = socket.socket()         # Create a socket object
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host = '130.215.206.158' # Get local machine name
        port = 12345                # Reserve a port for your service.
        s.bind((host, port))        # Bind to the port
        print(host, "  and  ", port)
        with open('/home/mqp-team/hand_mqp_script/bend.txt', 'w') as file:
            file.write('\n')
        s.listen(5)                 # Now wait for client connection.
        i = 0
        while (i<20):
            c, addr = s.accept()
            #print("Debug: Acceped connection with " + addr[0] + ":" + str(addr[1]))
            #print("Debug: Server is listening")
            data = c.recv(1024).decode()
            print("Client sends: ", data)
            with open('/home/mqp-team/hand_mqp_script/bend.txt', 'a') as file:
                file.write(data)
            mess = 'Thank you for connecting'
            c.send(mess.encode())
            c.close()                # Close the connection
            i = i+1
        s.shutdown(socket.SHUT_RDWR)
        s.close()

        s = None

    def lp_start_button_handle(self):
        print("In here: Live Plot")
        ani = animation.FuncAnimation(fig, animate,interval=1000)
        plt.show()


    def lp_close_button_handle(self):
        print("In here: Close Live Plot Forever")
        plt.close()


def animate(i):
    graph_data = open('/home/mqp-team/hand_mqp_script/file.txt','r').read()
    lines = graph_data.split('\n')
    d1new = [0,0,0,0,0]
    d1 = []
    d2 = []
    d3 = []
    d4 = []
    d5 = []
    index1 = [0,0,0,0,0]
    index = []
    icount = 5
    for line in lines:
        if len(line) > 1:
            s1,s2,s3,s4,s5 = line.split(',')
            if (s1 !='9999'):
                #print(line.split(','))
                d1input = (int(s1)+d1new[icount-1]+d1new[icount-2]+d1new[icount-3]+d1new[icount-4]+d1new[icount-5])/6
                d1new.append(d1input)
                d1.append(int(s1))
                d2.append(int(s2))
                d3.append(int(s3))
                d4.append(int(s4))
                d5.append(int(s5))
                index.append(icount)
                index1.append(icount)
                icount = icount+1
    ax1.clear()
    ax1.plot(index1,d1new)
    ax11.clear()
    ax11.plot(index,d1)
    ax12.clear()
    ax12.plot(index,d2)
    ax13.clear()
    ax13.plot(index,d3)
    ax14.clear()
    ax14.plot(index,d4)
    ax15.clear()
    ax15.plot(index,d5)