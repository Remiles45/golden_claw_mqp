import rospy
from python_qt_binding.QtWidgets import * #QWidget, QToolTip,QPushButton,QLabel,QGridLayout,QLineEdit


class GloveWidget(QWidget):
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
            desired_pose = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_f4,k2=tar_k2)
            vel = VelocityCommand(f1=0.1,f2=0.1,f3=0.1,k1=0.1,k2=0.1)
            cmd = Command(pose=desired_pose,velocity=vel)
            self.moveHandtoPose(command=cmd) #desired_pose, velocity0)
