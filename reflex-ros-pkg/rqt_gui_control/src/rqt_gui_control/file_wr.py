import os
import sys
import rospkg
import rospy
from os import listdir
from os.path import isfile, join
from python_qt_binding.QtWidgets import *
from reflex_msgs.msg import PoseCommand, Command, VelocityCommand

rospack = rospkg.RosPack()
FILE_DIR = rospack.get_path('rqt_gui_control') + '/data'


class FileWriteRead(QWidget):
    def __init__(self):
        super(FileWriteRead, self).__init__()
        self.filename = []
        self.fileListWidget = QListWidget()

    def save_file(self,commands):
        """
            Encode a list of commands into a format that can be decoded
        """
        try:
            #prompt to edit filename/path
            filepath = QFileDialog.getSaveFileName(self, 'Save File', FILE_DIR)[0]
            name = os.path.basename(filepath)
            #write waypoint list to file
            if len(commands) > 0:
                print 'Saving waypoints... '
                with open(filepath, 'w') as file:
                    for point in commands:
                        #add indicator for each chunk of data
                        data = '//' + str(point.pose) + "***" + str(point.velocity)
                        file.write(data)
                print str(len(commands)) + ' waypoints saved to '+ name
            else:
                print "No waypoints to save"
        except Exception:
            name = False
        return name


    def readFile(self, file_name):
        """
            Decode the file with the given file name
            return the stored list of commands
        """
        pose_list = []
        try:

            file_path = "{}/{}".format(FILE_DIR, file_name)
            file = open(file_path,'r').read()
            # Divide file into poses
            data_chunks = file.split('//')
            data_chunks.pop(0) #the file gets saved with an extra // trigger
            for command in data_chunks:
                p_or_v = command.split('***')
                cnt = 0
                for test in p_or_v:
                    # divide each pose up by commands per finger
                    f1,f2,f3,k1,k2 = test.split('\n')
                    # choose only the numerical chunk of command and convert to float
                    tar_f1 = float(f1.split(': ')[1])
                    tar_f2 = float(f2.split(': ')[1])
                    tar_f3 = float(f3.split(': ')[1])
                    tar_k1 = float(k1.split(': ')[1])
                    tar_k2 = float(k2.split(': ')[1])
                    if cnt == 0:
                        pose0 = PoseCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_k1,k2=tar_k2)
                        cnt = cnt + 1
                    else:
                        velocity0 = VelocityCommand(f1=tar_f1,f2=tar_f2,f3=tar_f3,k1=tar_k1,k2=tar_k2)

                cmd = Command(pose=pose0,velocity=velocity0)
                pose_list.append(cmd)
        except AttributeError:
            error_msg = QErrorMessage(self)
            error_msg.setWindowTitle("File Error")
            error_msg.showMessage("Please Select A File to Execute")
            pose_list = False

        except Exception:
            error_msg2 = QErrorMessage(self)
            error_msg2.setWindowTitle("File Error")
            error_msg2.showMessage("Could not load file")
            pose_list = False
        return pose_list
