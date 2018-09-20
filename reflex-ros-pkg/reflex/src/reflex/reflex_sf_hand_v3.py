#!/usr/bin/env python

#############################################################################
# Copyright 2015 Right Hand Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#############################################################################

__author__ = 'Eric Schneider'
__copyright__ = 'Copyright (c) 2015 RightHand Robotics'
__license__ = 'Apache License 2.0'
__maintainer__ = 'RightHand Robotics'
__email__ = 'reflex-support@righthandrobotics.com'

from string import lstrip

from os.path import join
import yaml

from dynamixel_msgs.msg import JointState
import rospkg
import rospy
from std_srvs.srv import Empty
from rqt_service.srv import SendTwoInt

from reflex_hand_v2 import ReflexHandV2
from reflex_sf_motor import ReflexSFMotor
import reflex_msgs.msg


class ReflexSFHandV2(ReflexHandV2):
    def __init__(self):
        super(ReflexSFHandV2, self).__init__('/reflex_sf', ReflexSFMotor)
        self.hand_state_pub = rospy.Publisher(self.namespace + '/hand_state',
                                              reflex_msgs.msg.Hand, queue_size=10)
        rospy.Service(self.namespace + '/calibrate_fingers', Empty, self.calibrate)
        rospy.Service('/send_two_int', SendTwoInt, self.gui_calibrate)
        rospy.Service(self.namespace + '/auto_calibrate', Empty, self.auto_calibrate)
        self.motornames = rospy.get_param('motor_name')

    def _receive_cmd_cb(self, data):
        self.disable_force_control()
        self.set_speeds(data.velocity)
        self.set_angles(data.pose)

    def _receive_angle_cmd_cb(self, data):
        self.disable_force_control()
        self.reset_speeds()
        self.set_angles(data)

    def _receive_vel_cmd_cb(self, data):
        self.disable_force_control()
        self.set_velocities(data)

    def _receive_force_cmd_cb(self, data):
        self.disable_force_control()
        self.reset_speeds()
        self.set_force_cmds(data)
        self.enable_force_control()

    def disable_torque(self):
        for ID, motor in self.motors.items():
            motor.disable_torque()

    def enable_torque(self):
        for ID, motor in self.motors.items():
            motor.enable_torque()

    def _publish_hand_state(self):
        # state = reflex_msgs.msg.Hand()
        # motor_names = ('_f1', '_f2', '_f3', '_preshape1', '_preshape2')
        # for i in range(5):
        #     state.motor[i] = self.motors[self.namespace + motor_names[i]].get_motor_msg()
        # self.hand_state_pub.publish(state)
        
        state = reflex_msgs.msg.Hand()
        for i in range(len(self.motornames)):
            state.motor[i] = self.motors[self.namespace + self.motornames[i]].get_motor_msg()
        self.hand_state_pub.publish(state)

    def calibrate(self, data=None):
        for motor in sorted(self.motors):
            rospy.loginfo("Calibrating motor " + motor)
            command = raw_input("Type 't' to tighten motor, 'l' to loosen \
motor, or 'q' to indicate that the zero point has been reached\n")
            while not command.lower() == 'q':
                if command.lower() == 't' or command.lower() == 'tt':
                    print "Tightening motor " + motor
                    self.motors[motor].tighten(0.35 * len(command) - 0.3)
                elif command.lower() == 'l' or command.lower() == 'll':
                    print "Loosening motor " + motor
                    self.motors[motor].loosen(0.35 * len(command) - 0.3)
                else:
                    print "Didn't recognize that command, use 't', 'l', or 'q'"
                command = raw_input("Tighten: 't'\tLoosen: 'l'\tDone: 'q'\n")
            rospy.loginfo("Saving current position for %s as the zero point", motor)
            self.motors[motor]._set_local_motor_zero_point()
        print "Calibration complete, writing data to file"
        self._zero_current_pose()
        return []


    # This function call when the service data get call to calibrate from gui_control
    def gui_calibrate(self, data):
        #base on motor varible in calibrate function, motor is reflex_sf + _f1
        command = 1
        if (data.a == 1):
            if (data.b == 0):
                print "Tightening motor f1"
                self.motors[self.namespace + '_f1'].tighten(0.35 * command - 0.3)
            else:
                print "Loosening motor f1"
                self.motors[self.namespace + '_f1'].loosen(0.35 * command - 0.3)

            self.motors[self.namespace + '_f1']._set_local_motor_zero_point()

        if (data.a == 2):
            if (data.b == 0):
                print "Tightening motor f2"
                self.motors[self.namespace + '_f2'].tighten(0.35 * command - 0.3)
            else:
                print "Loosening motor f2"
                self.motors[self.namespace + '_f2'].loosen(0.35 * command - 0.3)

            self.motors[self.namespace + '_f2']._set_local_motor_zero_point()

        if (data.a == 3):
            if (data.b == 0):
                print "Tightening motor f3"
                self.motors[self.namespace + '_f3'].tighten(0.35 * command - 0.3)
            else:
                print "Loosening motor f3"
                self.motors[self.namespace + '_f3'].loosen(0.35 * command - 0.3)

            self.motors[self.namespace + '_f3']._set_local_motor_zero_point()
        if (data.a == 4):
            if (data.b == 0):
                print "Tightening motor _preshape"
                self.motors[self.namespace + '_preshape'].tighten(0.35 * command - 0.3)
            else:
                print "Loosening motor _preshape"
                self.motors[self.namespace + '_preshape'].loosen(0.35 * command - 0.3)

            self.motors[self.namespace + '_preshape']._set_local_motor_zero_point()

        print "Calibration complete, writing data to file"
        self._zero_current_pose()
        return 1

    # Motor autocalibration process
    def auto_calibrate(self, data=None):
        # Zeroed the current position first
        # self._zero_current_pose()

        # Variable to store zero pos
        preshape = "/reflex_sf_preshape"
        zero_pos = dict()

        # First thing, manually calibrate the preshape joint
#         rospy.loginfo("Start manual calibrating %s.", preshape.lstrip("/"))
#         command = raw_input("Type 't' to tighten motor, 'l' to loosen \
# motor, or 'q' to indicate that the zero point has been reached\n")
        while not command.lower() == 'q':
            if command.lower() == 't' or command.lower() == 'tt':
                print "Tightening motor " + preshape
                self.motors[preshape].tighten(0.35 * len(command) - 0.3)
            elif command.lower() == 'l' or command.lower() == 'll':
                print "Loosening motor " + preshape
                self.motors[preshape].loosen(0.35 * len(command) - 0.3)
            else:
                print "Didn't recognize that command, use 't', 'l', or 'q'"
            command = raw_input("Tighten: 't'\tLoosen: 'l'\tDone: 'q'\n")
        rospy.loginfo("Manual calibration done, start auto calibrate the rest of the fingers.")

        # Goes through the fingers first, the preshape is still tricky
        for motor in sorted(self.motors):
            if (motor == preshape):
                zero_pos[preshape.lstrip("/")] = dict(zero_point=self.motors[self.namespace + '_preshape'].get_current_raw_motor_angle())
                break

            # State we are currently auto calibrating motor
            rospy.loginfo("Start auto calibrating motor " + motor)

            # Slowly increment the joint position until overload reached
            while (self.motors[motor].get_load() < (self.motors[motor].get_load_threshold() * 0.5)):
                self.motors[motor].set_motor_velocity(1.25)

            # Open up again ?!
            # TODO: Actually do something meaningful here...
            self.motors[motor].set_motor_velocity(-1.25)
            if (self.motors[motor].get_flip()):
                offset = -4.7
            else:
                offset = 4.7
            zero_pos[motor.lstrip("/")] = dict(zero_point=self.motors[motor].get_current_raw_motor_angle() - offset)
            # print("Overload angle:", self.motors[motor].get_current_raw_motor_angle())
            rospy.loginfo("%s done.", motor)

        # End here, write calibration data to .yaml file and prompt user
        print "Auto-calibration complete, writing data to file"
        # print(zero_pos)
        yaml_path = rospy.get_param('yaml_zero_path')
        self._write_zero_point_data_to_file(yaml_path, data)
        # self._write_zero_point_data_to_file('reflex_sf_zero_points.yaml', zero_pos)
        return [] # rospy will raise an error if I return None here, interesting



    def _write_zero_point_data_to_file(self, filename, data):
        rospack = rospkg.RosPack()
        reflex_sf_path = rospack.get_path("reflex")
        yaml_path = "yaml"
        file_path = join(reflex_sf_path, yaml_path, filename)
        with open(file_path, "w") as outfile:
            outfile.write(yaml.dump(data))

    def _zero_current_pose(self):
        # data = dict(
        #     reflex_sf_f1=dict(zero_point=self.motors[self.namespace + '_f1'].get_current_raw_motor_angle()),
        #     reflex_sf_f2=dict(zero_poxint=self.motors[self.namespace + '_f2'].get_current_raw_motor_angle()),
        #     reflex_sf_f3=dict(zero_point=self.motors[self.namespace + '_f3'].get_current_raw_motor_angle()),
        #     reflex_sf_preshape=dict(zero_point=self.motors[self.namespace + '_preshape'].get_current_raw_motor_angle())
        # )
        data = dict()
        for motorname in self.motornames:
            name = self.namespace + motorname
            data.update({name:dict(zero_point=self.motors[name].get_current_raw_motor_angle())})
        yaml_path = rospy.get_param('yaml_zero_path')
        self._write_zero_point_data_to_file(yaml_path, data)


def main():
    rospy.sleep(4.0)  # To allow services and parameters to load
    # hand = ReflexSFHandV2()
    # rospy.on_shutdown(hand.disable_torque)
    test()
    # r = rospy.Rate(20)
    while not rospy.is_shutdown():
        # hand._publish_hand_state()
        # r.sleep()
        pass

def test():
    #hand = ReflexSFHandV2()
    testmsg = reflex_msgs.msg.PoseCommand(4.0,3.0,2.0,1.0)
    print(testmsg)
    x = getattr(testmsg,"f1")
    print(x)
    yaml_path = rospy.get_param('yaml_zero_path')
    print(yaml_path)

if __name__ == '__main__':
    main()
