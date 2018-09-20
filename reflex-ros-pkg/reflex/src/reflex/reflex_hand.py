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


import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_srvs.srv import Empty

import reflex_msgs.msg


class ReflexHand(object):
    def __init__(self, name, MotorClass):
        '''
        Assumes that "name" is the name of the hand with a preceding
        slash, e.g. /reflex_takktile or /reflex_sf
        '''
        self.namespace = name
        rospy.init_node('reflex_hand')

        # Identifier to check whether the controllers are spawned successfully or not
        self.controller_startup = False

        # Wait for the controller process to done completely
        rospy.Subscriber('controller_spawner_done_init', Bool, self.controller_status_cb)
        while not self.get_controller_startup_status():
            pass
        rospy.sleep(0.4) # brief delay just so all of the additional informations from the 
        # dxl packages get printed out

        # Start initiating motor controllers based on the input list from the launch file
        rospy.loginfo('Starting up the hand')
        self.input_motors = rospy.get_param('motors_list')
        self.motors = dict()
        self.motor_names = []
        for motor in self.input_motors:
            name = self.namespace + "_" + motor
            self.motors[name] = MotorClass(name)
            self.motor_names.append(name)

        # Setup subscriber for sending out messages to motors
        rospy.Subscriber(self.namespace + '/command',
                         reflex_msgs.msg.Command, self._receive_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_position',
                         reflex_msgs.msg.PoseCommand, self._receive_angle_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_velocity',
                         reflex_msgs.msg.VelocityCommand, self._receive_vel_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_motor_force',
                         reflex_msgs.msg.ForceCommand, self._receive_force_cmd_cb)
        rospy.loginfo('ReFlex hand has started, waiting for commands...')


    def _receive_cmd_cb(self, data):
        raise NotImplementedError

    def _receive_angle_cmd_cb(self, data):
        raise NotImplementedError

    def _receive_vel_cmd_cb(self, data):
        raise NotImplementedError

    def _receive_force_cmd_cb(self, data):
        raise NotImplementedError

    def set_angles(self, pose):
        for motor_name in self.input_motors:
            self.motors[self.namespace+"_"+motor_name].set_motor_angle(getattr(pose, motor_name))

    def set_velocities(self, velocity):
        for motor_name in self.input_motors:
            self.motors[self.namespace+"_"+motor_name].set_motor_angle(getattr(velocity, motor_name))

    def set_speeds(self, speed):
        for motor_name in self.input_motors:
            self.motors[self.namespace+"_"+motor_name].set_motor_angle(getattr(speed, motor_name))

    def set_force_cmds(self, torque):
        for motor_name in self.input_motors:
            self.motors[self.namespace+"_"+motor_name].set_motor_angle(getattr(torque, motor_name))

    def reset_speeds(self):
        for ID, motor in self.motors.items():
            motor.reset_motor_speed()

    def disable_force_control(self):
        for ID, motor in self.motors.items():
            motor.disable_force_control()
        rospy.sleep(0.05)  # Lets commands stop before allowing any other actions

    def enable_force_control(self):
        rospy.sleep(0.05)  # Lets other actions happen before beginning constant torque commands
        for ID, motor in self.motors.items():
            motor.enable_force_control()

    # Callback to update the status of the controller status
    def controller_status_cb(self, data):
        self.controller_startup = data.data

    # Helper function to check if the controller spawners has finished initiating
    def get_controller_startup_status(self):
        return self.controller_startup