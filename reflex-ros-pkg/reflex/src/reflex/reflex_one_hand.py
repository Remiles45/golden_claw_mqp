#############################################################################
# Modified reflex_sf_hand.py from Right Hand Robotics to scalable to nth
# number of motors
#############################################################################

__author__ = 'Hoang Nguyen'
__maintainer__ = 'Hoang Nguyen'
__email__ = 'nphoang1102@gmail.com'


from string import lstrip
from math import pi as PI

from os.path import join
import yaml

from dynamixel_msgs.msg import JointState
import rospkg
import rospy
from std_srvs.srv import Empty
from rqt_service.srv import SendTwoInt

from reflex_hand import ReflexHand
from reflex_sf_motor import ReflexSFMotor
import reflex_msgs.msg


class ReflexOneHand(ReflexHand):
    def __init__(self):
        super(ReflexOneHand, self).__init__('/reflex_one', ReflexOneMotor)
        self.motor_names = rospy.get_param('~motors_list')
        print self.motor_names
        self.hand_state_pub = rospy.Publisher(self.namespace + '/hand_state',
                                              reflex_msgs.msg.Hand, queue_size=10)
        rospy.Service(self.namespace + '/calibrate_fingers', Empty, self.calibrate)
        rospy.Service(self.namespace + '/auto_calibrate', Empty, self.auto_calibrate)

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
        state = reflex_msgs.msg.Hand()
        motor_names = ('_f1', '_f2', '_f3', '_preshape')
        for i in range(4):
            state.motor[i] = self.motors[self.namespace + motor_names[i]].get_motor_msg()
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
    def auto_calibrate(self, data=None, speed=1.00, manual_start=True):

        # Variable to store zero pos
        preshape = "/reflex_sf_preshape"
        zero_pos = dict()

        # First thing, manually calibrate the preshape joint, only when needed
        if (manual_start):
            rospy.loginfo("Start manual calibrating %s.", preshape.lstrip("/"))
            command = raw_input("Type 't' to tighten motor, 'l' to loosen \
            motor, or 'q' to indicate that the zero point has been reached\n")
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

            # Zeroed the current position first
            self.motors[motor]._set_local_motor_zero_point()
            
            # Prompt user that we are currently auto calibrating motor
            rospy.loginfo("Start auto calibrating motor " + motor)

            # Slowly increment the joint position until overload reached
            while (self.motors[motor].get_load() < (self.motors[motor].get_load_threshold() * 0.5)):
                self.motors[motor].set_motor_velocity(speed)

            # Overload position reached, start computation
            if (self.motors[motor].get_flip()):
                offset = -PI * self.motors[motor].get_gear_ratio()
            else:
                offset = PI * self.motors[motor].get_gear_ratio()
            # Explanation on the math here: according to the documentation under reflex_msgs/Finger.msg, the
            # moving space of the finger is from 0 to pi, thus explained the mathametical model here

            # Computating the zeroed angle to send out
            zeroed_angle = self.motors[motor].get_current_raw_motor_angle() - offset
            zero_pos[motor.lstrip("/")] = dict(zero_point=zeroed_angle)

            # Bug fix: Write the zero position to the ros param, or else it would not take effect
            self.motors[motor]._set_local_motor_zero_point(zeroed_angle)

            # Got zero position, open up the hand to desired position and print out
            # cannot send negative speed as the zeroed angle will not be updated till then
            self.motors[motor].set_motor_angle(0.0)
            rospy.loginfo("%s done.", motor)

        # Write to .yaml file and prompt user
        self._write_zero_point_data_to_file('reflex_sf_zero_points.yaml', zero_pos)
        print "Auto-calibration complete, writing data to file"
        return [] # rospy will raise an error if I return None here, interesting



    def _write_zero_point_data_to_file(self, filename, data):
        rospack = rospkg.RosPack()
        reflex_sf_path = rospack.get_path("reflex")
        yaml_path = "yaml"
        file_path = join(reflex_sf_path, yaml_path, filename)
        with open(file_path, "w") as outfile:
            outfile.write(yaml.dump(data))

    def _zero_current_pose(self):
        data = dict(
            reflex_sf_f1=dict(zero_point=self.motors[self.namespace + '_f1'].get_current_raw_motor_angle()),
            reflex_sf_f2=dict(zero_point=self.motors[self.namespace + '_f2'].get_current_raw_motor_angle()),
            reflex_sf_f3=dict(zero_point=self.motors[self.namespace + '_f3'].get_current_raw_motor_angle()),
            reflex_sf_preshape=dict(zero_point=self.motors[self.namespace + '_preshape'].get_current_raw_motor_angle())
        )
        self._write_zero_point_data_to_file('reflex_sf_zero_points.yaml', data)


def main():
    # rospy.sleep(4.0)  # To allow services and parameters to load
    hand = ReflexOneHand()
    rospy.on_shutdown(hand.disable_torque)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        hand._publish_hand_state()
        r.sleep()


if __name__ == '__main__':
    main()
