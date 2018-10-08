import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from manual_cntrl_widget import ManualHandControlWidget
# from my_widget_vel_pos import MyWidgetVelPos
# from my_widget_vel import MyWidgetVel
# from calibration_widget import CalibrationWidget
from server_gui import ServerGui
# from grasp_cntrl_widget import GraspControlWidget

import socket               # Import socket module

class RobotHandPlugin(Plugin):

    def __init__(self, context):
        super(RobotHandPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('RobotHandPlugin')


        # Create QWidget
        self._widget = ManualHandControlWidget()
        self._widget.setObjectName('MyPosUI')

        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
