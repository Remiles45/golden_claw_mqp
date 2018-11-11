import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from ui_design import *

widget = QMainWindow()
class RobotHandPlugin(Plugin):
    def __init__(self, context):
        super(RobotHandPlugin, self).__init__(context)
        context.ui_design = SetUpUI()
        context.add_widget(context.ui_design)


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
