This folder contain Arduino Code upload to the new glove hardware, the Arduino run ros packages on itself and send 3 int back to a topic. Then reflex can just listen to that topic for control use.

Run roscore then do with ttyUS0 is the usb port for arduino nano
	rosrun rosserial_python serial_node.py /dev/ttyUSB0

Then the data will be stream at chatter
	rostopic echo chatter
