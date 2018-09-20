#!/usr/bin/env python
import rospy
from rqt_service.srv import SendTwoInt

def handle_data(data):
    print "Motor: " + str(data.a)
    if data.b == 0:
    	print "tightening"
    else:
    	print "loosening"
    return 0

def send_two_ints_server():
    print "Start Service"
    rospy.init_node('test_server')
    s = rospy.Service('send_two_int', SendTwoInt, handle_data)
    print "Ready to receive two ints."
    rospy.spin()

if __name__ == "__main__":
    send_two_ints_server()