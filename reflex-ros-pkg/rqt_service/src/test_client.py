import rospy
from rqt_service.srv import SendTwoInt

if __name__ == "__main__":
    rospy.wait_for_service('send_two_int')
    x = 1
    y = 2
    try:
        send_two_int = rospy.ServiceProxy('send_two_int', SendTwoInt)
        resp1 = send_two_int(x, y)
        print resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e