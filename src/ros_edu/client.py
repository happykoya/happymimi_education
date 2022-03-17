#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool

if __name__ == "__main__":
    rospy.wait_for_service('service_call')
    try:
        service_call = rospy.ServiceProxy('service_call', SetBool)
        service_call(True)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)
