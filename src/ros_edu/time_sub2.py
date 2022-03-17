#!/usr/bin/env python
import rospy
from education.msg import Date  #changed

def callback(message):
    print("date : %d , time : %f" % (message.date,message.time) )  #changed

if __name__ == "__main__":
    rospy.init_node('time_sub')
    sub = rospy.Subscriber('Date_and_Time', Date , callback)  #changed
    rospy.spin()
