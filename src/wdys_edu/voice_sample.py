#!/usr/bin/env python
# -*- coding: utf-8 -*-
#######################
#title: 演習問題１
#author: Koya Okuse
#data: 2021/09/28
#######################

import rospy
import sys
from std_msgs.msg import String
import roslib

sys.path.insert(0, '/home/athome/catkin_ws/src/voice_common_pkg/')
from voice_common_pkg.srv import *
import Levenshtein

tts_pub = rospy.ServiceProxy('/tts', TTS)
stt_pub = rospy.ServiceProxy('/stt_server', SpeechToText)

def speak(sentense):
    tts_pub(sentense)
    return

def SpeechRecog():
    return stt_pub(short_str=True)

def main():
    str1 = 'hello'
    speak(str1)
    speak('My name is Koya Okuse')
    if SpeechRecog() ==  'hello':
        print 'good!!'

if __name__ == '__main__':
    stt_pub(short_str = True)
    main()
