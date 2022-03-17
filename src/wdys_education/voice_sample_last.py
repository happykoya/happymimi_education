#!/usr/bin/env python
#-*- coding: utf-8 -*-

import roslib
import rospy
from voice_common_pkg.srv import *

tts_pub = rospy.ServiceProxy('/tts', TTS)
stt_pub = rospy.ServiceProxy('/stt_server', SpeechToText)


def main():
    cnt = 0
    WDYS = rospy.ServiceProxy('/bf/conversation_srvserver',WhatDidYouSay)
    print "Are you ready?"
    tts_pub('Are you ready?')
    if str(stt_pub(short_str=True).result_str) == 'yes':
        for i in range(10):
            tts_pub('Talk to me.')
            result = WDYS().result
            if result == True:
                cnt += 1

    print cnt

if __name__ == '__main__':
    rospy.init_node('voice_sample_last')
    main()
