#!/usr/bin/env python
# -*- coding: utf-8 -*-
#######################
#title: 音声認識テスト
#author: Koya Okuse
#data: 2021/08/06
#######################

import rospy
import roslib
from voice_common_pkg.srv import *
import lp_wdys

tts_pub = rospy.ServiceProxy('/tts', TTS)
stt_pub = rospy.ServiceProxy('/speech_recog', SpeechRecog)

file_name = "WRS2020_whatdidyousay_questions.yaml"
file_path = roslib.packages.get_pkg_dir("basic_fanc") + "/config/" + file_name
lp = lp_wdys.Selector(file_path)


def speak():
    tts_pub(sentense)
    return

def SpeechRecog():
    return stt_pub()

def voice_question():
    ros_response = WhatDidYouSayResponse()
    f = open('/home/athome/catkin_ws/src/mimi_common_pkg/config/kouya.yaml')
    result_list = load(f)
    f.close()

    # recognition result
    # sentence = speechRecog()
    # print(sentence)
    # result_list = lp.checker(sentence.result)

    if all(result_list):
        speak("Talk to me")

        speak("Answer is")
        speak(result_list[1])
        ros_response.result = True
    else:
        speak("Sorry I couldn't be recognized")
        ros_response.result = False

    return ros_response

def rosConfig():
    rospy.init_node('voice_question')
    rospy.Service('/bf/voice_question', WhatDidYouSay, conversation)
    rospy.loginfo('Ready to  voice_question')
    rospy.spin()



if __name__ == '__main__':
    rosConfig()
