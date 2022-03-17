#!/usr/bin/env python
#-*- coding: utf-8 -*-

import roslib
import rospy
from voice_common_pkg.srv import *
import Levenshtein

tts_pub = rospy.ServiceProxy('/tts', TTS)
stt_pub = rospy.ServiceProxy('/stt_server', SpeechToText)

def speak(sentence):
    tts_pub(sentence)
    return

def speechRecog():
    return stt_pub(short_str=True)

def main():
    result = speechRecog().result_str
    rsu = str(result)
    speak(rsu)
    print(rsu)
    result2 = speechRecog().result_str
    rsu2 = str(result2)
    speak(rsu2)
    print(rsu2)

    print(Levenshtein.distance(rsu,rsu2))


if __name__ == '__main__':
    rospy.init_node('voice_sample3')
    main()
