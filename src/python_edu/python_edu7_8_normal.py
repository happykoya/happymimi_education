#!/usr/bin/env python
#-*- coding: utf-8 -*-

#######################################
#title: python講習会第7,8回演習問題Easy
#author: Koya Okuse
#data: 2022/03/17
#######################################

def triangle(a):

    for i in range(a+1):
        for j in range(i):
            print('*', end='')

        print("")

a = int(input("数値を入力して下さい："))
triangle(a)
