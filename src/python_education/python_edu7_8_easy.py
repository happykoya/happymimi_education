#!/usr/bin/env python
#-*- coding: utf-8 -*-

#######################################
#title: python講習会第7,8回演習問題Easy
#author: Koya Okuse
#data: 2022/03/17
#######################################

def list_func(array):

    print("リストの長さは"+str(len(array))+"です。")
    print("リストaの最大値は"+str(max(array))+"です。")
    print("リストaに含まれる最小値は"+str(min(array))+"です")
    print("リストaの合計は"+str(sum(array))+"です")
    print("このリストをソートすると"+str(sorted(array))+"です")


a = [4, 8, 3, 4, 1]
list_func(a)
