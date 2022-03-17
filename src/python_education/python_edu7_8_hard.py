#!/usr/bin/env python
#-*- coding: utf-8 -*-

#######################################
#title: python講習会第7,8回演習問題Easy
#author: Koya Okuse
#data: 2022/03/17
#######################################

def make_chocolate(small, big, goal):
  
  result = 0
  for i in range(big):
    if goal >= 5:
      goal = goal - 5
      if goal == 0:
        return 0
    
    elif goal == 0:
      return 0
    
    else:
      break
    
  for j in range(small):
    goal = goal - 1
    if goal == 0:
      return j+1 
      
  if goal > 0:
    return -1
  
print(make_chocolate(4, 1, 9))
print(make_chocolate(4, 1, 10))
print(make_chocolate(4, 1, 5))
print(make_chocolate(1, 2, 5)) 
print(make_chocolate(6, 1, 10)) 
