#!/usr/bin/python
#-*-coding:utf-8-*-
import RPi.GPIO as GPIO  #导入RPi.GPIO模块

import time   #导入时间模块，用于延时
GPIO.setwarnings(False)  #屏蔽警告信息
#---第一步：设置引脚编码模式
GPIO.setmode(GPIO.BCM)  #设置引脚为BCM编码
#---第二步：设置引脚方向----
KEY = 18   #定义18脚作为按键使用（可更改）
BEEP = 27   #定义27脚为蜂鸣器使用
GPIO.setup(KEY,GPIO.IN)  #设置按键为输入
GPIO.setup(BEEP,GPIO.OUT,initial=GPIO.LOW)  #设置BEEP为输出，并且初始化为低电平
#----第三步：循环判断按键是否按下

def wait():
    Flag = 0 #蜂鸣器可修改
    while 1:
        if GPIO.input(KEY)==1:#按下（该按键模块，当松开的时候是低电平，按下时高电平）
           time.sleep(0.02)  #延时20ms  绕过抖动区间，为了防抖
           if(GPIO.input(KEY)==1):    #再次判断是否在按下的状态(判断是否在稳定区间)
              while(GPIO.input(KEY)==1): #等待松手
                   pass   #占位行
              
              #实现按键要做的事情-调用佳琪的导航模块
              print('Get Lost...')
              import navigate
              navigate.dao_hang()
              break
          
              if Flag == 0:
                 GPIO.output(BEEP,GPIO.HIGH)
                 Flag = 1
              else:
                 GPIO.output(BEEP,GPIO.LOW)
                 Flag = 0
    return

if __name__ == '__main__':
    wait()