# -*- coding: utf-8 -*-
"""
Created on Sat Nov 12 16:41:53 2022

@author: Annie_007
"""

import sys
sys.path.append('/home/pi/env/lib/python3.7/site-packages') #添加树莓派中包的环境路径

import time
import threading

time.sleep(10)
    
if __name__ == '__main__':
    threads = []
    
    import automanage
    t1 = threading.Thread(target=automanage.drive)  #并行运行——驾驶程序
    threads.append(t1)
    
    import navigate
    import button
    t2 = threading.Thread(target=button.wait) #并行运行——状态转移
    threads.append(t2)    
    
    for t in threads:
        t.start()
