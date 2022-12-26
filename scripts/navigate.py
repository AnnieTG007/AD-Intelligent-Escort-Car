#-*- coding: utf-8 -*-

# 加入gps
import math
import time
import coordTransform_utils
import os
import re
import sys
from math import radians, cos, sin, asin, sqrt
from omxplayer import OMXPlayer
from pathlib import Path
from time import sleep


def dao_hang():
    # GPS控制线程，将硬件检测到的经纬度生成全局变量方便小车实时受控
    class GPS_Control:
        def __init__(self):
            self.latitude = 39.576271
            self.longitude = 116.211059

        def run_threaded(self):
            return self.latitude, self.longitude

        def update(self):
            self.latitude  = self.run_lat()
            self.longitude = self.run_log()

        def run_lat(self):
            # GPS模块读取数据
            import L76X
            x = L76X.L76X()
            x.L76X_Set_Baudrate(9600)
            x.L76X_Send_Command(x.SET_NMEA_BAUDRATE_115200)
            time.sleep(0.5)
            x.L76X_Set_Baudrate(115200)
            x.L76X_Gat_GNRMC()
            if x.Status == 1:
                print('Already positioned')
            else:
                print('No positioning')

            print('Time %d:' % x.Time_H + '%d:' % x.Time_M + '%d' % x.Time_S)
            x.L76X_Baidu_Coordinates(x.Lat, x.Lon)
            amap = coordTransform_utils.bd09_to_gcj02(x.Lon_Baidu, x.Lat_Baidu)
            print('AMap coordinate %f' % amap[0] + ',%f' % amap[1])
            return format(amap[1], '.6f')
            
        def run_log(self):
             # GPS模块读取数据
             import L76X
             x = L76X.L76X()
             x.L76X_Set_Baudrate(9600)
             x.L76X_Send_Command(x.SET_NMEA_BAUDRATE_115200)
             time.sleep(0.5)
             x.L76X_Set_Baudrate(115200)
             x.L76X_Gat_GNRMC()
             if x.Status == 1:
                 print('Already positioned')
             else:
                 print('No positioning')

             print('Time %d:' % x.Time_H + '%d:' % x.Time_M + '%d' % x.Time_S)
             x.L76X_Baidu_Coordinates(x.Lat, x.Lon)
             amap = coordTransform_utils.bd09_to_gcj02(x.Lon_Baidu, x.Lat_Baidu)
             print('AMap coordinate %f' % amap[0] + ',%f' % amap[1])
             return format(amap[0], '.6f')  

    '''
    教一门口：经度116.211059 纬度39.576271        [116.211059],[39.576271]
    第一个左转点前：经度116.210724 纬度39.576207   [116.210724],[39.576207]
    第一个左转点后：经度116.210717 纬度39.576224   [116.210717],[39.576224]
    第二个左转点前：经度116.210743 纬度39.575728   [116.210743],[39.575728]
    第二个左转点后：经度116.210709 纬度39.575711   [116.210709],[39.575711]
    教二门口：经度116.211116 纬度39.575689        [116.211116],[39.575689]
    '''

    class Drive_gps:
        def __init__(self):
            self.num = 1
        
        def play(self,name):
            VIDEO_PATH = Path(name)
            player = OMXPlayer(VIDEO_PATH, args = '-b -o local' )
            sleep(1.5)
            player.quit()
        
        def run(self):
            while self.num < 4:
                pos = GPS_Control()
                pos.update()
                longitudeG = pos.longitude
                latitudeG = pos.latitude
                
                turnPos = [[[116.300222], [40.043158]], [[116.300535], [40.042583]]]
                destination = [[116.301623], [40.042810]]  # 9单元门口
                
                # 球坐标系计算当前所处位置和下一个转折点之间的距离（单位米）
                if (self.num < 3):
                    lon1, lat1, lon2, lat2 = map(radians, [float(longitudeG), float(latitudeG), turnPos[self.num - 1][0][0],
                                                            turnPos[self.num - 1][1][0]])
                if (self.num == 3):
                    lon1, lat1, lon2, lat2 = map(radians, [float(longitudeG), float(latitudeG), destination[0][0],
                                                            destination[1][0]])
                d_lon = lon2 - lon1
                d_lat = lat2 - lat1
                a = sin(d_lat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(d_lon / 2) ** 2
                c = 2 * asin(sqrt(a))
                r = 6371  # 地球平均半径，单位为公里
                distance = c * r * 1000
                
                print(distance)
                print(longitudeG)
                print(latitudeG)
                print('num')
                print(self.num)
                
                std = 15
                
                if self.num == 1:
                    if distance < std:  # 距离转折点距离小于2米
                        self.num = self.num + 1
                        self.play('/home/pi/mycar/audio/turnright.mp3')
                elif self.num == 2:
                    if distance > std:
                        self.play('/home/pi/mycar/audio/gostraight.mp3')
                    else:  # 距离转折点距离小于2米
                        self.play('/home/pi/mycar/audio/turnleft.mp3')
                        self.num = self.num + 1
                elif self.num == 3:
                    if distance < std:  # 距离终点距离小于2米
                        self.play('/home/pi/mycar/audio/home.mp3')
                        self.num = 4
                        
            return
    
    car = Drive_gps()
    car.run()

# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    dao_hang()

