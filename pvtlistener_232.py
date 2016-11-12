#!/usr/bin/env python
# -*- coding: utf-8 -*-



"""
先初始化moveit！，话题订阅节点以及CAN；
根据要完成的动作计算末端执行器(end_effector_link)在空间中的坐标，然后让moveit!执行(逆运动学解算)IK，
订阅节点订阅rostopic:/move_group/display_planned_path，获取rosmsg:moveit_msgs/DisplayTrajectory。
从消息中解析出PVT数据，由CAN总线发送到下位机6个节点上，等节点算完PVT数据后，再次发送开始输出命令给
节点，完成一次姿态的运行。
"""



import rospy
import sys, os
import time, threading
import serial
import ctypes
import string
import moveit_commander
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory

	

class PvtListener:
    def __init__(self, nodename, topicname, dataclass):
        self.__NoResult = 'not find'
        self.__SecToNSec = 1000000000

        self.nodename = nodename
        self.topicname = topicname
        self.dataclass = dataclass

        self.callbackcount = 0

        serialid = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        try:
            self.ser = serial.Serial(serialid, 115200) 
        except Exception, e:
            print 'Open Serial Failed...'
            exit(1)
	
        print 'A Serial Echo Is Running...'


    def f2h(self, data):
        fp = ctypes.pointer(ctypes.c_float(data))
        cp = ctypes.cast(fp, ctypes.POINTER(ctypes.c_longlong))
        return hex(cp.contents.value)


    def pvtdatatransmit(self):
        #
        #print 'Start Converting...', time.strftime("%Y-%m-%d-%H:%M:%S",time.localtime(time.time()))
        for index in range(len(self.pvt)):
	    datalen = len(self.pvt[index])
            pointnum = len(self.pvt[index][0])
            seriallen = datalen * pointnum * 4    #一个浮点数4字节
            #print seriallen
            serialdata = ''

            #print 'Start Converting...'

	    for i in range(datalen):
                for j in range(pointnum):
                    tempstr = self.f2h(self.pvt[index][i][j])[2:]
                    if tempstr == '0':
                        a = 0
                        b = 0
                        c = 0
                        d = 0
                    else:
                        a = int(tempstr[:2], 16)
                        b = int(tempstr[2:4], 16)
                        c = int(tempstr[4:6], 16)
                        d = int(tempstr[6:8], 16)

                    serialdata += chr(d)
                    serialdata += chr(c)
                    serialdata += chr(b)
                    serialdata += chr(a)
        
            print 'Start Sending %d' %index
            self.ser.write(chr(index + 1))
            time.sleep(0.4)
            self.ser.write(serialdata)
            time.sleep(0.4)
            #print time.strftime("%Y-%m-%d-%H:%M:%S",time.localtime(time.time()))
            #print 'End Sending'					

			  	  
    def callback(self, data):	
	#
        jointtrajectory = getattr(data.trajectory[0], 'joint_trajectory', self.__NoResult)
        points = getattr(jointtrajectory, 'points', self.__NoResult)

        #self.callbackcount += 1
        #print 'callbackcount:%d' %self.callbackcount

        length = len(points)
        pvt0 = []
        pvt1 = []
        pvt2 = []
        pvt3 = []
        pvt4 = []
        pvt5 = []
        self.pvt = []

        for i in range(length):
            positions = getattr(points[i], 'positions', self.__NoResult)
            velocities = getattr(points[i], 'velocities', self.__NoResult)
            time = getattr(points[i], 'time_from_start', self.__NoResult)
            second = getattr(time, 'secs', self.__NoResult)
            nsecond = float(getattr(time, 'nsecs', self.__NoResult)) / self.__SecToNSec
            time_from_start = second + nsecond

            pvtpoint = []
            pvtpoint.append(positions[0])
            pvtpoint.append(velocities[0])
            pvtpoint.append(time_from_start)
            pvt0.append(pvtpoint)

            pvtpoint = []
            pvtpoint.append(positions[1])
            pvtpoint.append(velocities[1])
            pvtpoint.append(time_from_start)
            pvt1.append(pvtpoint)

            #因为机械设计的原因，关节2的位置和速度需要加上关节1的值
            pvtpoint = []
            pvtpoint.append(positions[2] + positions[1])
            pvtpoint.append(velocities[2] + velocities[1])
            pvtpoint.append(time_from_start)
            pvt2.append(pvtpoint)

            pvtpoint = []
            pvtpoint.append(positions[3])
            pvtpoint.append(velocities[3])
            pvtpoint.append(time_from_start)
            pvt3.append(pvtpoint)

            pvtpoint = []
            pvtpoint.append(positions[4])
            pvtpoint.append(velocities[4])
            pvtpoint.append(time_from_start)
            pvt4.append(pvtpoint)

            pvtpoint = []
            pvtpoint.append(positions[5])
            pvtpoint.append(velocities[5])
            pvtpoint.append(time_from_start)
            pvt5.append(pvtpoint)

        self.pvt.append(pvt0)
        self.pvt.append(pvt1)
        self.pvt.append(pvt2)
        self.pvt.append(pvt3)
        self.pvt.append(pvt4)
        self.pvt.append(pvt5)

        self.pvtdatatransmit()


    def listener(self):
	#InitNode
	rospy.init_node(self.nodename)
	rospy.Subscriber(self.topicname, self.dataclass, self.callback)
	
	# spin() simply keeps python from exiting until this node is stopped
        print 'Start rospy.spin'
	rospy.spin()
        print 'End rospy.spin'



if __name__ == "__main__":
    pvtlistener = PvtListener('robotTrjcListener', "/move_group/display_planned_path", \
                              DisplayTrajectory)
    pvtlistener.listener()
