#!/usr/bin/env python
# coding:utf-8
import rospy
from sensor_msgs.msg import LaserScan

def scancallback(scan_data):
	print("distance: ",scan_data.ranges[0])

def GetSDMData():
	rospy.init_node('Get_SDM_Data', anonymous=True)# ROS节点初始化
	rospy.Subscriber('/scan',LaserScan,scancallback,queue_size=1000)
	rospy.spin()

if __name__ == '__main__':
	GetSDMData()


