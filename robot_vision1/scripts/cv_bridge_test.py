#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class image_converter:
    def __init__(self):    
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)#/image_raw

    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #cam = cv2.VideoCapture(0)
            #cam.set(15,-7)
            #ret, frame = cam.read()
            #cv2.imshow("Image", frame)
        except CvBridgeError as e:
            print e

        (rows,cols,channels) = cv_image.shape
        #if cols > 60 and rows > 60 :
            #cv2.circle(cv_image, (60, 60), 30, (0,0,255), -1)
        I = cv_image.copy()
        l = np.size(I,0)
        w = np.size(I,1)
        h = np.size(I,2)

        Ir = cv2.resize(I,(int(w),int(l)))
        Ig = cv2.cvtColor(Ir,cv2.COLOR_BGR2GRAY)
        Igmax = np.max(Ig)
        LEDr,LEDc = np.where(Ig>Igmax-10)
        ILED = np.zeros((l,w))
        numLED = LEDr.size

        for i in range(0,numLED):
            for j in range(0,numLED):
                ILED[LEDr[i],LEDc[j]]=1

        Ic = cv2.Canny(Ig, 20, 200)
        Ib = Ic.copy()
        a = 0
        for i in range(0,int(l/2)):
            lnum = np.where(Ic[i]>120)
        if lnum[0].size !=0:
            a = lnum[0]
            anum = a.size
            for ii in range(1,anum-1):
                Ib[i][a[ii]]=0
 
        cv2.imshow("Original Image", cv_image)
        cv2.imshow("Grayscale Image", Ig)
        cv2.imshow("Binary", Ib)
        cv2.imshow("Processed Image", ILED)


        # 显示Opencv格式的图像
        #cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print e

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
