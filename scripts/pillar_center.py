#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
import actionlib
from scipy.optimize import leastsq
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatusArray

class PillarCenter:
    def __init__(self):
        rospy.init_node("pillar_center")
        #move_baseのアクションを監視する
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.is_active = False
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.check_move_base_status)
        rospy.loginfo("pillar_center and move_base_check start")
        rospy.spin()
        
    def check_move_base_status(self, msg):
        if msg.status_list:
            latest_status = msg.status_list[-1].status
            self.is_active = (latest_status == 1) #1はactive
            rospy.loginfo("move_base status topic: %d, is_active: %s", latest_status, str(self.is_active))
        else:
            self.is_active = False
            rospy.loginfo("move_base status topic: no status, is_active: False")
               
    def scan_callback(self, scan):
        
        def fit_circle(points):
            x = np.array([p[0] for p in points])
            y = np.array([p[1] for p in points])
        
            def calc_R(xc, yc):
                return np.sqrt((x - xc)**2 + (y - yc)**2)
        
            def residuals(c):
                Ri = calc_R(*c) #タプルのアンパック　cでまとめたのを計算できるようにしている
                return Ri - np.mean(Ri)
        
            center_estimate = np.mean(x), np.mean(y)
            center_fit = leastsq(residuals, center_estimate)[0] #最適解のみを取り出す じゃないと下の計算ができない
            radius_fit = np.mean(calc_R(*center_fit))
        
            return center_fit, radius_fit
        
        #ゴール設定しないうちは何もしないように！
        if not self.is_active:
            return
        else:
            ranges = scan.ranges
            points = []
            centers = []
            
            for i in range(1, len(ranges) - 1):
                if abs(ranges[i] - ranges[i + 1]) > 0.3 and 0.2 < ranges[i] < 2.5:
                    #上記を満たすレーザの角度　最小値を基準
                    angle = scan.angle_min + i * scan.angle_increment
                    x = ranges[i] * math.cos(angle)
                    y = ranges[i] * math.sin(angle)
                    points.append((x, y))
                
                if len(points) >= 7:
                    center, radius = fit_circle(points)
                    rospy.loginfo("Detected %d candidate points for pillar", len(points))
                    rospy.loginfo("Fitted circle center: (%.2f, %.2f), radius: %.2f", center[0], center[1], radius)

                    if 0.05 < radius < 2.5:
                        centers.append(center)
                        
            if len(centers) >= 2:
                rospy.loginfo("Number of centers are two over!")
                mid_x = (centers[0][0] + centers[1][0]) / 2.0
                mid_y = (centers[0][1] + centers[1][1]) / 2.0
                        #この座標に寄せる
                twist = Twist()
                angle_to_center = math.atan2(mid_y, mid_x) #座標から角度を出す
                twist.angular.z = 0.5 * angle_to_center
                twist.linear.x = 0.2
                self.cmd_pub.publish(twist)
                
if __name__ == "__main__":
    try:
        PillarCenter()
    except rospy.ROSInterruptException:
        pass
        