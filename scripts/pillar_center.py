#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
from scipy.optimize import leastsq
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker

class PillarCenter:
    def __init__(self):
        rospy.init_node("pillar_center")
        self.is_active = False
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.marker_pub = rospy.Publisher("/pillars_center", Marker, queue_size = 10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.check_move_base_status)
        rospy.loginfo("pillar_center and move_base_check start")
        rospy.spin()
        
    def check_move_base_status(self, msg):
        if msg.status_list:
            latest_status = msg.status_list[-1].status
            self.is_active = (latest_status == 1) #1はactive
            rospy.loginfo("move_base status topic: %d, is_active: %s", latest_status, str(self.is_active))
        
            if latest_status == 3:  # 自律移動以外
                stop_twist = Twist()  #動かないように止める
                self.cmd_pub.publish(stop_twist)

        else:
            self.is_active = False
            rospy.loginfo("move_base status topic: no status, is_active: False")
               
    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pillars_center"
        marker.id = 100
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)
    
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
            clusters = []
            current_cluster = []
            centers = []
            
            for i in range(1, len(ranges) - 1):
                #  abs(ranges[i] - ranges[i + 1]) > 0.3 and 
                if 0.2 < ranges[i] < 2.5 or 0.2 < ranges[i + 1] < 2.5:
                    #上記を満たすレーザの角度　最小値を基準
                    angle1 = scan.angle_min + i * scan.angle_increment
                    angle2 = scan.angle_min + (i + 1) * scan.angle_increment
                    x1 = ranges[i] * math.cos(angle1)
                    y1 = ranges[i] * math.sin(angle1)
                    x2 = ranges[i + 1] * math.cos(angle2)
                    y2 = ranges[i + 1] * math.sin(angle2)
                    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    current_cluster.append((x1, y1))
                    
                    #点同士の距離が離れるとクラスタを分ける
                    if dist > 0.5 and len(current_cluster) >= 5:
                        clusters.append(current_cluster)
                        current_cluster = []
            
            #最後のクラスタの確認
            if len(current_cluster) >= 5:
                clusters.append(current_cluster)
                        
            for cluster in clusters:
                center, radius = fit_circle(cluster)
                rospy.loginfo("Detected %d candidate points for pillar", len(cluster))
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
                self.publish_marker(mid_x, mid_y)
                
if __name__ == "__main__":
    try:
        PillarCenter()
    except rospy.ROSInterruptException:
        pass
        