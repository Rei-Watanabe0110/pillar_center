#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction

class PillarCenter:
    def __init__(self):
        rospy.init_node("pillar_center")
        #move_baseのアクションを監視する
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.is_active = False
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        #タイマーを使って逐一ステータスチェック
        rospy.Timer(rospy.Duration(0.5), self.check_move_base_status)
        rospy.loginfo("pillar_center and move_base_check start")
        rospy.spin()
        
    def check_move_base_status(self, event):
        state = self.move_base_client.get_state()
        self.is_active = (state == actionlib.GoalStatus.ACTIVE)
    
    def scan_callback(self, scan):
        #ゴール設定しないうちは何もしないように！
        if not self.is_active:
            return
        else:
            ranges = scan.ranges
            pillars = []
        
            for i in range(1, len(ranges) - 1):
                if abs(ranges[i] - ranges[i + 1]) > 0.5 and 0.3 < ranges[i] < 2.0:
                    #上記を満たすレーザの角度　最小値を基準
                    angle = scan.angle_min + i * scan.angle_increment
                    x = ranges[i] * math.cos(angle)
                    y = ranges[i] * math.sin(angle)
                    pillars.append((x, y))
                
                if len(pillars) >= 2:
                    #複数見れることはあるので近い最初の２本から中央距離を算出
                    x_mid = (pillars[0][0] + pillars[1][0]) / 2
                    y_mid = (pillars[0][1] + pillars[1][1]) / 2
                
                    #この座標に寄せる
                    twist = Twist()
                    angle_to_center = math.atan2(y_mid, x_mid) #座標から角度を出す
                    twist.angular.z = 0.5 * angle_to_center
                    twist.linear.x = 0.2
                    self.cmd_pub.publish(twist)
                
if __name__ == "__main__":
    try:
        PillarCenter()
    except rospy.ROSInterruptException:
        pass
        