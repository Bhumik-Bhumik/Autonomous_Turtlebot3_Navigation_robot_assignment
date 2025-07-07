#!/usr/bin/env python3

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from potential_fields import PotentialFieldAvoidance

class Navigator:
    def __init__(self):
        rospy.init_node('navigator')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/planned_path', Path, self.path_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.pose = None
        self.path = []
        self.index = 0
        self.scan = None
        self.pf = PotentialFieldAvoidance()

        self.goal_tol = 0.15
        self.obstacle_thresh = 0.4

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose = (pos.x, pos.y, yaw)

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.index = 0
        rospy.loginfo("Path received with %d waypoints", len(self.path))

    def scan_callback(self, msg):
        self.scan = msg

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.pose is None or not self.path or self.index >= len(self.path):
                self.pub.publish(Twist())
                rate.sleep()
                continue

            if self.scan and min(self.scan.ranges) < self.obstacle_thresh:
                # Obstacle too close → use PF
                force = self.pf.compute_repulsive_force(self.scan) + self.pf.bias
                cmd = self.pf.to_twist(force)
            else:
                # Normal waypoint tracking
                cmd = self.compute_tracking_cmd()

            self.pub.publish(cmd)
            rate.sleep()

    def compute_tracking_cmd(self):
        x, y, theta = self.pose
        xg, yg = self.path[self.index]
        dx = xg - x
        dy = yg - y
        rho = math.hypot(dx, dy)
        alpha = math.atan2(dy, dx) - theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        if rho < self.goal_tol:
            self.index += 1
            return Twist()

        cmd = Twist()
        cmd.linear.x = min(0.3 * rho, 0.25)
        cmd.angular.z = 1.5 * alpha
        return cmd

if __name__ == '__main__':
    Navigator().run()

