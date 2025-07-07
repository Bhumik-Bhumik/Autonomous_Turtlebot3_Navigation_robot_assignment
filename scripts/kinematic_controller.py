#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class KinematicController:
    def __init__(self):
        rospy.init_node('kinematic_controller')
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/planned_path', Path, self.path_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.current_pose = None
        self.path = []
        self.index = 0

        self.k_rho = 0.6
        self.k_alpha = 1.0
        self.k_beta = -0.3
        self.tol = 0.15

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_pose = (pos.x, pos.y, yaw)

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.index = 0
        rospy.loginfo("Path received: %d waypoints", len(self.path))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_pose is None or not self.path:
                rate.sleep()
                continue

            if self.index >= len(self.path):
                self.pub.publish(Twist())
                continue

            x, y, theta = self.current_pose
            xg, yg = self.path[self.index]

            dx = xg - x
            dy = yg - y
            rho = math.hypot(dx, dy)
            alpha = math.atan2(dy, dx) - theta
            alpha = math.atan2(math.sin(alpha), math.cos(alpha))

            if rho < self.tol:
                self.index += 1
                continue

            v = self.k_rho * rho
            w = self.k_alpha * alpha

            cmd = Twist()
            cmd.linear.x = min(v, 0.25)
            cmd.angular.z = w
            self.pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    KinematicController().run()

