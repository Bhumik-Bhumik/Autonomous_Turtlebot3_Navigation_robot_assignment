import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PotentialFieldAvoidance:
    def __init__(self, d0=0.3, k_rep=1.2, max_speed=0.2, angular_gain=2.0):
        self.d0 = d0
        self.k_rep = k_rep
        self.max_speed = max_speed
        self.angular_gain = angular_gain
        self.bias = np.array([0.2, 0])  # slight forward push

    def compute_repulsive_force(self, scan):
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges)
        mask = np.isfinite(ranges) & (ranges > 0.05)

        xs = ranges[mask] * np.cos(angles[mask])
        ys = ranges[mask] * np.sin(angles[mask])
        points = np.stack((xs, ys), axis=-1)

        force = np.zeros(2)
        for p in points:
            d = np.linalg.norm(p)
            if d > self.d0:
                continue
            direction = -p / d
            magnitude = self.k_rep * (1.0 / d**2 - 1.0 / self.d0**2)
            force += max(magnitude, 0) * direction

        return force

    def to_twist(self, force_vector):
        angle = np.arctan2(force_vector[1], force_vector[0])
        speed = min(np.linalg.norm(force_vector), self.max_speed)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = np.clip(self.angular_gain * angle, -1.5, 1.5)
        return cmd

