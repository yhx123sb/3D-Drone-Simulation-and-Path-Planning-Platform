#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import RollPitchYawrateThrust
import math

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower_rpythrust')
        self.pub = rospy.Publisher('/firefly/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
        rospy.Subscriber('/rrt_path', Path, self.path_callback)
        self.current_pose = None
        rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)
        self.Kp_xy = 1.0
        self.Ki_xy = 0.0
        self.Kd_xy = 0.2
        self.Kp_z = 1.0
        self.Ki_z = 0.0
        self.Kd_z = 0.2
        self.thrust_base = 9.8 * 1.6
        self.prev_error = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.integral = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_time = rospy.Time.now().to_sec()
        rospy.loginfo('waypoint_follower_rpythrust node started, waiting for /rt_path...')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position

    def path_callback(self, msg):
        if not msg.poses:
            rospy.logwarn('Received empty path!')
            return
        for pose_stamped in msg.poses:
            self.goto_waypoint(pose_stamped.pose.position)

    def goto_waypoint(self, target):
        while not rospy.is_shutdown():
            if self.current_pose is None:
                self.rate.sleep()
                continue
            now = rospy.Time.now().to_sec()
            dt = now - self.last_time if self.last_time else 0.1
            dx = target.x - self.current_pose.x
            dy = target.y - self.current_pose.y
            dz = target.z - self.current_pose.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist < 0.3:
                rospy.loginfo('Reached waypoint (%.2f, %.2f, %.2f)' % (target.x, target.y, target.z))
                break
            # PID for x
            self.integral['x'] += dx * dt
            derivative_x = (dx - self.prev_error['x']) / dt if dt > 0 else 0.0
            # PID for y
            self.integral['y'] += dy * dt
            derivative_y = (dy - self.prev_error['y']) / dt if dt > 0 else 0.0
            # PID for z
            self.integral['z'] += dz * dt
            derivative_z = (dz - self.prev_error['z']) / dt if dt > 0 else 0.0
            # 防积分爆炸
            for k in self.integral:
                self.integral[k] = max(min(self.integral[k], 5.0), -5.0)
            roll = self.Kp_xy * dy + self.Ki_xy * self.integral['y'] + self.Kd_xy * derivative_y
            pitch = - (self.Kp_xy * dx + self.Ki_xy * self.integral['x'] + self.Kd_xy * derivative_x)
            yaw_rate = 0.0
            thrust = self.thrust_base + self.Kp_z * dz + self.Ki_z * self.integral['z'] + self.Kd_z * derivative_z
            cmd = RollPitchYawrateThrust()
            cmd.roll = roll
            cmd.pitch = pitch
            cmd.yaw_rate = yaw_rate
            cmd.thrust = thrust
            self.pub.publish(cmd)
            self.prev_error['x'] = dx
            self.prev_error['y'] = dy
            self.prev_error['z'] = dz
            self.last_time = now
            self.rate.sleep()

if __name__ == '__main__':
    WaypointFollower()
    rospy.spin() 