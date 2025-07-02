#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def path_callback(msg):
    rate = rospy.Rate(0.5)  # 每2秒发一个点
    for pose in msg.poses:
        pose.header.stamp = rospy.Time.now()
        pub.publish(pose)
        rospy.loginfo("Published waypoint: (%.2f, %.2f, %.2f)" % (
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('waypoint_follower')
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=1)
    rospy.Subscriber('/rrt_path', Path, path_callback)
    rospy.loginfo('waypoint_follower node started, waiting for /rrt_path...')
    rospy.spin() 