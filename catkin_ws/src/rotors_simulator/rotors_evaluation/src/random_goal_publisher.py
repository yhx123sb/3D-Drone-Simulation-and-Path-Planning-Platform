#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import PoseStamped
import math

# 障碍物参数，与world一致
SPHERES = [
    {'center': (3, 3), 'radius': 0.5},
    {'center': (6, 5), 'radius': 0.7},
    {'center': (8, 2), 'radius': 0.3},
]
MARGIN = 0.5  # 障碍物安全边界

def is_valid(x, y):
    for s in SPHERES:
        cx, cy = s['center']
        r = s['radius'] + MARGIN
        if math.hypot(x - cx, y - cy) < r:
            return False
    return True

def random_point():
    while True:
        x = random.uniform(1, 9)
        y = random.uniform(1, 9)
        if is_valid(x, y):
            return x, y

def publish_goals():
    pub_init = rospy.Publisher('/init_pose', PoseStamped, queue_size=1, latch=True)
    pub_start = rospy.Publisher('/start_pose', PoseStamped, queue_size=1, latch=True)
    pub_goal = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1, latch=True)
    rospy.sleep(1.0)  # 等待Publisher连接
    x1, y1 = random_point()
    x2, y2 = random_point()
    while abs(x1 - x2) < 1.0 and abs(y1 - y2) < 1.0:
        x2, y2 = random_point()  # 保证起终点不太近
    z = 1.0
    # 先发布init_pose
    msg_init = PoseStamped()
    msg_init.header.frame_id = 'world'
    msg_init.pose.position.x = x1
    msg_init.pose.position.y = y1
    msg_init.pose.position.z = z
    msg_init.pose.orientation.w = 1.0
    pub_init.publish(msg_init)
    rospy.loginfo('Published init_pose: (%.2f, %.2f, %.2f)' % (x1, y1, z))
    rospy.sleep(5.0)  # 等待无人机飞到起点
    # 再发布start_pose和goal_pose
    for pub, (x, y), name in zip([pub_start, pub_goal], [(x1, y1), (x2, y2)], ['start', 'goal']):
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        pub.publish(msg)
        rospy.loginfo('Published %s: (%.2f, %.2f, %.2f)' % (name, x, y, z))

if __name__ == '__main__':
    rospy.init_node('random_goal_publisher')
    publish_goals()
    rospy.spin() 