#!/usr/bin/env python3
import rospy
import random
import math
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

class Node:
    def __init__(self, x, y, z, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.parent = parent

class RRTPlanner:
    def __init__(self):
        rospy.init_node('rrt_planner_node')
        self.start_sub = rospy.Subscriber('/start_pose', PoseStamped, self.start_callback)
        self.goal_sub = rospy.Subscriber('/goal_pose', PoseStamped, self.goal_callback)
        self.path_pub = rospy.Publisher('/rrt_path', Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('/rrt_markers', MarkerArray, queue_size=1, latch=True)
        self.start = None
        self.goal = None
        self.map_bounds = [0, 10, 0, 10, 0, 5]  # xmin, xmax, ymin, ymax, zmin, zmax
        self.step_size = 1.0
        self.max_iter = 5000
        # 障碍物参数：与basic.world一致，三维球体
        self.spheres = [
            {'center': (3, 3, 1.5), 'radius': 0.5},
            {'center': (6, 5, 2.5), 'radius': 0.7},
            {'center': (8, 2, 3.0), 'radius': 0.3},
        ]
        rospy.loginfo('RRT Planner Node Initialized')
        self.publish_start_and_goal()

    def publish_start_and_goal(self):
        pub_start = rospy.Publisher('/start_pose', PoseStamped, queue_size=1, latch=True)
        pub_goal = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1, latch=True)
        rospy.sleep(1.0)
        msg_start = PoseStamped()
        msg_start.header.frame_id = 'world'
        msg_start.pose.position.x = 0.0
        msg_start.pose.position.y = 0.0
        msg_start.pose.position.z = 0.0
        msg_start.pose.orientation.w = 1.0
        pub_start.publish(msg_start)
        rospy.loginfo('Published start_pose: (0.00, 0.00, 0.00)')
        msg_goal = PoseStamped()
        msg_goal.header.frame_id = 'world'
        msg_goal.pose.position.x = 9.0
        msg_goal.pose.position.y = 5.0
        msg_goal.pose.position.z = 4.0
        msg_goal.pose.orientation.w = 1.0
        pub_goal.publish(msg_goal)
        rospy.loginfo('Published goal_pose: (9.00, 5.00, 4.00)')

    def start_callback(self, msg):
        self.start = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.try_plan()

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.try_plan()

    def try_plan(self):
        if self.start and self.goal:
            path = self.rrt(self.start, self.goal)
            if path:
                self.publish_path(path)
            else:
                rospy.logwarn('RRT failed to find a path.')

    def rrt(self, start, goal):
        nodes = [Node(start[0], start[1], start[2])]
        for i in range(self.max_iter):
            rand_x = random.uniform(self.map_bounds[0], self.map_bounds[1])
            rand_y = random.uniform(self.map_bounds[2], self.map_bounds[3])
            rand_z = random.uniform(min(self.start[2], self.goal[2]), max(self.start[2], self.goal[2]))
            nearest = min(nodes, key=lambda n: (n.x - rand_x)**2 + (n.y - rand_y)**2 + (n.z - rand_z)**2)
            dx = rand_x - nearest.x
            dy = rand_y - nearest.y
            dz = rand_z - nearest.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist == 0:
                continue
            new_x = nearest.x + self.step_size * dx / dist
            new_y = nearest.y + self.step_size * dy / dist
            new_z = nearest.z + self.step_size * dz / dist
            if self.collision_check((nearest.x, nearest.y, nearest.z), (new_x, new_y, new_z)):
                continue  # 跳过有碰撞的节点
            new_node = Node(new_x, new_y, new_z, nearest)
            nodes.append(new_node)
            # Check if goal is reached
            if math.sqrt((new_x - goal[0])**2 + (new_y - goal[1])**2 + (new_z - goal[2])**2) < self.step_size:
                if not self.collision_check((new_x, new_y, new_z), goal):
                    goal_node = Node(goal[0], goal[1], goal[2], new_node)
                    return self.extract_path(goal_node)
        return None

    def collision_check(self, p1, p2):
        # 检查三维线段p1-p2是否与任一球体相交
        for sphere in self.spheres:
            if self.line_intersects_sphere(p1, p2, sphere['center'], sphere['radius']):
                return True
        return False

    def line_intersects_sphere(self, p1, p2, center, radius):
        # 3D线段与球体的相交判定
        (x1, y1, z1), (x2, y2, z2) = p1, p2
        (cx, cy, cz) = center
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        fx = x1 - cx
        fy = y1 - cy
        fz = z1 - cz
        a = dx*dx + dy*dy + dz*dz
        b = 2 * (fx*dx + fy*dy + fz*dz)
        c = (fx*fx + fy*fy + fz*fz) - radius*radius
        discriminant = b*b - 4*a*c
        if discriminant < 0:
            return False  # 不相交
        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2*a)
        t2 = (-b + discriminant) / (2*a)
        if (0 <= t1 <= 1) or (0 <= t2 <= 1):
            return True  # 有交点
        return False

    def extract_path(self, node):
        path = []
        while node:
            path.append((node.x, node.y, node.z))
            node = node.parent
        path.reverse()
        return path

    def publish_path(self, path):
        ros_path = Path()
        ros_path.header = Header()
        ros_path.header.stamp = rospy.Time.now()
        ros_path.header.frame_id = 'map'
        for x, y, z in path:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        self.path_pub.publish(ros_path)
        rospy.loginfo('Published RRT path with %d waypoints.' % len(path))
        # 发布起点和终点Marker
        self.publish_markers(path)

    def publish_markers(self, path):
        marker_array = MarkerArray()
        # 起点
        start_marker = Marker()
        start_marker.header.frame_id = 'map'
        start_marker.header.stamp = rospy.Time.now()
        start_marker.ns = 'rrt_points'
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = path[0][0]
        start_marker.pose.position.y = path[0][1]
        start_marker.pose.position.z = path[0][2]
        start_marker.scale.x = 0.3
        start_marker.scale.y = 0.3
        start_marker.scale.z = 0.3
        start_marker.color.r = 1.0
        start_marker.color.g = 0.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        marker_array.markers.append(start_marker)
        # 终点
        goal_marker = Marker()
        goal_marker.header.frame_id = 'map'
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.ns = 'rrt_points'
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = path[-1][0]
        goal_marker.pose.position.y = path[-1][1]
        goal_marker.pose.position.z = path[-1][2]
        goal_marker.scale.x = 0.3
        goal_marker.scale.y = 0.3
        goal_marker.scale.z = 0.3
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        marker_array.markers.append(goal_marker)
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    planner = RRTPlanner()
    rospy.spin() 