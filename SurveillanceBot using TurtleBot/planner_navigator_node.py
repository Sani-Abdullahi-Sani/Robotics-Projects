import rospy

import random

import math

import numpy as np

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path, OccupancyGrid

from gazebo_msgs.srv import GetModelState, GetModelStateRequest

from actionlib import SimpleActionClient

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from actionlib_msgs.msg import GoalStatus

class RRTPathPlanner:

    def __init__(self):

        rospy.init_node('rrt_path_planner')

        self.current_pose = None

        self.goal_pose = None

        self.tree = []

        # Map data

        self.map_data = None

        # Publishers and subscribers

        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Wait for the gazebo get model state service to be available

        rospy.wait_for_service('/gazebo/get_model_state')

        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.move_base_client.wait_for_server()

        rospy.loginfo("move_base action server found.")

        self.get_user_goal()



    def get_user_goal(self):

        x = float(input("Enter x coordinate of the goal: "))

        y = float(input("Enter y coordinate of the goal: "))

        self.goal_pose = PoseStamped()

        self.goal_pose.pose.position.x = x

        self.goal_pose.pose.position.y = y

        # Check if goal coordinates are valid within the map

        if not self.is_point_valid(x, y):

            rospy.logerr("Goal coordinates are outside the map bounds or in an occupied space.")

            return



    def get_current_pose(self):

        try:

            req = GetModelStateRequest()

            req.model_name = 'mobile_base'  # Replace with your TurtleBot model name in Gazebo

            res = self.get_model_state(req)

            if res.success:

                self.current_pose = res.pose

            else:

                rospy.logwarn("Failed to get current pose from Gazebo.")

        except rospy.ServiceException as e:

            rospy.logerr("Service call failed: {0}".format(e))



    def map_callback(self, msg):

        self.map_data = msg



    def get_random_node(self, goal_bias=0.2):

        if random.random() < goal_bias and self.goal_pose:

            return self.goal_pose.pose.position.x, self.goal_pose.pose.position.y

        else:

            # Sample random points within the map bounds

            map_x_min = self.map_data.info.origin.position.x

            map_y_min = self.map_data.info.origin.position.y

            map_x_max = map_x_min + self.map_data.info.width * self.map_data.info.resolution

            map_y_max = map_y_min + self.map_data.info.height * self.map_data.info.resolution

            return random.uniform(map_x_min, map_x_max), random.uniform(map_y_min, map_y_max)



    def nearest_node(self, random_node):

        nearest_dist = float('inf')

        nearest_node = None

        for node in self.tree:

            dist = math.sqrt((random_node[0] - node[0]) ** 2 + (random_node[1] - node[1]) ** 2)

            if dist < nearest_dist:

                nearest_dist = dist

                nearest_node = node

        return nearest_node



    def is_point_free(self, x, y):

        if self.map_data is None:

            rospy.logwarn("Map data is not available.")

            return False

        # Convert (x, y) to map coordinates

        map_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)

        map_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Check if the coordinates are within the map bounds

        if map_x < 0 or map_x >= self.map_data.info.width or map_y < 0 or map_y >= self.map_data.info.height:

            return False  # Point is out of bounds

        # Calculate the index in the map data array

        index = map_y * self.map_data.info.width + map_x

        # Check if the point is free (0 is free, 100 is occupied, -1 is unknown)

        if self.map_data.data[index] == 0:

            return True

        else:

            return False



    def is_path_valid(self, from_node, to_node):

        if self.map_data is None:

            rospy.logwarn("Map data is not available.")

            return False

        # Convert from_node and to_node to map grid coordinates

        from_x = int((from_node[0] - self.map_data.info.origin.position.x) / self.map_data.info.resolution)

        from_y = int((from_node[1] - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        to_x = int((to_node[0] - self.map_data.info.origin.position.x) / self.map_data.info.resolution)

        to_y = int((to_node[1] - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Check for obstacles between from_node and to_node using Bresenham's line algorithm

        points = self.bresenham(from_x, from_y, to_x, to_y)

        for (p_x, p_y) in points:

            if not self.is_point_free(p_x * self.map_data.info.resolution + self.map_data.info.origin.position.x,

                                      p_y * self.map_data.info.resolution + self.map_data.info.origin.position.y):

                return False

        return True



    def rrt_path_planning(self):

        self.get_current_pose()

        if self.current_pose is None or not hasattr(self.current_pose, 'position'):

            rospy.logwarn("RRT: Current pose is not available.")

            return None

        self.tree = [(self.current_pose.position.x, self.current_pose.position.y)]

        path = Path()

        path.header.frame_id = "map"

        path.header.stamp = rospy.Time.now()

        path_found = False

        # Increase the number of iterations for better chances of finding a path

        for _ in range(3000):

            random_node = self.get_random_node()

            nearest_node = self.nearest_node(random_node)

            direction = np.array([random_node[0] - nearest_node[0], random_node[1] - nearest_node[1]])

            distance = np.linalg.norm(direction)

            if distance > 2.0:

                direction = direction / distance * 2.0

                new_node = (nearest_node[0] + direction[0], nearest_node[1] + direction[1])

            else:

                new_node = random_node

            if self.is_point_free(new_node[0], new_node[1]):

                if self.is_path_valid(nearest_node, new_node):

                    self.tree.append(new_node)

                    rospy.loginfo("Added node to path plan: ({}, {})".format(new_node[0], new_node[1]))

                    if self.is_path_valid(new_node, (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)):

                        path_found = True

                        self.tree.append((self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

                        break

        if path_found:

            rospy.loginfo("Path found to the goal.")

            path = self.construct_path((self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

            rospy.loginfo("Constructed path: ")

            for pose in path.poses:

                rospy.loginfo("({},{})".format(pose.pose.position.x, pose.pose.position.y))

            return path

        else:

            rospy.logwarn("RRT: Maximum iterations reached without finding a path to the goal.")

            return None



    def construct_path(self, end_node):

        path = Path()

        path.header.frame_id = "map"

        # Reconstruct path from end node to start node

        current_node = end_node

        while current_node:

            pose = PoseStamped()

            pose.header.frame_id = "map"

            pose.pose.position.x = current_node[0]

            pose.pose.position.y = current_node[1]

            path.poses.insert(0, pose)

            current_node = self.get_parent(current_node)

        return path



    def get_parent(self, node):

        # Find parent node for the given node in the tree

        if node in self.tree:

            index = self.tree.index(node)

            if index > 0:

                return self.tree[index - 1]

        return None



    def bresenham(self, x1, y1, x2, y2):

        points = []

        dx = abs(x2 - x1)

        dy = abs(y2 - y1)

        sx = 1 if x1 < x2 else -1

        sy = 1 if y1 < y2 else -1

        err = dx - dy

        while True:

            points.append((x1, y1))

            if x1 == x2 and y1 == y2:

                break

            e2 = 2 * err

            if e2 > -dy:

                err -= dy

                x1 += sx

            if e2 < dx:

                err += dx

                y1 += sy

        return points



    def is_point_valid(self, x, y):

        return self.is_point_free(x, y)



    def move_to_goal(self, path):

        rospy.loginfo("Now moving to goal")

        for pose in path.poses:

            goal = MoveBaseGoal()

            goal.target_pose.header.frame_id = "map"

            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position = pose.pose.position

            goal.target_pose.pose.orientation.w = 1.0

            self.move_base_client.send_goal(goal)

            self.move_base_client.wait_for_result()

            #Check if the goal was reached

            if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:

                rospy.loginfo("Reached goal position!!!!!")

            else:

                rospy.logwarn("Reached goal position!!!!")
	rospy.loginfo("Target has been reached!!!")




if __name__ == '__main__':

    planner = RRTPathPlanner()

    planned_path = planner.rrt_path_planning()

    if planned_path:

        planner.path_pub.publish(planned_path)

        planner.move_to_goal(planned_path)

    else:

        rospy.logwarn("Failed to find a valid path to the goal.")

    rospy.spin()
