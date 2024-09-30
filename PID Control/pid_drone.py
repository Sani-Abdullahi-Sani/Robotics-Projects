#!/usr/bin/python
import rospy
import sys
import math
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState

def takeoff():
    pub = rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    end_time = time.time() + 3
    while time.time() < end_time:
        rospy.loginfo("Takeoff running")
        pub.publish(Empty())
        rate.sleep()

def land():
    pub = rospy.Publisher('ardrone/land', Empty, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    end_time = time.time() + 3
    while time.time() < end_time:
        rospy.loginfo("Landing")
        pub.publish(Empty())
        rate.sleep()

def fly(linear, rotation):
    pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)

    Flight_path = Twist();
    Flight_path.linear.x = linear[0]
    Flight_path.linear.y = linear[1]
    Flight_path.linear.z = 0
    Flight_path.angular.x = rotation[0]
    Flight_path.angular.y = rotation[1]
    Flight_path.angular.z = rotation[2]
    
    rospy.loginfo("Flying")
    pub.publish(Flight_path)
    rate.sleep()


def norm(x, goal):
    n = 0
    for i in range(len(x)):
        error = goal[i] - x[i]
        n += error * error
    return math.sqrt(n)


def error(target, current):
    err = []
    for i in range(len(current)):
        err.append(target[i] - current[i])
    return err


def prop_error(err, k_p):
    prop = []
    for i in range(len(err)):
        prop.append(k_p * err[i])
    return prop


def int_error(err, k_i):
    for i in range(len(err)):
        err[i] += k_i * err[i]
    return err


def dev_error(prev, current, k_d):
    dev = []
    for i in range(len(current)):
        dev.append(k_d * (current[i] - prev[i]))
    return dev

def calculate_pid(p, i, d):
    u = []
    for j in range(len(p)):
        u.append(p[j]+ i[j] + d[j])
    return u
    

def pid(x, y):
    rospy.init_node('talker', anonymous=True)
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    res = model_coordinates('quadrotor', '')


    target_pos = [x, y]
    target_rot = [0.0, 0.0, 0.0]

    k_pos = [1.0, 0.5, 0.5]
    k_rot = [1.0, 0.5, 0.5]


    curr_pos = [res.pose.position.x, res.pose.position.y]
    curr_rot = [res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z]

    curr_error_pos = error(target_pos, curr_pos)
    curr_error_rot = error(target_rot, curr_rot)

    prev_error_pos = curr_error_pos
    prev_error_rot = curr_error_rot

    while norm(curr_pos, target_pos) > 1e-2:
        res = model_coordinates('quadrotor', '')
        curr_pos = [res.pose.position.x, res.pose.position.y]
        curr_rot = [res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z]

        curr_error_pos = error(target_pos, curr_pos)
        curr_error_rot = error(target_rot, curr_rot)

        prop_error_pos = prop_error(curr_error_pos, k_pos[0])
        prop_error_rot = prop_error(curr_error_rot, k_rot[0])
        int_error_pos = int_error(curr_error_pos, k_pos[1])
        int_error_rot = int_error(curr_error_rot, k_rot[1])
        dev_error_pos = dev_error(prev_error_pos, curr_error_pos, k_pos[2])
        dev_error_rot = dev_error(prev_error_rot, curr_error_rot, k_rot[2])

        u_pos = calculate_pid(prop_error_pos, int_error_pos, dev_error_pos)
        u_rot = calculate_pid(prop_error_rot, int_error_rot, dev_error_rot)
        
        fly(u_pos, u_rot)

        prev_error_pos = curr_error_pos
        prev_error_rot = curr_error_rot

        res = model_coordinates('quadrotor', '')
        curr_pos = [res.pose.position.x, res.pose.position.y]
        curr_rot = [res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z]
    
    
    
if __name__ == '__main__':
    try:
        if len(sys.argv) < 3:
            print("Usage: rosrun pkg_name python script x_pos y_pos")
        else:
            takeoff()
            pid(float(sys.argv[1]), float(sys.argv[2]))
            land()
    except rospy.ROSInterruptException:
        pass
