import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class DroneController:

    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_takeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('ardrone/land', Empty, queue_size=10)

        rospy.sleep(1)

        self.takeoff()

    def takeoff(self):
        rospy.loginfo("Taking off...")
        self.pub_takeoff.publish(Empty())


    def land(self):
        rospy.loginfo("Landing...")
        self.pub_land.publish(Empty())

    def fly_forward(self):
        rospy.loginfo("Moving forward...")

        twist = Twist()
        twist.linear.x = 1.0
        self.pub_cmd_vel.publish(twist)


    def stop(self):
        rospy.loginfo("Stopping...")
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


    def execute_commands(self):

        rate = rospy.Rate(10)

        rospy.sleep(3)

        self.fly_forward()

        rospy.sleep(3)

        self.stop()

        rospy.sleep(3)
        self.land()

        rate.sleep()



if __name__ == '__main__':
    try:

        rospy.init_node('drone_controller')


        controller = DroneController()

        controller.execute_commands()

    except rospy.ROSInterruptException:

        pass

