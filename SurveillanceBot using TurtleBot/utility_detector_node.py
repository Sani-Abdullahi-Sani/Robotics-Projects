#!/usr/bin/env python



import rospy

from sensor_msgs.msg import Image

from std_msgs.msg import String

from cv_bridge import CvBridge, CvBridgeError

import cv2

import numpy as np



class GreenUtilityCartDetector:

    def __init__(self):

        rospy.init_node('green_utility_cart_detector')

        

        # Initialize CvBridge

        self.bridge = CvBridge()

        

        # Subscribe to the camera topic

        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        

        # Publisher for detection result

        self.detect_pub = rospy.Publisher('/witsdetector', String, queue_size=10)

        

        # Flag to indicate if the node should keep running

        self.is_active = True

        

        # Flag to track if images are being received

        self.image_received = False



        # Timer to monitor image reception

        self.image_timer = rospy.Timer(rospy.Duration(5.0), self.image_timer_callback)  # Check every 5 seconds



    def image_callback(self, data):

        if not self.is_active:

            return

        

        try:

            # Convert ROS Image message to OpenCV image

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:

            rospy.logerr(e)

            return

        

        # Set flag indicating image is received

        self.image_received = True

        

        # Process the image to detect green utility cart

        detection_result = self.detect_green_utility_cart(cv_image)

        

        # Publish the detection result

        self.detect_pub.publish(detection_result)

    

    def detect_green_utility_cart(self, image):

        # Convert BGR to HSV

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        

        # Define range of green color in HSV

        lower_green = np.array([40, 50, 50])

        upper_green = np.array([60, 255, 255])

        

        # Threshold the HSV image to get only green colors

        mask = cv2.inRange(hsv, lower_green, upper_green)

        

        # Bitwise-AND mask and original image

        res = cv2.bitwise_and(image, image, mask=mask)

        

        # Count non-zero pixels in the mask (assuming green utility cart is visible)

        if cv2.countNonZero(mask) > 1000:  # Adjust threshold based on your needs

            return "yes"

        else:

            return "no"



    def shutdown(self):

        # This function will be called when the node is terminated

        self.is_active = False

        rospy.loginfo("Shutting down green_utility_cart_detector node.")



    def image_timer_callback(self, event):

        if not self.image_received:

            rospy.logwarn("No images received in the last 5 seconds.")

        self.image_received = False  # Reset image received flag



if __name__ == '__main__':

    try:

        detector = GreenUtilityCartDetector()

        rospy.on_shutdown(detector.shutdown)  # Register shutdown callback

        

        rospy.spin()  # Keep the node running

        

    except rospy.ROSInterruptException:

        pass

