#!/usr/bin/env python
import rospy
import setproctitle
import time
import cv2
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String
import random
import numpy as np
import imutils
import math

class Detection():
    """Class used to detect the laser pointer / receive the frame objects.
    """
    def __init__(self):
        
        # VARIABLES
        self.bridge = CvBridge()
        self.current_objects = None
        self.current_image = None
        self.laser_center = None # pixels
        self.laser_offset = [17, +57] # pixels // Aim for person headshot mostly
        self.target_position = None # pixels
        self.find_laser = True
        # PUBLISHERS
        self.toggle_pub = rospy.Publisher("/movement/shoot", Bool, queue_size=1)
        self.direction_pub = rospy.Publisher("/movement/direction", String, queue_size=1)
        # SUBSCRIBERS
        # Normally this two should be synced by the header stamp but it works anyway.
        rospy.Timer(rospy.Duration(5), self.detect_laser_position)
        self.rgb_sub = rospy.Subscriber("/d400/color/image_raw", Image, self.rgb_cb)
        self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bbox_cb)

        self.generate_colors()
        self.show_image()

    def generate_colors(self):
        """Generate the colors for the detection
        """
        self.colors = []
        for i in range(200):
            self.colors.append([random.randint(0, 254),random.randint(0, 254),random.randint(0, 254)])

    def bbox_cb(self, msg):
        """Callback function used to receive the bboxes from the yolo model.

        Args:
            msg (BoundingBoxes): Array of bounding boxes
        """
        self.current_objects = msg.bounding_boxes

    def rgb_cb(self, msg):
        """Callback function used to receive the rgb image from the camera.

        Args:
            msg (Image): rgb image
        """
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def draw_bboxes(self):
        """Draw the bounding boxes given the current objects
        """
        image_copy = self.current_image.copy()
        if self.current_objects is not None:
            for bbox in self.current_objects:
                image_copy = cv2.rectangle(image_copy, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), self.colors[bbox.id], 2)
                image_copy = cv2.putText(image_copy, bbox.Class, (bbox.xmin, bbox.ymin-5), cv2.FONT_HERSHEY_SIMPLEX, 1, self.colors[bbox.id], 1, cv2.LINE_AA)

        return image_copy

    def detect_laser_position(self, event):
        """Timer user to detect the laser

        Args:
            event (_type_): _description_
        """
        if self.current_image is not None and self.find_laser:
            image_copy = self.current_image.copy()# image_copy = self.current_image.copy()[int((self.current_image.shape[0] / 2 - 150)):int((self.current_image.shape[0]/2 + 150)), int((self.current_image.shape[1] / 2 - 100)):int((self.current_image.shape[1]/2 + 100))]
            hsv_image = cv2.cvtColor(image_copy, cv2.COLOR_BGR2HSV)
            laser_image = cv2.inRange(hsv_image, (145, 13, 254), (180, 122, 255))
            laser_image = cv2.erode(laser_image, np.ones((3, 3), np.uint8), iterations=1)
            laser_image = cv2.erode(laser_image, np.ones((1, 1), np.uint8), iterations=1)
            # laser_image = cv2.dilate(laser_image, np.ones((5, 5), np.uint8), iterations = 3)
            cnts = cv2.findContours(laser_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            for contour in cnts:
                M = cv2.moments(contour)
                if M['m00'] != 0 and cv2.contourArea(contour) < 40:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    circularity = math.pi * 4 * cv2.contourArea(contour) / math.pow(cv2.arcLength(contour,True), 2)
                    if cx > ((image_copy.shape[1] / 2) - 120) and cx < ((image_copy.shape[1] / 2) + 120) and cy > ((image_copy.shape[0] / 2) - 80) and cy < ((image_copy.shape[0] / 2) + 80):
                        self.laser_center = [cx, cy]
                        self.find_laser = False
                        self.target_position = [self.laser_center[0] + self.laser_offset[0], self.laser_center[1] + self.laser_offset[1]]
                        print("Laser detected at (%d, %d)", self.target_position[0], self.target_position[1])
            
            return laser_image

    def move_weapon(self, target_class = ['person']):
        """Move the turret after the target class.

        Args:
            target_class (str, optional): _description_. Defaults to 'person'.
        """
        if self.target_position is not None and self.current_objects is not None:
            for object in self.current_objects:
                if object.Class in target_class:
                    object_center_x = (object.xmin + object.xmax) / 2
                    object_center_y = (object.ymin + object.ymax) / 2

                    object_width = object.xmax - object.xmin
                    object_height = object.ymax - object.ymin

                    x_close_range = object_width / 3
                    y_close_range = object_height / 3

                    x_close = abs(object_center_x - self.target_position[0]) < x_close_range
                    y_close = abs(object_center_y - self.target_position[1]) < y_close_range
                    if x_close and y_close:
                        self.toggle_pub.publish(Bool())
                    else:
                        if not x_close and self.target_position[0] > object_center_x:
                            self.direction_pub.publish(String("right"))
                        if not x_close and self.target_position[0] < object_center_x:
                            self.direction_pub.publish(String("left"))
                        if not y_close and self.target_position[1] > object_center_y:
                            self.direction_pub.publish(String("down"))
                        if not y_close and self.target_position[1] < object_center_y:
                            self.direction_pub.publish(String("up"))
                    break
            



    def show_image(self):
        """Show the image with the bboxes
        """
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            i = 0
            if self.current_image is not None:
                image_copy = self.current_image.copy()
                if self.target_position is not None:
                    # if i % 3 == 0:
                    self.move_weapon()
                        # i = 0
                    image_copy = self.draw_bboxes()
                    cv2.circle(image_copy,(self.target_position[0],self.target_position[1]), 5, (0,0,255), -1)
                    cv2.imshow("imagine", image_copy)
                    cv2.waitKey(1)
                    i += 1
                    # self.current_objects = None
                rate.sleep()


if __name__ == '__main__':
    setproctitle.setproctitle('image_processing')
    rospy.init_node('image_processing')
    Detection()
    # rospy.spin()