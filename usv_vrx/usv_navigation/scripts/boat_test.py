#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Define the color range for a wider orange in HSV
lower_bound = np.array([5, 150, 150])
upper_bound = np.array([25, 255, 255])

# Reduced minimum area for a contour to be considered a buoy
min_contour_area = 100  # Reduced to detect smaller, distant objects

class BuoyDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('buoy_detector', anonymous=True)
        
        # Set up the subscriber to the camera topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize motor values
        self.left_motor_pwm = 1500
        self.right_motor_pwm = 1500
        
        # Start the ROS loop
        rospy.spin()

    def detect_color(self, image, lower_bound, upper_bound):
        """Detects the specified color in an image and returns a binary mask and the centroid of the detected area."""
        # Convert the image to HSV
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create a mask for the specified color range
        mask = cv2.inRange(image_hsv, lower_bound, upper_bound)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area only
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > min_contour_area]
        
        if filtered_contours:
            # Find the largest contour, which is likely to be the buoy
            largest_contour = max(filtered_contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                # Compute the centroid of the contour
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                return True, mask, (cX, cY)
        
        return False, mask, None

    def image_callback(self, data):
        """Callback function for image topic."""
        try:
            # Convert the ROS Image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Check if the specified color is detected with significant features
            detected, mask, centroid = self.detect_color(cv_image, lower_bound, upper_bound)
            if detected:
                print("Significant buoy feature detected! Adjusting motor values.")
                
                # Get the width of the frame to calculate the position of the buoy
                frame_height, frame_width = cv_image.shape[:2]
                cX, cY = centroid
                
                # Calculate the offset of the buoy from the center of the frame
                offset = cX - (frame_width // 2)
                
                # Determine motor speeds based on the offset
                if abs(offset) < frame_width * 0.1:
                    # If the buoy is near the center, move forward
                    self.left_motor_pwm = 1500
                    self.right_motor_pwm = 1500
                elif offset > 0:
                    # If the buoy is to the right, slow down the right motor
                    self.left_motor_pwm = 1600
                    self.right_motor_pwm = 1400
                else:
                    # If the buoy is to the left, slow down the left motor
                    self.left_motor_pwm = 1400
                    self.right_motor_pwm = 1600
                
                # Print the simulated motor values
                print(f"Left Motor PWM: {self.left_motor_pwm}, Right Motor PWM: {self.right_motor_pwm}")
            else:
                if cv2.countNonZero(mask) == 0:
                    print("No significant features detected.")
                else:
                    print("Mask detected, but too weak or small to act upon.")

            # Display the camera frame and the mask for visualization
            cv2.imshow("Camera Frame", cv_image)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

if __name__ == '__main__':
    try:
        BuoyDetector()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
