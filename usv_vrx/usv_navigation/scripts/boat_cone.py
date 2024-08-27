#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import cv2
from pymavlink import mavutil
import numpy as np

# Average RGB color of the buoy
target_rgb = (139, 124, 119)

# Define a color range around the average color
lower_bound = np.array([target_rgb[0] - 20, target_rgb[1] - 20, target_rgb[2] - 20])
upper_bound = np.array([target_rgb[0] + 20, target_rgb[1] + 20, target_rgb[2] + 20])

def set_rc_channel_pwm(master, channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        master: MAVLink connection object
        channel_id (int): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return
    
    # Mavlink 2 supports up to 18 channels:
    rc_channel_values = [65535] * 18
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channel_values)

def detect_color(image, lower_bound, upper_bound):
    """Detects the specified color in an image and returns a binary mask and the centroid of the detected area."""
    # Convert the image to BGR (OpenCV format) if not already
    image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    # Create a mask for the specified color range
    mask = cv2.inRange(image_bgr, lower_bound, upper_bound)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Find the largest contour, which is likely to be the buoy
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            # Compute the centroid of the contour
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            return True, mask, (cX, cY)
    
    return False, mask, (0, 0)

def main():
    # Connect to the Pixhawk
    master = mavutil.mavlink_connection('udp://192.168.144.25:14540@192.168.144.26:14540', baud=57600)  # Adjust port as needed

    # Wait for the first heartbeat 
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    # Initialize the camera
    cap = cv2.VideoCapture(0)  # Change to the correct camera source if needed

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    try:
        while True:
            # Spin the boat in place
            set_rc_channel_pwm(master, 3, 1250)  # Left motor forward
            set_rc_channel_pwm(master, 1, 750)   # Right motor backward

            # Capture a frame from the camera
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                continue

            # Check if the specified color is detected
            detected, mask, centroid = detect_color(frame, lower_bound, upper_bound)
            if detected:
                print("Color detected! Adjusting boat movement.")
                
                # Get the width of the frame to calculate the position of the buoy
                frame_height, frame_width = frame.shape[:2]
                cX, cY = centroid
                
                # Calculate the offset of the buoy from the center of the frame
                offset = cX - (frame_width // 2)
                
                # Determine motor speeds based on the offset
                if abs(offset) < frame_width * 0.1:
                    # If the buoy is near the center, move forward
                    left_motor_pwm = 1500
                    right_motor_pwm = 1500
                elif offset > 0:
                    # If the buoy is to the right, slow down the right motor
                    left_motor_pwm = 1600
                    right_motor_pwm = 1400
                else:
                    # If the buoy is to the left, slow down the left motor
                    left_motor_pwm = 1400
                    right_motor_pwm = 1600
                
                # Apply the motor speeds
                set_rc_channel_pwm(master, 3, left_motor_pwm)  # Adjust left motor
                set_rc_channel_pwm(master, 1, right_motor_pwm)  # Adjust right motor

                # Add a small delay to stabilize the movement
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        cap.release()  # Release the camera resource

if __name__ == '__main__':
    main()

