#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import cv2
import serial
import time
from std_msgs.msg import Float32MultiArray
from generate_path import DubinsPathPlanner

""" THIS CLASS IS PURELY USED TO TRACK THE NEEDLE TIP AND PUBLISH THE POSITION TO ROSTOPIC """


class trackNeedleAngle:
    def __init__(self):
        self.needle_range = [([11, 36, 54], [23, 166, 107])]
        self.video = cv2.VideoCapture(2)
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.min_contour_area = 1
    def track_angle(self):
        for _ in range(10): #cycle through 10 frames and process the 10th frame
            ret, frame = self.video.read()
            if not ret:
                print("Error: No frame to read")
                break
        frame = np.flipud(frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
        # plt.show()

        lower_bound = np.array(self.needle_range[0][0])
        upper_bound = np.array(self.needle_range[0][1])
        mask_obstacle = cv2.inRange(hsv, lower_bound, upper_bound)

        # Filter ROI: keep only the range x = 144 to x = 477
        roi_mask = np.zeros_like(mask_obstacle)
        roi_mask[:400, 144:477] = 255  # Unmask the desired region
        mask_obstacle = cv2.bitwise_and(mask_obstacle, mask_obstacle, mask=roi_mask)

        # Find contours
        contours, _ = cv2.findContours(mask_obstacle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        plot_map = np.zeros_like(frame)
        for contour in contours:  # for debugging
            cv2.drawContours(plot_map, [contour], -1, (0, 255, 0), -1)

        # Display the plot map
        plt.imshow(plot_map)
        plt.show()

def main():
    tracker = trackNeedleAngle()
    tracker.track_angle()

if __name__ == "__main__":
    main()