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

        plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
        plt.show()

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

        # Find the **bottommost** point (was rightmost before)
        bottommost_point = None
        for contour in contours:
            if contour.size > 0:
                bottommost = tuple(contour[contour[:, :, 1].argmax()][0])  # Find the highest y-value
                if bottommost_point is None or bottommost[1] > bottommost_point[1]:  # Compare y-coordinates
                    bottommost_point = bottommost

        if bottommost_point:
            print(f"Bottommost green pixel: {bottommost_point}")

        # Find the **topmost** point with x within ±5 of bottommost x (was leftmost before)
        topmost_point = None
        if bottommost_point:
            for contour in contours:
                for point in contour:
                    x, y = point[0]
                    if abs(x - bottommost_point[0]) <= 5:  # Same x-range constraint
                        if topmost_point is None or y < topmost_point[1]:  # Find the lowest y-value
                            topmost_point = (x, y)

            print(f"Bottommost green pixel: {bottommost_point}")
            if topmost_point:
                print(f"Topmost green pixel with x ±5 of bottommost: {topmost_point}")
            else:
                print("No topmost point found within the x-range.")

        # Find the **absolute topmost** point (was absolute leftmost before)
        absolute_topmost_point = None
        for contour in contours:
            for point in contour:
                x, y = point[0]
                if absolute_topmost_point is None or y < absolute_topmost_point[1]:  # Find the lowest y-value
                    absolute_topmost_point = (x, y)

        if absolute_topmost_point:
            print(f"Absolute topmost green pixel: {absolute_topmost_point}")

        # Visualize points on the plot_map
        if bottommost_point:
            cv2.circle(plot_map, bottommost_point, 5, (255, 0, 0), -1)  # Bottommost in blue
        if topmost_point:
            cv2.circle(plot_map, topmost_point, 5, (0, 0, 255), -1)  # Topmost within x ±5 in red
        if absolute_topmost_point:
            cv2.circle(plot_map, absolute_topmost_point, 5, (0, 255, 255), -1)  # Absolute topmost in yellow

        # Display updated plot_map with points
        plt.imshow(plot_map)
        plt.show()


def main():
    tracker = trackNeedleAngle()
    tracker.track_angle()

if __name__ == "__main__":
    main()