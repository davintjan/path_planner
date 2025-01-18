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

        # Find the rightmost green pixel after displaying the image
        rightmost_point = None
        for contour in contours:
            if contour.size > 0:
                rightmost = tuple(contour[contour[:, :, 0].argmax()][0])
                if rightmost_point is None or rightmost[0] > rightmost_point[0]:
                    rightmost_point = rightmost

        if rightmost_point:
            print(f"Rightmost green pixel: {rightmost_point}")

        # Find the leftmost point with y within ±5 of rightmost_point
        leftmost_point = None
        if rightmost_point:
            for contour in contours:
                for point in contour:
                    x, y = point[0]
                    if abs(y - rightmost_point[1]) <= 5:
                        if leftmost_point is None or x < leftmost_point[0]:
                            leftmost_point = (x, y)

            print(f"Rightmost green pixel: {rightmost_point}")
            if leftmost_point:
                print(f"Leftmost green pixel with y ±5 of rightmost: {leftmost_point}")
            else:
                print("No leftmost point found within the y-range.")

        absolute_leftmost_point = None
        for contour in contours:
            for point in contour:
                x, y = point[0]
                if absolute_leftmost_point is None or x < absolute_leftmost_point[0]:
                    absolute_leftmost_point = (x, y)

        if absolute_leftmost_point:
            print(f"Absolute leftmost green pixel: {absolute_leftmost_point}")

        # Visualize all points on the plot_map
        if rightmost_point:
            cv2.circle(plot_map, rightmost_point, 5, (255, 0, 0), -1)  # Rightmost in blue
        if leftmost_point:
            cv2.circle(plot_map, leftmost_point, 5, (0, 0, 255), -1)  # Leftmost within y ±5 in red
        if absolute_leftmost_point:
            cv2.circle(plot_map, absolute_leftmost_point, 5, (0, 255, 255), -1)  # Absolute leftmost in yellow

        # Display updated plot_map with points
        plt.imshow(plot_map)
        plt.show()

def main():
    tracker = trackNeedleAngle()
    tracker.track_angle()

if __name__ == "__main__":
    main()