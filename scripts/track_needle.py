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


class trackNeedle(DubinsPathPlanner):

    def __init__(self, start, turn_radius, travel_distance, straight_step_distance, goal_threshold):
        super().__init__(start, turn_radius, travel_distance, straight_step_distance, goal_threshold)
        self.init_tracker = True  # Ensure tracker initialization is handled
        self.pub_centroid = rospy.Publisher('/needle_tip/centroid', Float32MultiArray, queue_size=10)
        rospy.init_node('needle_tip_tracker', anonymous=True)

        # Initialize video capture once
        self.video = cv2.VideoCapture(2)
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def track_needle(self):
        # Read a single frame
        ret, frame = self.video.read()
        if not ret:
            raise Exception("Failed to read frame from video")

        frame = np.flipud(frame)  # Flip y-axis to match rest of code

        # Thresholding
        _, threshold_frame = cv2.threshold(frame, 100, 255, cv2.THRESH_TOZERO)

        # Initialize tracker only once
        if self.init_tracker:
            self.tracker = cv2.TrackerCSRT_create()
            bbox = cv2.selectROI("Select needle", threshold_frame, False)
            self.tracker.init(frame, bbox)
            self.init_tracker = False  # No need to do this on the next frames
            cv2.destroyAllWindows()

        # Update tracker
        success, bbox = self.tracker.update(threshold_frame)
        if not success:
            raise Exception("Failed to track needle tip")

        x, y, w, h = [int(i) for i in bbox]

        # Extract ROI and calculate moments
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        roi = gray_frame[y:y + h, x:x + w]
        moments = cv2.moments(roi)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            centroid = (cx + x, cy + y)  # Adjust centroid to the original frame coordinates
        else:
            raise Exception("Failed to compute moments for the tracked ROI")

        # Publish centroid to ROS topic
        centroid_msg = Float32MultiArray()
        centroid_msg.data = [centroid[0], centroid[1]]
        self.pub_centroid.publish(centroid_msg)

        return centroid


def main():
    start = (504, 223)  # Where the needle enters the frame (need to calibrate)
    turn_radius = 5  # Turn radius for needle path planning
    travel_distance = 15  # Another turning parameter for needle path planning
    straight_step_distance = 15  # Distance to move straight in pixel space
    goal_threshold = 40  # The distance the needle needs to be near the goal to be successful
    track = trackNeedle(start, turn_radius, travel_distance, straight_step_distance, goal_threshold)

    # Call the track_needle method to track the needle tip
    rate = rospy.Rate(10)  # 10 Hz publishing rate
    while not rospy.is_shutdown():
        try:
            centroid = track.track_needle()
            print(f"Tracked Needle Tip Position: {centroid}")
        except Exception as e:
            rospy.logerr(f"Error: {e}")
            break
        rate.sleep()


if __name__ == "__main__":
    main()
