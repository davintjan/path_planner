#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import cv2
import serial
import time
from std_msgs.msg import Float32, Float32MultiArray
from generate_path import DubinsPathPlanner
from collections import deque

""" THIS CLASS IS PURELY USED TO TRACK THE NEEDLE TIP AND PUBLISH THE POSITION TO ROSTOPIC """


class trackNeedle(DubinsPathPlanner):
    def __init__(self, turn_radius, travel_distance, straight_step_distance, goal_threshold):
        # Initialize video capture once
        self.video = cv2.VideoCapture(2)
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        ret, frame = self.video.read()
        if ret:
            cv2.imshow("init Frame", frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("failed to cap")

        # Initialize variables
        self.angle_buffer = deque(maxlen=10)
        self.needle_range = [([11, 36, 54], [23, 166, 107])]
        self.init_tracker = True  # Ensure tracker initialization is handled
        self.pub_centroid = rospy.Publisher('/needle_tip/centroid', Float32MultiArray, queue_size=10)
        self.pub_angle = rospy.Publisher('/needle_tip/angle', Float32MultiArray, queue_size=10)
        self.pub_filtered_angle = rospy.Publisher('/needle_tip/filtered_angle', Float32MultiArray, queue_size=10)
        self.pub_target_pos = rospy.Publisher('/targetPos1', Float32, queue_size=10)
        start = self.get_starting_point()

        print('start')
        print(start)

        # Initialize Path Planner
        super().__init__(start, turn_radius, travel_distance, straight_step_distance, goal_threshold)
        rospy.init_node('needle_tip_tracker', anonymous=True)

        # Auto calibrate, get first and second positions
        # first_pos_msg = Float32()
        # first_pos_msg.data = 0.0
        # second_pos_msg = Float32()
        # second_pos_msg.data = 10000.0
        #
        # # Move needle to start
        # self.pub_target_pos.publish(first_pos_msg)
        #
        # # Check pixel position
        # first_pixel_pos = self.get_starting_point()
        # print('first_pixel_position')
        # print(first_pixel_pos)
        #
        # # Move needle to end
        # self.pub_target_pos.publish(second_pos_msg)
        #
        # # Check pixel position
        # second_pixel_pos = self.get_starting_point()
        # print('second_pixel_position')
        # print(second_pixel_pos)


    def calculate_angle(self, x1, y1, x2, y2, x3, y3, x4, y4):
        # Calculate vectors
        vector1 = np.array([x2 - x1, y2 - y1])
        vector2 = np.array([x4 - x3, y4 - y3])

        # Calculate dot product
        dot_product = np.dot(vector1, vector2)

        # Calculate magnitudes
        magnitude1 = np.linalg.norm(vector1)
        magnitude2 = np.linalg.norm(vector2)

        # Calculate cosine of the angle
        cos_angle = dot_product / (magnitude1 * magnitude2)

        # Calculate angle in radians
        angle_rad = np.arccos(np.clip(cos_angle, -1.0, 1.0))

        # Convert to degrees
        angle_deg = np.degrees(angle_rad)

        return angle_deg

    def get_angle(self, p1, p2, p3):
        return self.calculate_angle(p1[1], p1[0], p2[1], p2[0], p2[1], p2[0], p3[1], p3[0])

    def track_needle(self):
        # Read a single frame
        ret, frame = self.video.read()
        if not ret:
            raise Exception("Failed to read frame from video")

        frame = np.flipud(frame)  # Flip y-axis to match rest of code

        # Thresholding
        _, threshold_frame = cv2.threshold(frame, 120, 255, cv2.THRESH_TOZERO)

        # Initialize tracker only once
        if self.init_tracker:
            self.tracker = cv2.TrackerCSRT_create()

            # Automate bounding box selection
            bbox = cv2.selectROI("Select needle", threshold_frame, False)
            # w = 30
            # h = 30
            # bbox = (self.start[0]-15, self.start[0]-15, 30, 30)

            # Draw the bounding box
            # color = (0, 255, 0)  # Green color in BGR
            # thickness = 2  # Line thickness

            # cv2.rectangle(threshold_frame, (bbox[1], bbox[0]), (bbox[1] + w, bbox[0] + h), color, thickness)

            # Display the image
            # cv2.imshow('Image with Bounding Box', threshold_frame)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            print('bbox')
            print(bbox)

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

    def track_three_points(self):
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
        roi_mask[100:400, :420] = 255  # Unmask the desired region
        mask_obstacle = cv2.bitwise_and(mask_obstacle, mask_obstacle, mask=roi_mask)

        # Find contours
        contours, _ = cv2.findContours(mask_obstacle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        plot_map = np.zeros_like(frame)
        for contour in contours:  # for debugging
            cv2.drawContours(plot_map, [contour], -1, (0, 255, 0), -1)
            
        # Display the plot map
        # plt.imshow(plot_map)
        # plt.show()

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
        # plt.imshow(plot_map)
        # plt.show()

        return bottommost_point, topmost_point, absolute_topmost_point

    def track_angle(self):
        bottommost_point, topmost_point, absolute_topmost_point = self.track_three_points()
        # Publish centroid to ROS topic
        # Ensure valid points before calculating angle
        if not bottommost_point or not topmost_point or not absolute_topmost_point:
            rospy.logwarn("Skipping angle calculation due to missing points.")
            return None

        angle = self.get_angle(bottommost_point, topmost_point, absolute_topmost_point)

        # Handle NaN values
        # if np.isnan(angle):
        #     rospy.logwarn("Calculated angle is NaN. Skipping publishing.")
        #     return None
        angle_msg = Float32MultiArray()
        angle = angle if angle == angle else 0
        angle_msg.data = [angle]
        self.pub_angle.publish(angle_msg)
        if angle is not None:
            self.angle_buffer.append(angle)
            filtered_angle = sum(self.angle_buffer) / len(self.angle_buffer)

            filtered_angle_msg = Float32MultiArray()
            filtered_angle_msg.data = [filtered_angle]
            self.pub_filtered_angle.publish(filtered_angle_msg)
        
        print(f"Tracked Needle Angle Position: {angle}")
        print(f"Tracked Filtered Needle Angle Position: {filtered_angle}")

        return angle, filtered_angle

    def get_starting_point(self):
        _, _, absolute_leftmost_point = self.track_three_points()
        return absolute_leftmost_point


def main():
    # start = (320, 180)  # Where the needle enters the frame (need to calibrate)
    turn_radius = 5  # Turn radius for needle path planning
    travel_distance = 15  # Another turning parameter for needle path planning
    straight_step_distance = 15  # Distance to move straight in pixel space
    goal_threshold = 40  # The distance the needle needs to be near the goal to be successful
    track = trackNeedle(turn_radius, travel_distance, straight_step_distance, goal_threshold)

    # Call the track_needle method to track the needle tip
    rate = rospy.Rate(10)  # 10 Hz publishing rate

    while not rospy.is_shutdown():
        try:
            centroid = track.track_needle()
            angle = track.track_angle()
            print(f"Tracked Needle Tip Position: {centroid}")
            if angle is not None:
                print(f"Tracked Needle Angle Position: {angle}")
        except Exception as e:
            rospy.logerr(f"Error: {e}")
            break
        rate.sleep()


if __name__ == "__main__":
    main()
