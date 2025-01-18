#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from generate_path import DubinsPathPlanner

""" GET PATH FROM generate_path.py then activate track_needle and subscribe to the rostopic of centroid. Execute based on that """

class executePath:
    def __init__(self, path, path_strings):
        self.needle_tip_coords = None  # Placeholder for needle tip coordinates
        self.path = path  # Planned path
        self.path_strings = path_strings  # Path commands (e.g., "step", "turn")
        rospy.init_node('path_executor', anonymous=True)
        self.rate = rospy.Rate(1)
        # Subscriber for needle tip centroid
        rospy.Subscriber('/needle_tip/centroid', Float32MultiArray, self.needle_tip_callback)

        # Publisher for target position
        self.target_pos_pub = rospy.Publisher('/targetPos1', Float32, queue_size=10)
        self.current_target = 0.0  # Starting position for /targetPos1

    def needle_tip_callback(self, msg):
        # Parse the Float32MultiArray message
        if len(msg.data) == 2:
            self.needle_tip_coords = [msg.data[0], msg.data[1]]  # Extract x and y coordinates
            rospy.loginfo(f"Updated Needle Tip Coordinates: {self.needle_tip_coords}")
        else:
            rospy.logwarn("Received malformed needle tip data!")

    def publish_move_command(self, increment):
        # Increment and publish the target position
        self.current_target += increment
        command_msg = Float32()
        command_msg.data = self.current_target
        self.target_pos_pub.publish(command_msg)
        rospy.loginfo(f"Published to /targetPos1: {self.current_target}")

    def execute_planned_path(self, allowed_path_error):
        while self.needle_tip_coords is None:
            rospy.logwarn("Waiting for needle tip coordinates...")
            rospy.sleep(1)

        path_taken = []
        path_copy = self.path.copy()

        for i, command in enumerate(self.path_strings):
            if command == "step":
                rospy.loginfo(f"Executing step: target {self.path[i]}")
                rospy.sleep(4)
                rospy.loginfo(f"NEEDLE TIP COORDS ------------------------- {self.needle_tip_coords[0]}")
                while self.needle_tip_coords[0] > self.path[i][0] - allowed_path_error:
                    rospy.loginfo(f"Needle Tip: {self.needle_tip_coords}, Target: {self.path[i]}")

                    # Publish incremented target position
                    self.publish_move_command(100.0)
                    rospy.sleep(2)  # Allow time for movement

                    # Wait for updated needle tip coordinates
                    if self.needle_tip_coords is None:
                        rospy.logwarn("Waiting for needle tip coordinates...")
                        rospy.sleep(2)

                # Update path after reaching target
                path_copy.pop(0)
                rospy.loginfo(f"Updated path: {path_copy}")

                path_taken.append("step")
                rospy.sleep(4)

            elif command == "turn":
                rospy.loginfo("Executing turn command")
                path_taken.append("turn")

            rospy.loginfo(f"Path Taken: {path_taken}")

        rospy.loginfo("Path executed successfully, needle reached the goal")


def main():
    # Initialize parameters
    start = (457, 214)  # Starting position of the needle
    allowed_path_error = 2  # Allowed needle tip error
    turn_radius = 8  # Turn radius for needle path planning
    travel_distance = 3  # Travel distance for needle path planning
    straight_step_distance = 20  # Distance to move straight in pixel space
    goal_threshold = 50  # Threshold distance for successful goal reach

    # Plan the path
    planner = DubinsPathPlanner(start, turn_radius, travel_distance, straight_step_distance, goal_threshold)
    planner.initial_obstacle_goal_detection()
    planner.generate_path()
    planner.plot_path()
    rospy.loginfo(f"Path Strings: {planner.path_strings}")

    # PAUSE AND YOU MANUALLY START TRACKING ON TERMINAL
    input("Press SPACE to continue execution of the planned path...")

    # rospy.loginfo("Testing publish_move_command directly...")
    # executor = executePath(planner.path, planner.path_strings)
    # rate = rospy.Rate(1)
    # value = 100.0
    # while not rospy.is_shutdown():
    #     executor.publish_move_command(value)
    #     rate.sleep()


    # Execute the planned path
    executor = executePath(planner.path, planner.path_strings)
    executor.execute_planned_path(allowed_path_error)


if __name__ == "__main__":
    main()
