#!/usr/bin/env python3
import rospy
import numpy as np
import time
from std_msgs.msg import Float32, Float32MultiArray
import cv2
import matplotlib.pyplot as plt
""" Go to point xy """

class executeCurveTurn:
    def __init__(self, target):
        # Initialize ROS node
        rospy.init_node('execute_turn', anonymous=True)

        self.rate = rospy.Rate(10)

        # Initialize needle tip angle
        self.needle_tip_angle = None
        self.needle_tip_coords = None
        self.start_position = None
        self.position1 = None
        self.position2 = None
        self.position3 = None

        self.received_positions = []  # Store first 10 readings
        self.current_target2 = 0.0
        # Subscribers
        rospy.Subscriber('/needle_tip/filtered_angle', Float32MultiArray, self.needle_angle_callback)
        rospy.Subscriber('/needle_tip/centroid', Float32MultiArray, self.needle_tip_callback)

        # Publisher for target position
        self.target_pos_pub_2 = rospy.Publisher('/targetPos2', Float32, queue_size=10)

        self.target = target

    def needle_tip_callback(self, msg):
        """Callback function to receive needle tip coordinates."""
        if len(msg.data) == 2:
            self.needle_tip_coords = [msg.data[0], msg.data[1]]  # Extract x and y coordinates
            self.received_positions.append(self.needle_tip_coords)

            # Store the 10th received position as the start position
            if len(self.received_positions) == 10 and self.start_position is None:
                self.start_position = self.received_positions[-1]
                rospy.loginfo(f"Set Start Position (10th Read): {self.start_position}")
            #
            # rospy.loginfo(f"Updated Needle Tip Coordinates: {self.needle_tip_coords}")
        else:
            rospy.logwarn("Received malformed needle tip data!")

    def needle_angle_callback(self, msg):
        """Callback function to receive the needle tip angle."""
        if msg.data:
            self.needle_tip_angle = msg.data[0]  # Extract angle value
            # rospy.loginfo(f"Updated Needle Angle: {self.needle_tip_angle}")
        else:
            rospy.logwarn("Received empty needle angle data!")

    def publish_move_command(self, increment):
        """Publish a movement command to /ta    rgetPos2."""
        self.current_target2 += increment
        command_msg = Float32()
        command_msg.data = self.current_target2
        self.target_pos_pub_2.publish(command_msg)
        rospy.loginfo(f"Published to /targetPos2: {self.current_target2}")

    def align_turn_angle(self, goal):
        # when angle 0, create a vector down (-y)
        # When angle > 0 vector points to  (-x, -y) direction
        # Plot the vector, curr pos, and goal pos
        # Check if the vector is close enough to the goal point
        # If not, then:
        # self.publish_move_command(1.0)
        # rospy.sleep(0.5)
        # then recheck again from curr_x, curr_y

        if self.needle_tip_coords is None or self.needle_tip_angle is None:
            rospy.logwarn("Waiting for valid needle tip data...")
            return

        curr_x, curr_y = self.needle_tip_coords
        angle = np.radians(self.needle_tip_angle)  # Convert angle to radians
        # angle = np.radians(25)
        # Calculate the unit vector in the current needle direction
        if angle == 0:
            vector_x, vector_y = 0, -1  # Pointing directly downward
        else:
            vector_x, vector_y = -np.sin(angle), -np.cos(angle)  # Pointing (-x, -y) direction

        goal_x, goal_y = goal  # Unpack goal coordinates

        current_pos = (curr_x, curr_y)
        goal_pos = (goal_x, goal_y)
        current_vector = (vector_x, vector_y)
        norm = self.calculate_norm_and_direction(
                current_pos,
                goal_pos,
                current_vector
        )

        if norm[0] > 1.0:
            self.publish_move_command(1.0)
            rospy.sleep(0.1)
        return norm[0]
    
    def calculate_norm_and_direction(self, current_pos, goal_pos, current_vector):
        # Convert inputs to numpy arrays
        current_pos = np.array(current_pos)
        goal_pos = np.array(goal_pos)
        current_vector = np.array(current_vector)

        # Normalize current_vector
        current_vector = current_vector / np.linalg.norm(current_vector)

        # Calculate vector from current position to goal
        vector_to_goal = goal_pos - current_pos

        # Project vector_to_goal onto current_vector
        projection = np.dot(vector_to_goal, current_vector) * current_vector

        # Calculate the perpendicular component
        perpendicular = vector_to_goal - projection

        # The norm is the magnitude of the perpendicular component
        norm_distance = np.linalg.norm(perpendicular)

        # Calculate direction to goal
        if np.linalg.norm(vector_to_goal) != 0:
            direction_to_goal = vector_to_goal / np.linalg.norm(vector_to_goal)
        else:
            direction_to_goal = np.array([0, 0])  # If at goal, no direction

        return norm_distance, direction_to_goal


def main():
    """Main function to execute the turn."""

    # Initialize the class
    turn_executor = executeCurveTurn(target=0)

    # Wait for `start_position` to be set (after 10 readings)
    while turn_executor.start_position is None and not rospy.is_shutdown():
        rospy.sleep(0.1)  # Allow time for messages to be received

    # Compute positions after start_position is set
    turn_executor.position1 = [turn_executor.start_position[0] - 4, turn_executor.start_position[1] - 10]
    turn_executor.position2 = [turn_executor.position1[0] - 6, turn_executor.position1[1] - 7]
    turn_executor.position3 = [turn_executor.position2[0] - 9, turn_executor.position2[1] - 6]

    # Print the computed positions
    rospy.loginfo(f"Start Position: {turn_executor.start_position}")
    rospy.loginfo(f"Position1: {turn_executor.position1}")
    rospy.loginfo(f"Position2: {turn_executor.position2}")
    rospy.loginfo(f"Position3: {turn_executor.position3}")

    # Call the track_needle method to track the needle tip
    rate = rospy.Rate(10)  # 10 Hz publishing rate

    while not rospy.is_shutdown():
        try:
            norm = turn_executor.align_turn_angle(turn_executor.position1)
            if norm > 1.0:
                print(f"Tracked Needle Norm: {norm}")
            else:
                print("Turning finished")
                print(f"Tracked Needle Norm: {norm}")
                rospy.signal_shutdown('Finished turning!')
                break
        except Exception as e:
            rospy.logerr(f"Error: {e}")
            break
        rate.sleep()

    rospy.spin()  # Keep the node running


if __name__ == "__main__":
    main()
