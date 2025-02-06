#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
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
        """Publish a movement command to /targetPos2."""
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
        # angle = np.radians(self.needle_tip_angle)  # Convert angle to radians
        angle = np.radians(25)
        # Calculate the unit vector in the current needle direction
        if angle == 0:
            vector_x, vector_y = 0, -1  # Pointing directly downward
        else:
            vector_x, vector_y = -np.sin(angle), -np.cos(angle)  # Pointing (-x, -y) direction

        goal_x, goal_y = goal  # Unpack goal coordinates

        # Plot current position, goal position, and direction vector
        plt.figure(figsize=(6, 6))
        plt.quiver(curr_x, curr_y, vector_x, vector_y, angles='xy', scale_units='xy', scale=1, color='r',
                   label='Needle Direction')
        plt.scatter(curr_x, curr_y, color='b', label='Current Position')
        plt.scatter(goal_x, goal_y, color='g', label='Goal Position')

        plt.xlim(min(curr_x, goal_x) - 1, max(curr_x, goal_x) + 1)
        plt.ylim(min(curr_y, goal_y) - 1, max(curr_y, goal_y) + 1)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        plt.grid()
        plt.title('Needle Alignment Visualization')
        plt.show()

        # DANIEL TO DO, ELONGATE THE VECTOR, FIND THE MINIMUM DIST FROM GOAL TO VECTOR, MINIMIZE THAT BY TURNING


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

    turn_executor.align_turn_angle(turn_executor.position1)

    rospy.spin()  # Keep the node running


if __name__ == "__main__":
    main()
