#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray

""" Get an angle setpoint and turn the robot until it achieves that setpoint """

class executeTurn:
    def __init__(self, target):
        # Initialize ROS node (fix for ROSInitException)
        rospy.init_node('execute_turn', anonymous=True)

        self.rate = rospy.Rate(1)

        # Initialize needle tip angle
        self.needle_tip_angle = None

        # Subscriber for needle tip angle
        rospy.Subscriber('/needle_tip/angle', Float32MultiArray, self.needle_angle_callback)

        # Publisher for target position
        self.target_pos_pub_2 = rospy.Publisher('/targetPos2', Float32, queue_size=10)

        self.current_target = 0.0  # Starting position for /targetPos1
        self.target = target

    def needle_angle_callback(self, msg):
        """Callback function to receive the needle tip angle."""
        if msg.data:
            self.needle_tip_angle = msg.data[0]  # Extract angle value
            rospy.loginfo(f"Updated Needle Angle: {self.needle_tip_angle}")
        else:
            rospy.logwarn("Received empty needle angle data!")

    def publish_move_command(self, increment):
        """Publish a movement command to /targetPos2."""
        self.current_target += increment
        command_msg = Float32()
        command_msg.data = self.current_target
        self.target_pos_pub_2.publish(command_msg)
        rospy.loginfo(f"Published to /targetPos2: {self.current_target}")

    def execute_planned_path(self):
        """Execute movement until the target angle is reached."""
        while self.needle_tip_angle is None:
            rospy.logwarn("Waiting for needle tip angle...")
            rospy.sleep(1)

        while self.needle_tip_angle < self.target:
            rospy.loginfo(f"Needle Tip: {self.needle_tip_angle}, Target: {self.target}")

            # Publish incremented target position
            self.publish_move_command(10.0)
            rospy.sleep(2)  # Allow time for movement

            # Wait for updated needle tip angle
            if self.needle_tip_angle is None:
                rospy.logwarn("Waiting for needle tip angle...")
                rospy.sleep(2)

        rospy.loginfo("Path executed successfully, needle reached the goal")


def main():
    """Main function to execute the turn."""
    executor = executeTurn(30)
    executor.execute_planned_path()


if __name__ == "__main__":
    main()
