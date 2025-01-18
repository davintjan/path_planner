#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def test_publisher():
    rospy.init_node('test_publisher', anonymous=True)
    pub = rospy.Publisher('/targetPos1', Float32, queue_size=10)
    rospy.loginfo("Initialized test publisher")

    rate = rospy.Rate(1)  # 1 Hz
    value = 0.0

    while not rospy.is_shutdown():
        value += 100.0
        msg = Float32()
        msg.data = value
        pub.publish(msg)
        rospy.loginfo(f"Published: {value} to /targetPos1")
        rate.sleep()

if __name__ == "__main__":
    try:
        test_publisher()
    except rospy.ROSInterruptException:
        pass