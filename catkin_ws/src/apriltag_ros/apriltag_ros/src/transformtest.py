#!/usr/bin/env python3
import rospy
import tf2_ros

if __name__ == "__main__":
    print("Here!")
    rospy.init_node('apple')
    print("there!")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.loginfo("Starting Listener")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Transform")
            trans= tfBuffer.lookup_transform("map", 'base_link', rospy.Time(0))
            print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue