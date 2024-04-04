#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped

from apriltag_ros.msg import AprilTagDetectionArray

import numpy as np
import filterpy.kalman.CubatureKalmanFilter as CubatureKalmanFilter

def AngleWrap(theta):
  return (theta + np.pi)%(2*np.pi) - np.pi

class AprilTagCKF:
    def __init__(self):
        print("Started AprilTagSmoother")
        self.fps = 15
        self.dt = 1/15
        self.ckf = CubatureKalmanFilter(dim_x=4,dim_z=2,dt=self.dt,
                                        hx=self.MeasurementFn,fx=self.TransitionFn)

        self.AprilTagDict = {}

        self.smooth_pose_pub = rospy.Publisher("/tag_detections_smoothed", AprilTagDetectionArray, queue_size=30)
        self.pose_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.predict_update)

    def predict_update(self, msg):
        for detection in msg.detections:
            id = detection.id[0]
            relative_pose = detection.pose.pose.pose.position
            cov = detection.pose.pose.covariance
            
            distance_vector = np.array([relative_pose.x,relative_pose.y,relative_pose.z])

            if self.AprilTagDict.get(detection.id,None) is None:
                self.AprilTagDict[id] = distance_vector
            
            else:
                self.AprilTagDict[id] = (self.AprilTagDict[id] + distance_vector)/2

        print(self.AprilTagDict)
        # print(msg.detections)
        # self.counter += msg.data
        # new_msg = PoseWithCovarianceStamped()
        # new_msg.data = self.counter
        # self.pub.publish(new_msg)

    
    def TransitionFn(self,x,dt):
        return x.ravel()
    
    def MeasurementFn(self,x,x_tb3):
        range_ = np.linalg.norm(x_tb3[:2] - x[:2])
        bearing = AngleWrap(np.arctan2(x[1]-x_tb3[1],x[0]-x_tb3[0])-x_tb3[2])

        z = np.array([range_,bearing])

        return z
        
if __name__ == '__main__':
    rospy.init_node('AprilTagCKF')
    AprilTagCKF()
    rospy.spin()