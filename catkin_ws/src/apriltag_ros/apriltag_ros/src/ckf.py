#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped

from apriltag_ros.msg import AprilTagDetectionArray

import numpy as np
from cubatureKalmanFilter import CubatureKalmanFilter

from scipy.spatial.transform import Rotation as sRotation
import tf2_ros
import geometry_msgs.msg


def AngleWrap(theta):
  return (theta + np.pi)%(2*np.pi) - np.pi

class AprilTagCKF:
    def __init__(self):
        print("Started AprilTagCKF")

        self.dt = 1/15  # this does nnot matter

        # cubature kalman filter parameters
        # measurement noise covariance
        self.R = lambda range_: np.array([[0.05,0],[0,np.pi/20]]) * range_** 4
        # state noise covariannce
        self.Q = np.eye(2) * 0
        # uncertainty in initial state estimate
        self.P = np.array([[0.5,0],[0,0.5]])

        # listener to get map to AT and  map to TB3 transformations of pose
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # initialize dictionary of cubature kalman filters
        self.AprilTagDict = {}

        # self.smooth_pose_pub = rospy.Publisher("/tag_detections_smoothed", AprilTagDetectionArray, queue_size=30)
        self.pose_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.predict_update)
        
    def predict_update(self, msg):
        try:
            # get the transformation from map to baselink (trans,rot)
            TB3_pose= self.GetSource2Target('map','base_link')

            # exttract TB3 positionn in xyz (world)
            TB3_pos = TB3_pose.transform.translation
            TB3_pos = np.array([TB3_pos.x,TB3_pos.y,TB3_pos.z])

            # exttract TB3 yaw in xyz (world) from quaternian
            TB3_rot = TB3_pose.transform.rotation
            TB3_yaw = sRotation.from_quat([TB3_rot.x, TB3_rot.y,TB3_rot.z, TB3_rot.w]).as_euler("zyx")[0]
            TB3_yaw = AngleWrap(TB3_yaw)
        
            # update any measurement of AprilTag from continuous_detector.launch
            for detection in msg.detections:
                id = detection.id[0]
                # relative_pose = detection.pose.pose.pose.position
                cov = detection.pose.pose.covariance        

                # get the transformation from map to Tag (trans,rot)
                AT_pose = self.GetSource2Target("map",f"Tag{int(id)}")

                #  get the AT quarternian in world
                AT_rot = AT_pose.transform.rotation
                AT_quart = np.array([AT_rot.x, AT_rot.y,AT_rot.z, AT_rot.w])

                # get the  AT position in world
                AT_pos = AT_pose.transform.translation
                AT_pos = np.array([AT_pos.x,AT_pos.y,AT_pos.z])


                # if this is first time detecting apriltag, add to dictionary with CKF initial position as first detection
                if self.AprilTagDict.get(id,None) is None:
                    ckf = CubatureKalmanFilter(dim_x=2,dim_z=2,dt=self.dt,
                                            hx=self.MeasurementFn,fx=self.TransitionFn)
                    
                    # initial guess at AT position, which is jjust  first AT detection estimate
                    ckf.x = AT_pos[:2].reshape(-1,1)

                    # set CKF parameters described earlier
                    ckf.R  = self.R((1))
                    ckf.P  = self.P
                    ckf.Q =  self.Q

                    self.AprilTagDict[id] = ckf

                # get the range and bearing measurement between AT and TB3. Bearing is from TB3 to AT.
                else:
                    # get the range (onnly in x-y coordinates)
                    range_ = np.linalg.norm(AT_pos.ravel()[:2]-TB3_pos.ravel()[:2])

                    rospy.logerr(f"RANGE Measure: {range_}")

                    self.AprilTagDict[id].R  = self.R((range_))

                    rospy.logerr(f"R cov: {self.AprilTagDict[id].R}")


                    # get the bearing
                    bearing = AngleWrap(np.arctan2(AT_pos[1]-TB3_pos[1],AT_pos[0]-TB3_pos[0]) - TB3_yaw)

                    # meassurement is range-bearing
                    z = np.array([range_,bearing])

                    # update CKF AT position estimate
                    self.AprilTagDict[id].update(z.reshape(-1,1),hx_args=(TB3_pos,TB3_yaw))

                    # propogate predict step (identity in this case...)
                    self.AprilTagDict[id].predict()

                    # publish  the TF ffrom the map  to the smoothed CKF AT position
                    self.CreateSmoothAxes("map",f"Tag{int(id)}",self.AprilTagDict[id].x.ravel(),AT_quart)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass


    def GetSource2Target(self,source,target):
        while not rospy.is_shutdown():
            try:
                trans= self.tfBuffer.lookup_transform(source, target, rospy.Time(0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

        return trans
    
    def CreateSmoothAxes(self,source,target,AT_rel_trans,AT_rel_quart):
        #  broadcaster to  publish the new Smoothed CKF pose
        br = tf2_ros.TransformBroadcaster()

        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source
        t.child_frame_id = target+ "_ckf"

        t.transform.translation.x = AT_rel_trans[0]
        t.transform.translation.y = AT_rel_trans[1]
        t.transform.translation.z = 0

        t.transform.rotation.x = AT_rel_quart[0]
        t.transform.rotation.y = AT_rel_quart[1]
        t.transform.rotation.z = AT_rel_quart[2]
        t.transform.rotation.w = AT_rel_quart[3]

        br.sendTransform(t)

    def TransitionFn(self,x,dt):
        return x.ravel()
    
    def MeasurementFn(self,x,tb3_pos,tb3_yaw):
        tb3_pos = tb3_pos.ravel()
        x =  x.ravel()
        range_ = np.linalg.norm(tb3_pos[:2] - x[:2])
        bearing = AngleWrap(np.arctan2(x[1]-tb3_pos[1],x[0]-tb3_pos[0])-tb3_yaw)

        z = np.array([range_,bearing])

        # print(z)

        return z
        
if __name__ == '__main__':
    rospy.init_node('AprilTagCKF')
    AprilTagCKF()
    rospy.spin()