#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped

from apriltag_ros.msg import AprilTagDetectionArray

import numpy as np
import rosparam
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

def AngleWrap(theta):
  return (theta + np.pi)%(2*np.pi) - np.pi

class SearchAndRescue:
    def __init__(self):
        print("Started Search And Rescue")
        self.map_width = rosparam.get_param("/move_base/global_costmap/width")
        self.map_height = rosparam.get_param("/move_base/global_costmap/height")
        self.map_resolution = rosparam.get_param("/move_base/global_costmap/height")
        self.map_origin_x = rosparam.get_param("/move_base/global_costmap/origin_x")
        self.map_origin_y = rosparam.get_param("/move_base/global_costmap/origin_y")
        self.map_dims = (int(self.map_width*2 / self.map_resolution),int(self.map_height*2 / self.map_resolution))


        self.move_base_client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        self.move_base_client.wait_for_server()

        # self.smooth_pose_pub = rospy.Publisher("/tag_detections_smoothed", AprilTagDetectionArray, queue_size=30)
        # self.pose_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.predict_update)

    def send_goal(self, goal):
        
        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.move_base_client.get_result()

    def create_goal(self,x,y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.x = y
        goal.target_pose.pose.orientation.w = 1.0

    def map_indices_to_world_coordinates(self,ind_in_G):
        ind_in_G = ind_in_G.reshape((-1,2))
        
        x_ind = ind_in_G[:,[0]]
        y_ind = ind_in_G[:,[1]]
        
        pos_in_W_x = (self.map_origin_x - self.map_width) + y_ind * self.map_resolution
        pos_in_W_y = (self.map_origin_y - self.map_height) + x_ind * self.map_resolution

        return np.column_stack((pos_in_W_x,pos_in_W_y))


    def world_coordinates_to_map_indces(self,pos_in_W):
        input_dims = pos_in_W.shape

        pos_in_W = pos_in_W.reshape((-1,2))

        T_WtoG = (1/self.map_resolution) * np.array([
            [0,-1,self.map_origin_x],
            [1,0,self.map_origin_y],
            [0,0,1]
        ])

        pos_in_W_heterogeneous = np.hstack(
            [pos_in_W, np.ones((pos_in_W.shape[0], 1))]
        )
        pos_in_G_real = np.dot(T_WtoG, pos_in_W_heterogeneous.T).T[:, 0:2]

        # Discretize the grid coordinates into cell indices
        pos_in_G_int = np.floor(pos_in_G_real).astype(int)

        # Determine which vectors of pos_in_W are within the map's boundaries
        in_map = np.logical_and.reduce(
            (
                pos_in_G_int[:, 0] >= 0,
                pos_in_G_int[:, 1] >= 0,
                pos_in_G_int[:, 0] < self.map_dims[0],
                pos_in_G_int[:, 1] < self.map_dims[1],
            )
        )

        # For all vectors in pos_in_W that *werent* in the map, the
        # returned map index will be -1 --> make sure to look at
        # in_map as well

        # pylint:disable=singleton-comparison
        not_in_map_inds = np.where(in_map == False)  # noqa: E712
        # pylint:enable=singleton-comparison
        pos_in_G_int[not_in_map_inds[0], :] = -1
        pos_in_G_int = pos_in_G_int.reshape(input_dims)
        in_map = in_map.reshape(input_dims[:-1])


        return pos_in_G_int, in_map
    
    def MeasurementFn(self,x,x_tb3):
        range_ = np.linalg.norm(x_tb3[:2] - x[:2])
        bearing = AngleWrap(np.arctan2(x[1]-x_tb3[1],x[0]-x_tb3[0])-x_tb3[2])

        z = np.array([range_,bearing])

        return z
        
if __name__ == '__main__':
    rospy.init_node('search_and_rescue')
    AprilTagCKF()
    rospy.spin()