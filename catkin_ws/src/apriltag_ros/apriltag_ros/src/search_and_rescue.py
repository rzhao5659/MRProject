#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped

from visualization_msgs.msg import MarkerArray,Marker


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
        self.world_width = rosparam.get_param("/move_base/global_costmap/width")
        self.world_height = rosparam.get_param("/move_base/global_costmap/height")
        self.world_origin_x = rosparam.get_param("/move_base/global_costmap/origin_x")
        self.world_origin_y = rosparam.get_param("/move_base/global_costmap/origin_y")


        self.map_resolution = rosparam.get_param("/move_base/global_costmap/resolution")

        self.map_dims = (int(self.world_width*2 / self.map_resolution),int(self.world_height*2 / self.map_resolution))

        self.map_origin_x = self.map_dims[0]//2
        self.map_origin_y = self.map_dims[1]//2

        self.move_base_client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        self.move_base_client.wait_for_server()

        # self.smooth_pose_pub = rospy.Publisher("/tag_detections_smoothed", AprilTagDetectionArray, queue_size=30)
        # self.pose_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.predict_update)

        self.markerArray = MarkerArray()
        self.nav_goal_marker_pub = rospy.Publisher('visualization_goal', Marker,queue_size=10,latch=True)


        # self.nav_goal_marker_pub.publish(self.create_marker(2,-2.2))

        # self.marker_rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     self.nav_goal_marker_pub.publish(self.create_marker(5,5))
        #     self.marker_rate.sleep()
        #     print("here")

        # print("Lord have mercy")
        print(self.send_goal(self.create_goal(2.0,-2.2)))

    def send_goal(self, goal):
        self.nav_goal_marker_pub.publish(self.create_marker(goal.target_pose.pose.position.x,goal.target_pose.pose.position.y))

        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result()
        print(wait)

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            print("bleh")
        else:

            return self.move_base_client.get_result()
        
    def create_marker(self,x,y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time(0)
        marker.frame_locked = True
        marker.lifetime = rospy.Duration(10)
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        # self.markerArray.markers.append(marker)
        # self.nav_goal_marker_pub.publish(self.markerArray)
        return marker
    
    def create_goal(self,x,y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        return goal


    def map_indices_to_world_coordinates(self,ind_in_G):
        ind_in_G = ind_in_G.reshape((-1,2))
        
        x_ind = ind_in_G[:,[0]]
        y_ind = ind_in_G[:,[1]]
        
        pos_in_W_x = (self.world_origin_x - self.world_width) + y_ind * self.map_resolution
        pos_in_W_y = (self.world_origin_y - self.world_height) + x_ind * self.map_resolution

        return np.column_stack((pos_in_W_x,pos_in_W_y))


    def world_coordinates_to_map_indices(self,pos_in_W):
        input_dims = pos_in_W.shape

        pos_in_W = pos_in_W.reshape((-1,2))

        T_WtoG = (1/self.map_resolution) * np.array([
            [0,1,self.map_origin_x*self.map_resolution],
            [1,0,self.map_origin_y*self.map_resolution],
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

        
if __name__ == '__main__':
    rospy.init_node('search_and_rescue')
    SearchAndRescue()
    rospy.spin()