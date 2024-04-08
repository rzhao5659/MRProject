#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64,Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped

from visualization_msgs.msg import MarkerArray,Marker

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np
import rosparam
import actionlib

import cv2 as cv

from copy import deepcopy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

def AngleWrap(theta):
  return (theta + np.pi)%(2*np.pi) - np.pi

class SearchAndRescue:
    def __init__(self):
        rospy.loginfo("Started Search And Rescue Node")

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
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid,self.set_map)
        self.rot_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.exp_stop_sub = rospy.Subscriber("/exploration_stop_flag",Bool,self.start_search_and_rescue)


        # print("send goal")
        # coord = np.array([0,380]).reshape(-1,2)
        # goal = self.map_indices_to_world_coordinates(coord)
        # print(goal)
        # self.goal_and_rotate(goal.ravel()[0],goal.ravel()[1])

        # print(self.world_coordinates_to_map_indices(goal))
        # print("HERERE")
        
        # rospy.sleep(0.5)
        # self.goal_queue = [goal_pt.reshape(-1,2) for goal_pt in self.grid_map(self.map_grid)]


        # self.nav_goal_markers = rospy.Publisher('queue_goal', MarkerArray,queue_size=10,latch=True)

        # for goal in self.goal_queue:
        #     goal = goal.ravel()
        #     self.markerArray.markers.append(self.create_marker(goal[0],goal[1]))

        # id = 0
        # for m in self.markerArray.markers:
        #     m.id = id
        #     id += 1


        # while not rospy.is_shutdown():
        #     self.nav_goal_markers.publish(self.markerArray)



        # self.goal_and_rotate(-10,-10)
        
        # self.rot_sub = rospy.Subscriber('/cmd_vel_middle', Twist, self.send_rotation, queue_size=1)

        # self.RotStartTime = rospy.Time(0)
        # self.RotEndTime = rospy.Time(0)
        # self.RotDuration = rospy.Duration(5.0)

        # self.nav_goal_marker_pub.publish(self.create_marker(2,-2.2))

        # self.marker_rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     self.nav_goal_marker_pub.publish(self.create_marker(5,5))
        #     self.marker_rate.sleep()
        #     print("here")

        # print("Lord have mercy")
        # print(self.send_goal(self.create_goal(0,0.5)))


    def start_search_and_rescue(self,bool_msg):
        bool_flag = bool_msg.data
        rospy.loginfo(f"Starting Search and Rescue Operation: {bool_flag}")
        

        if bool_flag:
            self.goal_queue = [goal_pt.reshape(-1,2) for goal_pt in self.grid_map(self.map_grid)]
            self.create_goal_queue = True
            rospy.loginfo(f"Number of Search Points: {len(self.goal_queue)}")

            # print("Num Goals=",len(self.goal_queue))
            # print(np.stack(self.goal_queue))
            

            while len(self.goal_queue) != 0:
                current_goal = self.goal_queue.pop().ravel()
                print(current_goal)
                self.goal_and_rotate(current_goal[0],current_goal[1])

                rospy.loginfo(f"Search Point {current_goal}")
            if len(self.goal_queue) == 0:
                rospy.loginfo("Queue Emptied")

    def set_map(self,map):
        self.map_grid_width = map.info.width
        self.map_grid_height = map.info.height

        # print(map.info.width,map.info.width)

        self.map_grid = np.reshape(map.data,(self.map_grid_height,self.map_grid_width)).T
        self.map_dims = (int(self.map_grid_width),int(self.map_grid_width))

        # import matplotlib.pyplot as plt
        # plt.imshow(self.map_grid,origin="lower")
        # plt.savefig("random.png")

    def grid_map(self,map_grid):
        height,width = map_grid.shape


        # import matplotlib.pyplot as plt
        # plt.imshow(self.map_grid,origin="lower")
        # plt.savefig("house.png")
        # dense_idx = np.mgrid[0:height,0:width]

        # sparse_idx = dense_idx[:,::10,::10]

        # goal_pts = map_grid[sparse_idx[0].ravel(),sparse_idx[1].ravel()]
        xmin,xmax = -height//2 * self.map_resolution,height//2 *self.map_resolution
        ymin,ymax = -width//2 * self.map_resolution,width//2 * self.map_resolution


        xs,ys = np.linspace(xmin,xmax,25),np.linspace(ymin,ymax,25)
        xs,ys = np.meshgrid(xs,ys)
        pos_in_W = np.column_stack((xs.ravel(),ys.ravel()))

        pos_in_W  = np.stack(self.snake_sort(coordinates=pos_in_W,rows=25,cols=25))

        map_indices,valid_idx = self.world_coordinates_to_map_indices(pos_in_W)

        map_grid = self.inflate_map(map_grid,radius=0.2)

        # import matplotlib.pyplot as plt
        # plt.imshow(map_grid,origin="lower")
        # plt.savefig("house_inflate.png")

        valid_map_indices  = map_indices[valid_idx]

        free = map_grid[valid_map_indices[:,0],valid_map_indices[:,1]]

        search_indices = valid_map_indices[free==0]

        pos_in_W = self.map_indices_to_world_coordinates(search_indices)

        return pos_in_W
    
    def snake_sort(self,coordinates, rows, cols,direction="h"):

        sorted_coordinates = []
        
        if direction == "v":

            for row in range(rows):

                if row % 2 == 0:

                    for col in range(cols):

                        sorted_coordinates.append(coordinates[row * cols + col])

                else:

                    for col in range(cols - 1, -1, -1):

                        sorted_coordinates.append(coordinates[row * cols + col])

        elif direction == "h":
            coordinates = sorted(coordinates, key=lambda coord: (coord[1], coord[0])) 

            for col in range(cols):

                if col % 2 == 0:

                    for row in range(rows):

                        sorted_coordinates.append(coordinates[row * cols + col])

                else:

                    for row in range(rows - 1, -1, -1):

                        sorted_coordinates.append(coordinates[row * cols + col])
    
        return sorted_coordinates

    def inflate_map(self,map_grid,radius=0.055):
        cell_inflation = int(np.ceil(radius/self.map_resolution))
        
        row,col = map_grid.shape

        map_grid_inflated = deepcopy(map_grid)
        for i in range(row):
            for j in range(col):
                occupied = (self.map_grid[i,j] != 0) and (self.map_grid[i,j] != -1)

                if occupied:
                    min_i = max(0,i-cell_inflation)
                    min_j = max(0,j-cell_inflation)

                    max_i = min(row-1,i+cell_inflation)
                    max_j = min(col-1,j+cell_inflation)

                    map_grid_inflated[min_i:max_i,min_j:max_j] = 100
        

        # import matplotlib.pyplot as plt
        # plt.imshow(map_grid_inflated,origin="lower")
        # plt.savefig("random_inflated_house.png")
        return map_grid_inflated

        
    def send_goal(self, goal):
        self.nav_goal_marker_pub.publish(self.create_marker(goal.target_pose.pose.position.x,goal.target_pose.pose.position.y))

        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result(timeout=rospy.Duration.from_sec(10))
        print(wait)
        rospy.sleep(0.05)
        if not wait:
            rospy.logerr("Timed out to reach goal")

            # rospy.logerr("Action server not available!")
            # rospy.signal_shutdown("Action server not available!")
            print("bleh")
            return False
        else:
            return self.move_base_client.get_result()
        
    def rotate_360(self):
        msg_new  = Twist()
        msg_new.linear.x = 0
        msg_new.angular.z = 0.3125  
    
        StartTime = rospy.Time.now()
        EndTime = StartTime + rospy.Duration(20)
        print("Rotating to search for tags.")
        while rospy.Time.now() <= EndTime:
            self.rot_pub.publish(msg_new)
            rospy.sleep(0.1) 

    def goal_and_rotate(self,x,y):
        goal_msg = self.create_goal(x,y)
        self.send_goal(goal_msg)

        # if success:
        #     self.rotate_360()

        # success=True

    # def send_rotation(self,msg):
    #     if rospy.Time.now() >= self.RotEndTime:
    #         msg_new  = Twist()
    #         msg_new.linear.x = 0
    #         msg_new.angular.z = 0.3125  
    #         StartTime = rospy.Time.now()
    #         EndTime = StartTime + rospy.Duration(20)
    #         while rospy.Time.now() <= EndTime:
    #             print("Rotating to search for tags.")
    #             self.rot_pub.publish(msg_new)
    #             rospy.sleep(0.1) 
    #         self.RotStartTime = rospy.Time.now()
    #         self.RotEndTime = self.RotStartTime + self.RotDuration
    #     else:
    #         self.rot_pub.publish(msg)
        
    def create_marker(self,x,y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time(0)
        marker.frame_locked = True
        marker.lifetime = rospy.Duration()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
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
        
        pos_in_W_x = (self.world_origin_x - self.world_height) + x_ind * self.map_resolution
        pos_in_W_y = (self.world_origin_y - self.world_width) + y_ind * self.map_resolution

        return np.column_stack((pos_in_W_x,pos_in_W_y))


    def world_coordinates_to_map_indices(self,pos_in_W):
        input_dims = pos_in_W.shape

        pos_in_W = pos_in_W.reshape((-1,2))

        T_WtoG = (1/self.map_resolution) * np.array([
            [1,0,self.map_origin_x*self.map_resolution],
            [0,1,self.map_origin_y*self.map_resolution],
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
    rospy.sleep(1)
    SearchAndRescue()
    rospy.spin()