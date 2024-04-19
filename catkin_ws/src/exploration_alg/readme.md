The explore_node continuously runs frontier detection and acts as a server for any ExplorationGoalRequest.
The move base client node is a client both to move base and explore_node:  it will request ExplorationGoal from explore_node and send it to move_base.  This continues until the exploration is completely done (when explore node sends back a response with exploration_done = true). 

## Running the code (completely autonomous)
```bash
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch
roslaunch turtlebot3_navigation move_base.launch
roslaunch exploration_alg explore_node.launch
rosrun exploration_alg move_base_client_node_
```

## Running the code with step-by-step visualization
This is useful for testing and visualizing how the whole process is done.
This requires rqt to send service request, and use rviz move base interface to send goal. 
```bash
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch
roslaunch turtlebot3_navigation move_base.launch
roslaunch exploration_alg test_explore_node.launch
```