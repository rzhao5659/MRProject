


There is only one ros node which is explore_node.  This will internally run map_client and frontier_detector.
Some codes were adapted from costmap2d package and some from explore_lite.


Test plan:
1.to test map works,just publish a occuapncy grid message and visualize it in rviz. 
2. 








My plan with: determineExplorationGoal()

1. Read the list of frontiers. (this is continuously updated by frontier_detector)
2. Filter out small size frontiers.
3. Sample points (x,y) around frontier centroid (maybe N iterations): \
   If occupied, continue; \
   else:\
        Discretize yaw to optimize over it:\
        for each pose (x,y,theta):\
            scorePose(pose) according to NBV/timeToGoal.

    choose best pose as best goal near that frontier.

4.  If we have M frontiers, we will have at this point M goals.   Choose best goal out of these M and use this as the next exploration goal.


5.  Someone can write a client node to request an exploration goal and use actionlib to pubblish it toward move_base.  If empty, then exploration is done.  
