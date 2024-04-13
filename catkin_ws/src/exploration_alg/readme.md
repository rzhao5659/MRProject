


There is only one ros node which is explore_node.  This will internally run map_client and frontier_detector.
Some codes were adapted from costmap2d package and some from explore_lite.


PLAN:
1. Need to test whether resetting active area would still work.
2. Makes more sense for explorenode handle move_base client.  Offer explore() service and stop() service. 
3. implement NBV and sampling. 



