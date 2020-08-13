(export ROS_MASTER_URI=http://192.168.33.122:11311) &&
(sleep 3
 rosrun master_discovery_fkie master_discovery) &&
(
  sleep 5
  rosrun master_sync_fkie master_sync	
)
