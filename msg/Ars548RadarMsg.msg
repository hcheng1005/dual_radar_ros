# Livox publish pointcloud msg format.

Header header               # ROS standard message header
uint32 point_num            # Total number of pointclouds
ArbeRadarPoint[] points     # Pointcloud data

