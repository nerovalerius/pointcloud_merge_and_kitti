# pointcloud_merge_and_kitti
ROS Package - Merge multiple pointclouds and then store them as KITTI binary files

## Usage
Start Merge Node to merge the pointclouds
```ros2 run pointcloud_merge_and_kitti merge_and_kitti```

Start ROSBAG Playback
```ros2 bag play rosbag2_2022_06_21-12_56_43_0.db3 -s sqlite3 --rate 1```

View Pointcloud
``` pcl_viewer full_cloud_x.pcd -fc 255,255,255 -bc 0,0,0```