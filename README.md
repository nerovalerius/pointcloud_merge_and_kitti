# pointcloud_merge_and_kitti

![full cloud image](https://github.com/nerovalerius/pointcloud_merge_and_kitti/blob/main/images/merged_cloud.jpg)



ROS Package - Merge multiple pointclouds and then store them as KITTI binary files

For now, this node uses 5 point clouds coming from 5 livox horizon lidars, mounted on a bicycle.
This node reads each topic, transforms the pointclouds with information out of the ROS2 tf tree and merges them.
5 subscribers push the incoming point clouds into a fifo queue and then merge and store them as single .pcd file.
Furthermore, the timestamp of each point cloud are checked and logged into a csv file.,

## Usage
Start Merge Node to merge the pointclouds
```ros2 run pointcloud_merge_and_kitti merge_and_kitti```

Start ROSBAG Playback
```ros2 bag play rosbag2_2022_06_21-12_56_43_0.db3 -s sqlite3 --rate 1```


View Pointclouds
``` pcl_viewer full_cloud_x.pcd -fc 255,255,255 -bc 0,0,0```
