# PCL-cpp
> Minhyuk Kim


Distribution
-------------
- Ubuntu 20.04, ROS2 Foxy
- Ubuntu 22.04, ROS2 Humble

## 1.1 PCL function
- pcl_filters: Remove noise
- pcl_features: 3D feture estimation
- pcl_keypoints: Keypoint detection
- pcl_registration: Combine several datasets
- pcl_kdtree: Using FLANN
- pcl_octree: Creating a hierarchical tree data stucture
- pcl_segmentation: Segmenting a point cloud
- pcl_sample_consensus: RANSEC
- pcl_surface: reconstructing surfaces from 3D scans
- pcl_range_image: Represent distance, convert from sensors
- pcl_io: r/w PCD files
- pcl_visualization: Visualize the algorithms operating on 3D point cloud data

## 1.2 Template & Struct
```cpp
// template
pcl::PointCloud<pcl::PointXYZ>

// struct
pcl::PointXYZ
pcl::PointXYZI // LiDARp
pcl::PointXYZRGB // Depth cam
pcl::PCLPointCloud2
```

## 1.3 sensor_msgs::PointCloud2 <-> pcl::PointCloud
```cpp
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
```
sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2
```cpp
pcl::PCLPointCloud2 rtp (const sensor_msgs::PointCloud2 msg) {
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(msg, pcl_pc);
  return pcl_pc;
}
```
pcl::PointCloud -> sensor_msgs::PointCloud2
```cpp
pcl::PointCloud<PointXYZI> ptr (const pcl::PCLPointCloud2 pcl_pc) {
  pcl::PointCloud<PointXYZI> msg;
  pcl_conversions::toPCL(pcl_pc, msg);
  return msg;
}
```

## 1.4 ROI
passthrough filter
```cpp
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

//~~~~~~~~~~~

pcl::PassThrough<pcl::PointXYZI> pass;
pass.setInputCloud (cloud);        // input
pass.setFilterFieldName ("z");     // eg. x, y, z
pass.setFilterLimits (0.70, 1.5);  // min, max
pass.filter (*cloud_filtered);     // apply filter
```
## 1.5 Voxelization
N Pointcloud -> 1 Centroid
```cpp
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//~~~~~~~~~~~

pcl::PointCloud<pcl::PointXYZ> pc_voxelized;
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud::PointXYZ);
pcl::PointCloud<pcl::PointXYZ> voxel_filter;

float voxelsize = 0.2;

voxel_filter.setInputCloud(src);  // src: Orignal PointCloud(Input)
voxel_filter.setLeafSize(voxelsize, voxelsize, voxelsize);
voxel_filter.filter(*ptr_filtered); // output

pc_voxelized = *ptr_filtered;
...
