#ifndef POINTCLOUD_SEGMENTATION__ENVIRONMENT_HPP_
#define POINTCLOUD_SEGMENTATION__ENVIRONMENT_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

struct Plane {
    Eigen::Vector3f normal;
    float d;
};

class environment : public rclcpp::Node {
public:
    // Constructor
    explicit environment();

private:
    /// @brief Apply voxel grid & ROI filtering to the point cloud
    /// @param[in] cloud PCL PointCloud2 Data
    /// @param[in] filterRes XYZ Leaf Size
    /// @param[in] minPoint ROI Min Point
    /// @param[in] maxPoint ROI Max Point
    pcl::PointCloud<pcl::PointXYZI>::Ptr FilterCloud(
        pcl::PointCloud<pcl::PointXYZI>::Ptr    cloud,
        float                                   filterRes,
        Eigen::Vector4f                         minPoint,
        Eigen::Vector4f                         maxPoint);

    /// @brief Detect Robot Forward Object
    /// @param[in] cloud PCL PointCloud2 Data
    /// @param[in] view Visualization Output Data
    /// @param[in] plane1 First Rectangular Plane in 3D Space; Boundaries of the Frustum
    /// @param[in] plane2 Second Rectangular Plane in 3D Space
    /// @return Detected Points
    pcl::PointCloud<pcl::PointXYZI>::Ptr detectObject(
        pcl::PointCloud<pcl::PointXYZI>::Ptr             cloud,
        bool                                        view,
        std::pair<Eigen::Vector3f, Eigen::Vector3f> plane1,
        std::pair<Eigen::Vector3f, Eigen::Vector3f> plane2);

    /// @brief Get Plane Function (Three Points)
    Plane getPlane(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3);

    /// @brief Check Point is Inside the prisms
    bool isPointInside(const std::vector<Plane>& planes, Eigen::Vector3f point);

    double range_min_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    /// @brief Detect object using PointCloud2
    /// @param[in] cloud_msg ROS2 PointCloud2 data
    void pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
};

#endif
