#include "pointcloud_segmentation/environment.hpp"

using std::placeholders::_1;

environment::environment() : Node("lidar_environment") {
    RCLCPP_INFO(get_logger(), "Init Lidar Environment Node");
    declare_parameter("range_min", 0.0);
    get_parameter("range_min", range_min_);

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(5));

    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "livox/lidar", qos_profile, std::bind(&environment::pointcloud_cb, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_grid_filter_test", 2);

}

pcl::PCLPointCloud2::Ptr environment::FilterCloud(
    pcl::PCLPointCloud2::Ptr              cloud,
    float                                 filterRes,
    Eigen::Vector4f                       minPoint,
    Eigen::Vector4f                       maxPoint)
{
    pcl::PCLPointCloud2::Ptr cloudFiltered (new pcl::PCLPointCloud2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Filter Region of interest
    pcl::CropBox<pcl::PCLPointCloud2> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudFiltered);

    // Filter points on the robot roof top
    std::vector<int> indices;
    pcl::CropBox<pcl::PCLPointCloud2> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudFiltered);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }

    // Remove roof points
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(cloudFiltered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudFiltered);

    return cloudFiltered;
}

void environment::pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    pcl::PCLPointCloud2::Ptr cloud_dst(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    sensor_msgs::msg::PointCloud2 cloud_out;

    pcl_conversions::toPCL(*cloud_msg, *cloud_dst);

    cloud_filtered = FilterCloud(
    cloud_dst,
    0.3 ,
    Eigen::Vector4f (-20, -6, -3, 1),
    Eigen::Vector4f ( 30, 7, 2, 1));

    pcl_conversions::fromPCL(*cloud_filtered,cloud_out);

    cloud_out.header.frame_id = cloud_msg->header.frame_id;
    cloud_out.header.stamp = cloud_msg->header.stamp;

    publisher_->publish(cloud_out);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<environment>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}