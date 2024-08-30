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

void environment::pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    pcl::PCLPointCloud2::Ptr cloud_dst(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    pcl_conversions::toPCL(*cloud_msg, *cloud_dst);

    sensor_msgs::msg::PointCloud2 cloud_out;

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