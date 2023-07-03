#include <pcl/common/common.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

class Filter : public rclcpp::Node {
 public:
  Filter() : Node("minimal_subscriber") {
    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10, std::bind(&Filter::topic_callback, this, std::placeholders::_1));

    using namespace std::chrono_literals;
    publisher_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl/filtered", 10);
    publisher_cluster_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl/cluster", 10);
  }

 private:
  void topic_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)", msg->height, msg->width);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Filter
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");  // x axis
    // extract point cloud between 1.0 and 3.0 m
    pass.setFilterLimits(1.0, 1.75);
    // pass.setFilterLimits(1.0, 2.0);
    // pass.setFilterLimitsNegative (true);   // extract range reverse
    pass.filter(*cloud_filtered);

    // // Voxel Grid
    // pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    // vg.setInputCloud(cloud_filtered);
    // vg.setLeafSize(0.05f, 0.05f, 0.05f);
    // // vg.setDownsampleAllData(true);
    // vg.filter(*cloud);

    // // Statistical Outlier Removal
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(cloud);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(0.1);
    // sor.setNegative(false);
    // sor.filter(*cloud_filtered);

    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);
    // create the extraction object for the clusters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
    // specify euclidean cluster parameters
    ece.setClusterTolerance(0.02);  // 2cm
    ece.setMinClusterSize(20);
    ece.setMaxClusterSize(10000);
    ece.setSearchMethod(tree);
    ece.setInputCloud(cloud_filtered);
    // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
    ece.extract(cluster_indices);
    // pcl::PCDWriter writer;
    int j = 0;
    float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_filtered, *cloud_cluster);
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
        cloud_cluster->points[*pit].r = colors[j % 6][0];
        cloud_cluster->points[*pit].g = colors[j % 6][1];
        cloud_cluster->points[*pit].b = colors[j % 6][2];
      }
      // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      // std::stringstream ss;
      // ss << "cloud_cluster_" << j << ".pcd";
      // writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
      j++;
    }

    // // Cluster
    // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    // tree->setInputCloud(cloud);
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // ec.setClusterTolerance (0.1); // 0.02なら2cm　VLPは分解能は低めなので許容範囲は広く
    // ec.setMinClusterSize (100);//点群の最小サイズを指定
    // ec.setMaxClusterSize (2500);//点群の最大サイズを指定
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (cloud);
    // ec.extract(cluster_indices);

    sensor_msgs::msg::PointCloud2 sensor_msg;
    pcl::toROSMsg(*cloud_filtered, sensor_msg);
    publisher_filtered_->publish(sensor_msg);
    pcl::toROSMsg(*cloud_cluster, sensor_msg);
    publisher_cluster_->publish(sensor_msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_filtered_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_cluster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Filter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
