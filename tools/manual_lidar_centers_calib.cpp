#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <rclcpp/rclcpp.hpp>

#include "common_lib.h"
#include "../src/data_preprocess.hpp"
#include "../src/qr_detect.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fast_calib");

  Params params = loadParameters(node);
  DataPreprocessPtr dataPreprocessPtr;
  dataPreprocessPtr.reset(new DataPreprocess(params));

  if (dataPreprocessPtr->img_input_.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Input image is empty");
    rclcpp::shutdown();
    return 2;
  }

  QRDetectPtr qrDetectPtr;
  qrDetectPtr.reset(new QRDetect(node, params));

  pcl::PointCloud<pcl::PointXYZ>::Ptr qr_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  qr_center_cloud->reserve(4);
  qrDetectPtr->detect_qr(dataPreprocessPtr->img_input_, qr_center_cloud);
  if (qr_center_cloud->size() != 4)
  {
    RCLCPP_ERROR(node->get_logger(), "Expected 4 QR/camera centers, got %zu", qr_center_cloud->size());
    cv::imwrite(params.output_path + "/qr_detect_manual_lidar.png", qrDetectPtr->imageCopy_);
    rclcpp::shutdown();
    return 3;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  lidar_center_cloud->reserve(4);
  lidar_center_cloud->push_back(pcl::PointXYZ(3.57454f, 0.07096f, 0.11873f));
  lidar_center_cloud->push_back(pcl::PointXYZ(3.55390f, -0.43180f, 0.14098f));
  lidar_center_cloud->push_back(pcl::PointXYZ(3.54801f, 0.06100f, -0.27500f));
  lidar_center_cloud->push_back(pcl::PointXYZ(3.52758f, -0.45020f, -0.25500f));

  pcl::PointCloud<pcl::PointXYZ>::Ptr qr_centers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
  sortPatternCenters(qr_center_cloud, qr_centers, "camera");
  sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

  RCLCPP_INFO(node->get_logger(), "Sorted camera centers:");
  for (const auto &p : qr_centers->points)
  {
    std::cout << "  " << p.x << " " << p.y << " " << p.z << std::endl;
  }
  RCLCPP_INFO(node->get_logger(), "Sorted manual LiDAR centers:");
  for (const auto &p : lidar_centers->points)
  {
    std::cout << "  " << p.x << " " << p.y << " " << p.z << std::endl;
  }

  Eigen::Matrix4f transformation;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
  svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
  aligned_lidar_centers->reserve(lidar_centers->size());
  alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);

  double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
  RCLCPP_INFO(node->get_logger(), "[Manual LiDAR centers] RMSE: %.6f m", rmse);
  RCLCPP_INFO(node->get_logger(), "[Manual LiDAR centers] T_cam_lidar:");
  std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  projectPointCloudToImage(dataPreprocessPtr->cloud_input_, transformation, qrDetectPtr->cameraMatrix_,
                           qrDetectPtr->distCoeffs_, dataPreprocessPtr->img_input_, colored_cloud);
  saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

  rclcpp::shutdown();
  return 0;
}
