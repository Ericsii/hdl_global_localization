#include <hdl_global_localization/engines/global_localization_fpfh_teaser.hpp>

#include <ros/ros.h>
#include <teaser/matcher.h>
#include <teaser/registration.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <hdl_global_localization/ransac/matching_cost_evaluater_flann.hpp>

namespace hdl_global_localization {

GlobalLocalizationEngineFPFH_Teaser::GlobalLocalizationEngineFPFH_Teaser(ros::NodeHandle& private_nh) : GlobalLocalizationEngineFPFH_RANSAC(private_nh) {}

GlobalLocalizationEngineFPFH_Teaser::~GlobalLocalizationEngineFPFH_Teaser() {}

void GlobalLocalizationEngineFPFH_Teaser::set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  ROS_INFO_STREAM("Map point: " << cloud->size() << "recv.");

  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*cloud, *filteredCloud1, indices);

  ROS_INFO_STREAM("Downsampling pointcloud");
  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(filteredCloud1);
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  filter.filter(*filteredCloud);
  ROS_INFO_STREAM("After filter: " << filteredCloud->size());

  this->global_map = filteredCloud;
  this->global_map_features = extract_fpfh(filteredCloud);

  evaluater.reset(new MatchingCostEvaluaterFlann());
  evaluater->set_target(this->global_map, private_nh.param<double>("ransac/max_correspondence_distance", 1.0));

  ROS_INFO_STREAM("FPFH_Teaser map setted.");
}

GlobalLocalizationResults GlobalLocalizationEngineFPFH_Teaser::query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);

  auto cloud_features = extract_fpfh(filteredCloud);

  teaser::PointCloud target_cloud, source_cloud;
  teaser::FPFHCloud target_features, source_features;
  target_cloud.reserve(global_map->size());
  target_features.reserve(global_map->size());
  for (int i = 0; i < global_map->size(); i++) {
    target_cloud.push_back({global_map->at(i).x, global_map->at(i).y, global_map->at(i).z});
    target_features.push_back(global_map_features->at(i));
  }

  source_cloud.reserve(filteredCloud->size());
  source_features.reserve(filteredCloud->size());
  for (int i = 0; i < filteredCloud->size(); i++) {
    source_cloud.push_back({filteredCloud->at(i).x, filteredCloud->at(i).y, filteredCloud->at(i).z});
    source_features.push_back(cloud_features->at(i));
  }

  ROS_INFO_STREAM("Find Correspondences");
  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(
    source_cloud,
    target_cloud,
    source_features,
    target_features,
    false,
    private_nh.param<bool>("teaser/corss_check", false),
    private_nh.param<bool>("teaser/tuple_test", false),
    private_nh.param<double>("teaser/tuple_scale", 0.95));

  ROS_INFO_STREAM("Run Teaser");
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = private_nh.param<double>("teaser/noise_bound", 0.5);
  params.cbar2 = private_nh.param<double>("teaser/cbar2", 1.0);
  params.estimate_scaling = false;
  params.rotation_max_iterations = private_nh.param<int>("teaser/rotation_max_iterations", 100);
  params.rotation_gnc_factor = private_nh.param<double>("teaser/rotation_gnc_factor", 1.4);
  params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = private_nh.param<double>("teaser/rotation_cost_threshold", 0.005);

  teaser::RobustRegistrationSolver solver(params);
  solver.solve(source_cloud, target_cloud, correspondences);

  auto solution = solver.getSolution();

  Eigen::Isometry3f transformation = Eigen::Isometry3f::Identity();
  transformation.linear() = solution.rotation.cast<float>();
  transformation.translation() = solution.translation.cast<float>();

  double inlier_fraction = 0.0;
  double error = evaluater->calc_matching_error(*filteredCloud, transformation.matrix(), &inlier_fraction);

  std::vector<GlobalLocalizationResult::Ptr> results(1);
  results[0].reset(new GlobalLocalizationResult(error, inlier_fraction, transformation));

  return GlobalLocalizationResults(results);
}

}  // namespace hdl_global_localization