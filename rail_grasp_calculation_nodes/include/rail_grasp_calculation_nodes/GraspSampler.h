#ifndef GRASP_SAMPLER_H_
#define GRASP_SAMPLER_H_

// ROS
#include <actionlib/server/simple_action_server.h>
#include <rail_grasp_calculation_nodes/PoseWithHeuristic.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_grasp_calculation_msgs/RankGraspsAction.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

// C++
#include <algorithm>
#include <boost/thread/recursive_mutex.hpp>

class GraspSampler
{
public:
  GraspSampler();

private:
  /**
   * @brief Calculate grasp heuristics over a segmented object point cloud
   * @param goal object point cloud information required for grasp calculation
   */
  void rankGraspsObject(const rail_grasp_calculation_msgs::RankGraspsGoalConstPtr &goal);

  /**
   * @brief Calculate grasp heuristics over a full scene
   * @param goal point cloud scene information required for grasp calculation
   */
  void rankGraspsScene(const rail_grasp_calculation_msgs::RankGraspsGoalConstPtr &goal);

  /**
   * @brief Calculate grasp heuristics around a point of interest
   * @param goal point cloud and point of interest information required for grasp calculation
   */
  void rankGraspsPOI(const rail_grasp_calculation_msgs::RankGraspsGoalConstPtr &goal);

  /**
   * @brief Cluster and transform poses to get a final list of grasp candidates
   * @param poses Raw pose input from a grasp sampler or database
   * @param commonFrame Coordinate frame that all poses should be in
   */
  void preprocessPoses(geometry_msgs::PoseArray &poses, std::string commonFrame);

  /**
   * @brief Cluster poses by similar position and orientation
   * @param graspPoses list of poses to cluster
   */
  void clusterPoses(std::vector<geometry_msgs::Pose> &graspPoses);


  /**
   * @brief calculate a distance from normal to plane heuristic
   * @param pose pose to calculate the heuristic for
   * @param plane plane that the pose should be normal to
   * @return heuristic value [0,1], where 0 is perpendicular
   */
  static double heuristicNormal(geometry_msgs::Pose pose, std::vector<float> &plane);

  /**
   * @brief calculate alignment to principal direction heuristic
   * @param pose pose to calculate the heuristic for
   * @param cloud point cloud to calculate the principal direction for
   * @return heuristic vlaue [0,1], where 0 is aligned with the principal direction
   */
  static double heuristicAlignment(geometry_msgs::Pose pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


  /**
   * @brief Crop a point cloud to a given workspace
   * @param workspace workspace to crop the point cloud within
   * @param cloudIn point cloud to crop
   * @param cloudOut point cloud object to return the result
   */
  static void cropToWorkspace(rail_grasp_calculation_msgs::Workspace workspace,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut);

  /**
   * @brief Crop a point cloud to a cube centered at a given point
   * @param point center point of the crop box
   * @param cropSize how far to extend the crop box in each direction from the center point
   * @param cloudIn point cloud to crop
   * @param cloudOut cropped point cloud
   */
  static void cropAtPoint(geometry_msgs::Point point, float cropSize,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut);

  /**
   * Compute the principal direction of a point cloud
   * @param cloud input point cloud
   * @return principal direction
   */
  static Eigen::Quaterniond computePrincipalDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


  static double squaredDistance(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

  static double squaredDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);

  ros::NodeHandle n, pnh;

  actionlib::SimpleActionServer<rail_grasp_calculation_msgs::RankGraspsAction> rankGraspsObjectServer;
  actionlib::SimpleActionServer<rail_grasp_calculation_msgs::RankGraspsAction> rankGraspsSceneServer;
  actionlib::SimpleActionServer<rail_grasp_calculation_msgs::RankGraspsAction> rankGraspsPOIServer;

  boost::recursive_mutex cloudMutex;

  double neighborhoodRadius;
  double orientationThreshold;
  float localSize;
  int clusterSize;
  bool removeTable;
  sensor_msgs::PointCloud2 cloud;
  tf::TransformBroadcaster tfBroadcaster;
  tf::TransformListener tfListener;
};

#endif
