#include "rail_grasp_calculation_nodes/GraspSampler.h"

using namespace std;

GraspSampler::GraspSampler() :
    pnh("~"),
    rankGraspsObjectServer(pnh, "rank_grasps_object", boost::bind(&GraspSampler::rankGraspsObject, this, _1), false),
    rankGraspsSceneServer(pnh, "rank_grasps_scene", boost::bind(&GraspSampler::rankGraspsScene, this, _1), false),
    rankGraspsPOIServer(pnh, "rank_grasps_poi", boost::bind(&GraspSampler::rankGraspsPOI, this, _1), false)
{
  pnh.param("neighborhood_radius", neighborhoodRadius, 0.02);
  pnh.param("orientation_threshold", orientationThreshold, 0.1);
  pnh.param("cluster_size", clusterSize, 5);
  pnh.param("remove_table", removeTable, false);
  pnh.param("local_window_size", localSize, 0.015f);

  neighborhoodRadius = pow(neighborhoodRadius, 2);
  orientationThreshold = pow(orientationThreshold, 2);

  rankGraspsObjectServer.start();
  rankGraspsSceneServer.start();
  rankGraspsPOIServer.start();
}

void GraspSampler::rankGraspsObject(const rail_grasp_calculation_msgs::RankGraspsGoalConstPtr &goal)
{
  rail_grasp_calculation_msgs::RankGraspsFeedback feedback;
  rail_grasp_calculation_msgs::RankGraspsResult result;

  //make sure environment point cloud is in same frame as object cloud and convert both to PCL clouds
  string commonFrame = goal->segmentedCloud.header.frame_id;
  ROS_INFO("COMMON FRAME: %s", commonFrame.c_str());
  sensor_msgs::PointCloud2 transformedCloud;
  pcl::PCLPointCloud2::Ptr tempCloud(new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertedEnvironmentCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  //object cloud
  pcl_conversions::toPCL(goal->segmentedCloud, *tempCloud);
  pcl::fromPCLPointCloud2(*tempCloud, *convertedCloud);

  //environment cloud
  if (goal->sceneCloud.header.frame_id != commonFrame)
  {
    pcl_ros::transformPointCloud(commonFrame, goal->sceneCloud, transformedCloud, tfListener);
    transformedCloud.header.frame_id = commonFrame;
    pcl_conversions::toPCL(transformedCloud, *tempCloud);
  }
  else
  {
    pcl_conversions::toPCL(goal->sceneCloud, *tempCloud);
  }
  pcl::fromPCLPointCloud2(*tempCloud, *convertedEnvironmentCloud);

  ROS_INFO("Received %lu grasps...", goal->graspList.poses.size());

  ROS_INFO("Merging similar poses...");
  geometry_msgs::PoseArray finalPoses = goal->graspList;
  preprocessPoses(finalPoses, commonFrame);

  // Pose heuristics
  pcl::SACSegmentation<pcl::PointXYZRGB> planeSegmenter;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // set the segmentaion parameters
  planeSegmenter.setOptimizeCoefficients(true);
  planeSegmenter.setModelType(pcl::SACMODEL_PLANE);
  planeSegmenter.setMethodType(pcl::SAC_RANSAC);
  planeSegmenter.setDistanceThreshold(0.005);

  //Extract the global (environment) dominant plane
  bool environmentPlaneFound = false;
  vector<float> globalPlane;
  globalPlane.resize(4);
  planeSegmenter.setInputCloud(convertedEnvironmentCloud);
  planeSegmenter.segment(*inliers, *coefficients);
  if (!inliers->indices.empty())
  {
    environmentPlaneFound = true;
    for (size_t i = 0; i < 4; i ++)
    {
      globalPlane[i] = coefficients->values[i];
    }
  }

  // cluster object cloud
  vector<pcl::PointIndices> clusterIndices;
  vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
  vector<pcl::search::KdTree<pcl::PointXYZRGB>::Ptr> searchTrees;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clusterer;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kdTree->setInputCloud(convertedCloud);
  clusterer.setInputCloud(convertedCloud);
  clusterer.setClusterTolerance(0.01);
  clusterer.setMinClusterSize(20);
  clusterer.setMaxClusterSize(30000);
  clusterer.setSearchMethod(kdTree);
  clusterer.extract(clusterIndices);

  for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it ++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit ++)
    {
      tempCluster->points.push_back(convertedCloud->points[*pit]);
    }
    tempCluster->width = tempCluster->points.size();
    tempCluster->height = 1;
    tempCluster->is_dense = true;
    clusters.push_back(tempCluster);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    searchTree->setInputCloud(tempCluster);
    searchTrees.push_back(searchTree);
  }

  if (clusters.empty())
  {
    ROS_INFO("No clusters could be extracted, cannot calculate grasps...");
    rankGraspsObjectServer.setSucceeded(result);
    return;
  }

  vector<PoseWithHeuristic> rankedFinalPoses;

  for (unsigned int i = 0; i < finalPoses.poses.size(); i ++)
  {
    //heuristic 1: perpendicular to global (environment) plane
    double h1 = 1;
    if (environmentPlaneFound)
    {
      h1 = heuristicNormal(finalPoses.poses[i], globalPlane);
    }

    //heuristic 2: perpendicular to closest local (object) plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr localCroppedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cropAtPoint(finalPoses.poses[i].position, localSize, convertedCloud, localCroppedCloud);

    //fit local plane
    planeSegmenter.setInputCloud(localCroppedCloud);
    planeSegmenter.segment(*inliers, *coefficients);

    double h2 = 1;
    if (!inliers->indices.empty())
    {
      h2 = heuristicNormal(finalPoses.poses[i], coefficients->values);
    }

    //heuristic 3: alignment with principal component analysis
    double minClusterDst = numeric_limits<double>::max();
    int minClusterIndex = 0;
    vector<int> kIndices;
    vector<float> kSqrDistances;
    kIndices.resize(1);
    kSqrDistances.resize(1);
    pcl::PointXYZRGB testPoint;
    testPoint.x = static_cast<float>(finalPoses.poses[i].position.x);
    testPoint.y = static_cast<float>(finalPoses.poses[i].position.y);
    testPoint.z = static_cast<float>(finalPoses.poses[i].position.z);
    for (unsigned int j = 0; j < clusters.size(); j ++)
    {
      searchTrees[j]->nearestKSearch(testPoint, 1, kIndices, kSqrDistances);
      if (kSqrDistances[0] < minClusterDst)
      {
        minClusterDst = kSqrDistances[0];
        minClusterIndex = j;
      }
    }

    double h3 = heuristicAlignment(finalPoses.poses[i], clusters[minClusterIndex]);

    //heuristic 4: distance from object center (a parametrization of grasp location)
    geometry_msgs::Point centerPoint;
    pcl::PointXYZRGB minPointObject, maxPointObject;
    pcl::PointXYZRGB minPointEnv, maxPointEnv;
    pcl::getMinMax3D(*convertedCloud, minPointObject, maxPointObject);
    pcl::getMinMax3D(*convertedEnvironmentCloud, minPointEnv, maxPointEnv);
    centerPoint.x = (minPointObject.x + maxPointObject.x)/2.0;
    centerPoint.y = (minPointObject.y + maxPointObject.y)/2.0;
    centerPoint.z = (minPointObject.z + maxPointObject.z)/2.0;
    double workspaceDiagonalSquared = (pow(minPointEnv.x - maxPointEnv.x, 2)
                                 + pow(minPointEnv.y - maxPointEnv.y, 2)
                                 + pow(minPointEnv.z - maxPointEnv.z, 2))/4.0;
    double h4 = sqrt(squaredDistance(centerPoint, finalPoses.poses[i].position)
                     / workspaceDiagonalSquared);

    //heuristic 5: distance to closest point in object cloud
    kdTree->nearestKSearch(testPoint, 1, kIndices, kSqrDistances);
    pcl::PointXYZRGB nearestPoint = convertedCloud->at(static_cast<size_t>(kIndices.at(0)));

    double h5 = sqrt((pow(nearestPoint.x - testPoint.x, 2)
                      + pow(nearestPoint.y - testPoint.y, 2)
                      + pow(nearestPoint.z - testPoint.z, 2))
                     / workspaceDiagonalSquared);

    double h = .2*h1 + .2*h2 + .2*h3 + .2*h4 + .2*h5;
    vector<double> hComponents;
    hComponents.push_back(h1);
    hComponents.push_back(h2);
    hComponents.push_back(h3);
    hComponents.push_back(h4);
    hComponents.push_back(h5);

    PoseWithHeuristic rankedPose(finalPoses.poses[i], h, hComponents);
    rankedFinalPoses.push_back(rankedPose);
  }

  sort(rankedFinalPoses.begin(), rankedFinalPoses.end());
  result.graspList.poses.clear();
  result.graspList.header.frame_id = commonFrame;
  result.heuristicList.clear();
  for (unsigned int i = 0; i < rankedFinalPoses.size(); i ++)
  {
    result.graspList.poses.push_back(rankedFinalPoses[i].pose);
    rail_grasp_calculation_msgs::Heuristics tempHs;
    tempHs.heuristics = rankedFinalPoses[i].hComponents;
    result.heuristicList.push_back(tempHs);
  }

  rankGraspsObjectServer.setSucceeded(result);
}

void GraspSampler::rankGraspsPOI(const rail_grasp_calculation_msgs::RankGraspsGoalConstPtr &goal)
{
  rail_grasp_calculation_msgs::RankGraspsFeedback feedback;
  rail_grasp_calculation_msgs::RankGraspsResult result;

  //transform point cloud to the frame of the grasps
  sensor_msgs::PointCloud2 transformedCloud;
  pcl::PCLPointCloud2::Ptr tempCloud(new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  transformedCloud.header.frame_id = goal->graspList.header.frame_id;
  pcl_ros::transformPointCloud(transformedCloud.header.frame_id, cloud, transformedCloud, tfListener);
  pcl_conversions::toPCL(transformedCloud, *tempCloud);
  pcl::fromPCLPointCloud2(*tempCloud, *convertedCloud);

  //crop point cloud to workspace
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cropToWorkspace(goal->workspace, convertedCloud, croppedCloud);

  ROS_INFO("Received %lu grasps...", goal->graspList.poses.size());

  ROS_INFO("Merging similar poses...");
  geometry_msgs::PoseArray finalPoses = goal->graspList;
  clusterPoses(finalPoses.poses);

  ROS_INFO("Found %lu poses!", finalPoses.poses.size());
  if (finalPoses.poses.empty())
  {
    rankGraspsPOIServer.setSucceeded(result);
    return;
  }

  // Pose heuristics
  pcl::SACSegmentation<pcl::PointXYZRGB> planeSegmenter;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Extract the dominant plane
  planeSegmenter.setOptimizeCoefficients(true);
  planeSegmenter.setModelType(pcl::SACMODEL_PLANE);
  planeSegmenter.setMethodType(pcl::SAC_RANSAC);
  planeSegmenter.setDistanceThreshold(0.005);
  planeSegmenter.setInputCloud(croppedCloud);
  planeSegmenter.segment(*inliers, *coefficients);
  bool planeFound = !inliers->indices.empty();

  // cluster within window
  vector<pcl::PointIndices> clusterIndices;
  vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
  vector<pcl::search::KdTree<pcl::PointXYZRGB>::Ptr> searchTrees;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clusterer;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  if (removeTable)
  {
    //remove table plane
    pcl::ModelCoefficients::Ptr tableCoefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr tableInliers (new pcl::PointIndices);
    planeSegmenter.setDistanceThreshold(0.01);
    planeSegmenter.setInputCloud(convertedCloud);
    planeSegmenter.segment(*tableInliers, *tableCoefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extractTable;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tableRemovedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    extractTable.setInputCloud(convertedCloud);
    extractTable.setIndices(tableInliers);
    extractTable.setNegative(true);
    extractTable.filter(*tableRemovedCloud);

    //re-crop
    cropToWorkspace(goal->workspace, tableRemovedCloud, croppedCloud);
  }
  if (croppedCloud->size() == 0)
  {
    ROS_INFO("No points in plane-removed point cloud, cannot calculate grasps...");
    rankGraspsPOIServer.setSucceeded(result);
    return;
  }
  kdTree->setInputCloud(croppedCloud);
  clusterer.setInputCloud(croppedCloud);
  clusterer.setClusterTolerance(0.01);
  clusterer.setMinClusterSize(20);
  clusterer.setMaxClusterSize(10000);
  clusterer.setSearchMethod(kdTree);
  clusterer.extract(clusterIndices);

  for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it ++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit ++)
    {
      tempCluster->points.push_back(croppedCloud->points[*pit]);
    }
    tempCluster->width = tempCluster->points.size();
    tempCluster->height = 1;
    tempCluster->is_dense = true;
    clusters.push_back(tempCluster);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    searchTree->setInputCloud(tempCluster);
    searchTrees.push_back(searchTree);
  }

  if (clusters.empty())
  {
    ROS_INFO("No clusters could be extracted after plane removal, cannot calculate grasps...");
    rankGraspsPOIServer.setSucceeded(result);
    return;
  }

  vector<PoseWithHeuristic> rankedFinalPoses;
  for (unsigned int i = 0; i < finalPoses.poses.size(); i ++)
  {
    //heuristic 1: perpendicular to plane
    double h1 = 1;
    if (planeFound)
    {
      h1 = heuristicNormal(finalPoses.poses[i], coefficients->values);
    }

    //heuristic 2: alignment with principal component analysis
    double minClusterDst = numeric_limits<double>::max();
    int minClusterIndex = 0;
    for (unsigned int j = 0; j < clusters.size(); j ++)
    {
      vector<int> kIndices;
      vector<float> kSqrDistances;
      pcl::PointXYZRGB testPoint;
      testPoint.x = static_cast<float>(finalPoses.poses[i].position.x);
      testPoint.y = static_cast<float>(finalPoses.poses[i].position.y);
      testPoint.z = static_cast<float>(finalPoses.poses[i].position.z);
      kIndices.resize(1);
      kSqrDistances.resize(1);
      searchTrees[j]->nearestKSearch(testPoint, 1, kIndices, kSqrDistances);
      if (kSqrDistances[0] < minClusterDst)
      {
        minClusterDst = kSqrDistances[0];
        minClusterIndex = j;
      }
    }

    //heuristic 2: alignment with principle direction
    double h2 = heuristicAlignment(finalPoses.poses[i], clusters[minClusterIndex]);

    //heuristic 3: distance from clicked point
    double h3 = sqrt(squaredDistance(finalPoses.poses[i].position, goal->workspace.roiCenter)) /
                sqrt(pow(goal->workspace.roiDimensions.x/2.0, 2)
                     + pow(goal->workspace.roiDimensions.y/2.0, 2)
                     + pow(goal->workspace.roiDimensions.z/2.0, 2));

    double h = 0.6*h1 + 0.25*h2 + 0.15*h3;
    PoseWithHeuristic rankedPose(finalPoses.poses[i], h);
    rankedFinalPoses.push_back(rankedPose);
  }

  sort(rankedFinalPoses.begin(), rankedFinalPoses.end());
  result.graspList.poses.clear();
  result.graspList.header.frame_id = goal->graspList.header.frame_id;
  result.heuristicList.clear();
  for (unsigned int i = 0; i < rankedFinalPoses.size(); i ++)
  {
    result.graspList.poses.push_back(rankedFinalPoses[i].pose);
    rail_grasp_calculation_msgs::Heuristics tempHs;
    tempHs.heuristics = rankedFinalPoses[i].hComponents;
    result.heuristicList.push_back(tempHs);
  }

  rankGraspsPOIServer.setSucceeded(result);
}

void GraspSampler::rankGraspsScene(const rail_grasp_calculation_msgs::RankGraspsGoalConstPtr &goal)
{
  rail_grasp_calculation_msgs::RankGraspsResult result;

  string commonFrame = goal->sceneCloud.header.frame_id;
  ROS_INFO("COMMON FRAME: %s", commonFrame.c_str());
  pcl::PCLPointCloud2::Ptr tempCloud(new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr environmentCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl_conversions::toPCL(goal->sceneCloud, *tempCloud);
  pcl::fromPCLPointCloud2(*tempCloud, *environmentCloud);

  ROS_INFO("Received %lu grasps...", goal->graspList.poses.size());

  ROS_INFO("Merging similar poses...");
  geometry_msgs::PoseArray finalPoses = goal->graspList;
  preprocessPoses(finalPoses, commonFrame);
  if (finalPoses.poses.empty())
  {
    rankGraspsSceneServer.setSucceeded(result);
    return;
  }

  // Set up commonly used objects
  pcl::SACSegmentation<pcl::PointXYZRGB> planeSegmenter;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // segmentaion parameters
  planeSegmenter.setOptimizeCoefficients(true);
  planeSegmenter.setModelType(pcl::SACMODEL_PLANE);
  planeSegmenter.setMethodType(pcl::SAC_RANSAC);
  planeSegmenter.setDistanceThreshold(0.005);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clusterer;
  clusterer.setClusterTolerance(0.01);
  clusterer.setMinClusterSize(20);
  clusterer.setMaxClusterSize(30000);

  vector<PoseWithHeuristic> rankedFinalPoses;

  for (size_t i = 0; i < finalPoses.poses.size(); i ++)
  {
    geometry_msgs::Pose testPose = finalPoses.poses[i];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr localCroppedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cropAtPoint(testPose.position, localSize, environmentCloud, localCroppedCloud);

    // cluster local cloud to find an "object"
    vector<pcl::PointIndices> clusterIndices;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    vector<pcl::search::KdTree<pcl::PointXYZRGB>::Ptr> searchTrees;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdTree->setInputCloud(localCroppedCloud);
    clusterer.setInputCloud(localCroppedCloud);
    clusterer.setSearchMethod(kdTree);
    clusterer.extract(clusterIndices);

    for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it ++)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit ++)
      {
        tempCluster->points.push_back(localCroppedCloud->points[*pit]);
      }
      tempCluster->width = tempCluster->points.size();
      tempCluster->height = 1;
      tempCluster->is_dense = true;
      clusters.push_back(tempCluster);

      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
      searchTree->setInputCloud(tempCluster);
      searchTrees.push_back(searchTree);
    }

    //Set object cloud as closest cluster
    if (clusters.empty())
    {
      continue;
    }
    double minClusterDst = numeric_limits<double>::max();
    int minClusterIndex = 0;
    for (unsigned int j = 0; j < clusters.size(); j ++)
    {
      vector<int> kIndices;
      vector<float> kSqrDistances;
      pcl::PointXYZRGB testPoint;
      testPoint.x = static_cast<float>(testPose.position.x);
      testPoint.y = static_cast<float>(testPose.position.y);
      testPoint.z = static_cast<float>(testPose.position.z);
      kIndices.resize(1);
      kSqrDistances.resize(1);
      searchTrees[j]->nearestKSearch(testPoint, 1, kIndices, kSqrDistances);
      if (kSqrDistances[0] < minClusterDst)
      {
        minClusterDst = kSqrDistances[0];
        minClusterIndex = j;
      }
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    objectCloud = clusters[minClusterIndex];

    // Heuristic calculation

    //heuristic 1: perpendicular to global (environment) plane
    //Extract the global (environment) dominant plane
    vector<float> globalPlane;
    globalPlane.resize(4);
    planeSegmenter.setInputCloud(environmentCloud);
    planeSegmenter.segment(*inliers, *coefficients);

    double h1 = 1;
    if (!inliers->indices.empty())
    {
      h1 = heuristicNormal(testPose, coefficients->values);
    }

    //heuristic 2: perpendicular to closest local (object) plane
    pcl::PointXYZRGB testPoint;
    testPoint.x = static_cast<float>(finalPoses.poses[i].position.x);
    testPoint.y = static_cast<float>(finalPoses.poses[i].position.y);
    testPoint.z = static_cast<float>(finalPoses.poses[i].position.z);

    //fit local plane
    planeSegmenter.setInputCloud(objectCloud);
    planeSegmenter.segment(*inliers, *coefficients);

    double h2 = 1;
    if (!inliers->indices.empty())
    {
      h2 = heuristicNormal(testPose, coefficients->values);
    }

    //heuristic 3: alignment with principal component analysis
    double h3 = heuristicAlignment(testPose, objectCloud);

    //heuristic 4: distance from object center (a parametrization of grasp location)
    geometry_msgs::Point centerPoint;
    pcl::PointXYZRGB minPointObject, maxPointObject;
    pcl::getMinMax3D(*objectCloud, minPointObject, maxPointObject);
    centerPoint.x = (minPointObject.x + maxPointObject.x)/2.0;
    centerPoint.y = (minPointObject.y + maxPointObject.y)/2.0;
    centerPoint.z = (minPointObject.z + maxPointObject.z)/2.0;
    double maxDiagonalSquared = 3*pow(localSize, 2);
    double h4 = sqrt(squaredDistance(centerPoint, testPose.position)
                     / maxDiagonalSquared);

    //heuristic 5: distance to closest point in object cloud
    vector<int> kIndices;
    vector<float> kSqrDistances;
    searchTrees[minClusterIndex]->nearestKSearch(testPoint, 1, kIndices, kSqrDistances);
    pcl::PointXYZRGB nearestPoint = objectCloud->at(static_cast<size_t>(kIndices.at(0)));

    double h5 = sqrt((pow(nearestPoint.x - testPoint.x, 2)
                      + pow(nearestPoint.y - testPoint.y, 2)
                      + pow(nearestPoint.z - testPoint.z, 2))
                     /maxDiagonalSquared);

    double h = .2*h1 + .2*h2 + .2*h3 + .2*h4 + .2*h5;
    vector<double> hComponents;
    hComponents.push_back(h1);
    hComponents.push_back(h2);
    hComponents.push_back(h3);
    hComponents.push_back(h4);
    hComponents.push_back(h5);

    PoseWithHeuristic rankedPose(finalPoses.poses[i], h, hComponents);
    rankedFinalPoses.push_back(rankedPose);
  }

  sort(rankedFinalPoses.begin(), rankedFinalPoses.end());
  result.graspList.poses.clear();
  result.graspList.header.frame_id = commonFrame;
  result.heuristicList.clear();
  for (unsigned int i = 0; i < rankedFinalPoses.size(); i ++)
  {
    result.graspList.poses.push_back(rankedFinalPoses[i].pose);
    rail_grasp_calculation_msgs::Heuristics tempHs;
    tempHs.heuristics = rankedFinalPoses[i].hComponents;
    result.heuristicList.push_back(tempHs);
  }

  rankGraspsSceneServer.setSucceeded(result);
}


// *******************************************************************************************************************
//                                            Grasp Preprocessing
// *******************************************************************************************************************

void GraspSampler::preprocessPoses(geometry_msgs::PoseArray &poseList, string commonFrame)
{
  clusterPoses(poseList.poses);

  ROS_INFO("Found %lu poses!", poseList.poses.size());
  if (poseList.poses.empty())
  {
    return;
  }

  //transform poses to object coordinate frame
  if (poseList.header.frame_id != commonFrame)
  {
    for (size_t i = 0; i < poseList.poses.size(); i++)
    {
      geometry_msgs::PoseStamped poseIn, poseOut;
      poseIn.header.frame_id = poseList.header.frame_id;
      poseIn.header.stamp = ros::Time(0);
      poseIn.pose = poseList.poses[i];
      poseOut.header.frame_id = commonFrame;
      poseOut.header.stamp = ros::Time(0);
      tfListener.transformPose(commonFrame, poseIn, poseOut);
      poseList.poses[i] = poseOut.pose;
    }
    poseList.header.frame_id = commonFrame;
  }
}

void GraspSampler::clusterPoses(vector<geometry_msgs::Pose> &graspPoses)
{
  vector<geometry_msgs::Pose> clusteredPoses;
  random_shuffle(graspPoses.begin(), graspPoses.end());
  for (size_t i = 0; i < graspPoses.size(); i ++)
  {
    //find close poses
    vector<geometry_msgs::Pose> poseCluster;
    vector<size_t> poseClusterIndices;
    poseCluster.push_back(graspPoses[i]);
    poseClusterIndices.push_back(i);
    for (unsigned int j = 0; j < graspPoses.size(); j ++)
    {
      if (i == j)
        continue;

      if (squaredDistance(graspPoses[i].position, graspPoses[j].position) < neighborhoodRadius &&
          squaredDistance(graspPoses[i].orientation, graspPoses[j].orientation) < orientationThreshold)
      {
        poseCluster.push_back(graspPoses[j]);
        poseClusterIndices.push_back(j);
      }
    }

    //add new pose to final pose list
    if (poseCluster.size() > clusterSize)
    {
      size_t clusterSize = poseCluster.size();
      geometry_msgs::Pose averagePose;
      sort(poseClusterIndices.begin(), poseClusterIndices.end());
      int slerpCounter = 1;
      tf::Quaternion avgQuat, newQuat;
      for (int j = (int)poseClusterIndices.size() - 1; j >= 0; j --)
      {
        averagePose.position.x += graspPoses[poseClusterIndices[j]].position.x;
        averagePose.position.y += graspPoses[poseClusterIndices[j]].position.y;
        averagePose.position.z += graspPoses[poseClusterIndices[j]].position.z;
        if (slerpCounter == 1)
        {
          tf::quaternionMsgToTF(graspPoses[poseClusterIndices[j]].orientation, avgQuat);
        }
        else
        {
          tf::quaternionMsgToTF(graspPoses[poseClusterIndices[j]].orientation, newQuat);
          avgQuat.slerp(newQuat, 1.0/((double)slerpCounter)).normalize();
        }
        slerpCounter ++;

        graspPoses.erase(graspPoses.begin() + poseClusterIndices[j]);
        if (j <= i)
          i --;
      }
      averagePose.position.x /= (double)clusterSize;
      averagePose.position.y /= (double)clusterSize;
      averagePose.position.z /= (double)clusterSize;
      tf::quaternionTFToMsg(avgQuat, averagePose.orientation);
      clusteredPoses.push_back(averagePose);
    }
  }
  graspPoses = clusteredPoses;
}


// *******************************************************************************************************************
//                                           Heuristic Calculation
// *******************************************************************************************************************

double GraspSampler::heuristicNormal(geometry_msgs::Pose pose, vector<float> &plane)
{
  Eigen::Vector3d xVector, planeNormal;
  xVector[0] = 1;
  xVector[1] = 0;
  xVector[2] = 0;
  planeNormal[0] = plane[0];
  planeNormal[1] = plane[1];
  planeNormal[2] = plane[2];
  planeNormal.normalize();

  Eigen::Quaterniond poseQuaternion;
  Eigen::Matrix3d poseRotationMatrix;
  tf::quaternionMsgToEigen(pose.orientation, poseQuaternion);
  poseRotationMatrix = poseQuaternion.toRotationMatrix();
  Eigen::Vector3d testXVector = poseRotationMatrix * xVector;
  testXVector.normalize();

  double angle = acos(testXVector.dot(planeNormal));
  angle = min(angle, M_PI - angle);
  return angle / (M_PI/2.0);
}

double GraspSampler::heuristicAlignment(geometry_msgs::Pose pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  Eigen::Quaterniond principalDirection = computePrincipalDirection(cloud);

  Eigen::Vector3d yVector, zVector;
  yVector[0] = 0;
  yVector[1] = 1;
  yVector[2] = 0;
  zVector[0] = 0;
  zVector[1] = 0;
  zVector[2] = 1;

  Eigen::Quaterniond poseQuaternion;
  Eigen::Matrix3d poseRotationMatrix;
  tf::quaternionMsgToEigen(pose.orientation, poseQuaternion);
  poseRotationMatrix = poseQuaternion.toRotationMatrix();
  Eigen::Vector3d testYVector = poseRotationMatrix * yVector;
  Eigen::Vector3d testZVector = poseRotationMatrix * zVector;
  Eigen::Vector3d principalYVector = principalDirection*yVector;
  Eigen::Vector3d principalZVector = principalDirection*zVector;
  testYVector.normalize();
  testZVector.normalize();
  principalYVector.normalize();
  principalZVector.normalize();

  double yyAngle = fmod(acos(testYVector.dot(principalYVector)), M_PI/2.0);
  double zzAngle = fmod(acos(testZVector.dot(principalZVector)), M_PI/2.0);
  double yzAngle = fmod(acos(testYVector.dot(principalZVector)), M_PI/2.0);
  double zyAngle = fmod(acos(testZVector.dot(principalYVector)), M_PI/2.0);
  yyAngle = min(M_PI/2.0 - yyAngle, yyAngle);
  zzAngle = min(M_PI/2.0 - zzAngle, zzAngle);
  yzAngle = min(M_PI/2.0 - yzAngle, yzAngle);
  zyAngle = min(M_PI/2.0 - zyAngle, zyAngle);
  return max((yyAngle + zzAngle)/(M_PI/2.0), (yzAngle + zyAngle)/(M_PI/2.0));
}


// *******************************************************************************************************************
//                                          Point Cloud Processing
// *******************************************************************************************************************

void GraspSampler::cropToWorkspace(rail_grasp_calculation_msgs::Workspace workspace,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut)
{
  pcl::CropBox<pcl::PointXYZRGB> cropBox;
  Eigen::Vector4f minPoint, maxPoint;
  if (workspace.mode == rail_grasp_calculation_msgs::Workspace::CENTERED_ROI)
  {
    minPoint[0] = (float)(workspace.roiCenter.x - workspace.roiDimensions.x/2.0);
    minPoint[1] = (float)(workspace.roiCenter.y - workspace.roiDimensions.y/2.0);
    minPoint[2] = (float)(workspace.roiCenter.z - workspace.roiDimensions.z/2.0);
    maxPoint[0] = (float)(workspace.roiCenter.x + workspace.roiDimensions.x/2.0);
    maxPoint[1] = (float)(workspace.roiCenter.y + workspace.roiDimensions.y/2.0);
    maxPoint[2] = (float)(workspace.roiCenter.z + workspace.roiDimensions.z/2.0);
  }
  else
  {
    minPoint[0] = (float)(workspace.x_min);
    minPoint[1] = (float)(workspace.y_min);
    minPoint[2] = (float)(workspace.z_min);
    maxPoint[0] = (float)(workspace.x_max);
    maxPoint[1] = (float)(workspace.y_max);
    maxPoint[2] = (float)(workspace.z_max);
  }
  cropBox.setMin(minPoint);
  cropBox.setMax(maxPoint);
  cropBox.setInputCloud(cloudIn);
  cropBox.filter(*cloudOut);
}

void GraspSampler::cropAtPoint(geometry_msgs::Point point, float cropSize,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut)
{
  //crop to local region around grasp point
  pcl::CropBox<pcl::PointXYZRGB> cropBox;
  Eigen::Vector4f minPoint, maxPoint;
  minPoint[0] = static_cast<float>(point.x) - cropSize;
  minPoint[1] = static_cast<float>(point.y) - cropSize;
  minPoint[2] = static_cast<float>(point.z) - cropSize;
  maxPoint[0] = static_cast<float>(point.x) + cropSize;
  maxPoint[1] = static_cast<float>(point.y) + cropSize;
  maxPoint[2] = static_cast<float>(point.z) + cropSize;
  cropBox.setMin(minPoint);
  cropBox.setMax(maxPoint);
  cropBox.setInputCloud(cloudIn);
  cropBox.filter(*cloudOut);
}

Eigen::Quaterniond GraspSampler::computePrincipalDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // compute principal direction
  Eigen::Matrix3d covariance;
  Eigen::Vector4d centroid;
  pcl::PointXYZRGB minPoint, maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  centroid[0] = (minPoint.x + maxPoint.x)/2.0;
  centroid[1] = (minPoint.y + maxPoint.y)/2.0;
  centroid[2] = (minPoint.z + maxPoint.z)/2.0;
  //pcl::compute3DCentroid(*cloud, centroid);
  pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3d eig_dx = eigen_solver.eigenvectors();
  eig_dx.col(2) = eig_dx.col(0).cross(eig_dx.col(1));

  //final transform
  const Eigen::Quaterniond qfinal(eig_dx);

  return qfinal;
}


// *******************************************************************************************************************
//                                          Supporting Calculations
// *******************************************************************************************************************

double GraspSampler::squaredDistance(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
  return min(pow(q1.x - q2.x, 2) + pow(q1.y - q2.y, 2) + pow(q1.z - q2.z, 2) + pow(q1.w - q2.w, 2),
    pow(q1.x + q2.x, 2) + pow(q1.y + q2.y, 2) + pow(q1.z + q2.z, 2) + pow(q1.w + q2.w, 2));
}

double GraspSampler::squaredDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_sampler");
  GraspSampler gs;

  ros::spin();

  return EXIT_SUCCESS;
}