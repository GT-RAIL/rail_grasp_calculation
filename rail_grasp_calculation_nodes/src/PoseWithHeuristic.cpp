#include "rail_grasp_calculation_nodes/PoseWithHeuristic.h"

using namespace std;

PoseWithHeuristic::PoseWithHeuristic(geometry_msgs::Pose pose, double h)
{
  this->pose = pose;
  this->h = h;
}

PoseWithHeuristic::PoseWithHeuristic(geometry_msgs::Pose pose, double h, std::vector<double> hComponents)
{
  this->pose = pose;
  this->h = h;
  this->hComponents = hComponents;
}

bool PoseWithHeuristic::operator < (const PoseWithHeuristic obj) const
{
  return this->h < obj.h;
}
