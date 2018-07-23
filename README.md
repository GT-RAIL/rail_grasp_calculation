# rail_grasp_calculation
Metapackage for grasp pose clustering and heuristic calculation for grasp ranking, as well as supporting messages
for connecting to grasp samplers and calling the grasp ranking actions.  More details on the specific methods are
upcoming in a future publication.

## Description
Grasp ranking can be performed for three different situations, depending on the application and system setup.  Each
method, at a high level, takes a set of sampled grasp poses, clusters them by similar position and orientation, and
calculates a ranking by calculating and combining a set of heuristics that describe the relationship between the grasp
pose and some local point cloud information.  The methods differ in the type of point cloud information used as input,
and each is implemented as a ROS action server detailed below. Grasps can be ranked for either: a point cloud for a
single segmented object (`rank_grasps_object`), a point cloud of an entire scene (`rank_grasps_scene`) where everything
can be grasped (e.g. for clearing a cluttered scene), and a local point cloud centered around a point-of-interest
(`rank_grasps_poi`).

This package is generalized and is designed to rank grasps regardless of the robot executing them, and is designed to
work with grasp poses coming from any source, such as a grasp sampler or a database of grasps.  As an example (and to
provide a complete pipeline), we developed the package using the antipodal grasp sampler originally included in the
[AGILE grasp ROS package](https://github.com/atenpas/agile_grasp), modified by the RAIL lab to interface with this
package.  The antipodal grasp sampler can be found [here](https://github.com/GT-RAIL/rail_agile).  If you'd like to use
a different source of grasps, you may want to change the `cluster_size` parameter described in the `grasp_sampler` node
section, depending on the grasp density. 

## Menu
* [rail_grasp_calculation_msgs](#rail_grasp_calculation_msgs)
* [rail_grasp_calculation_nodes](#rail_grasp_calculation_nodes)
  * [grasp_sampler](#grasp_sampler)
* [Installation](#Installation)
* [Usage](#Usage)
 

## rail_grasp_calculation_msgs
This package contains action definitions, messages, and services used for grasp sampling and ranking.  For more details,
check the comments within the message definitions themselves.  The primary actions are as follows:
* [RankGrasps.action](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/RankGrasps.action):
Action definition for all grasp ranking.  Intended usage: requires an unranked grasp list and point cloud information
as input, and returns a list of ranked grasps.
* [SampleGrasps.action](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/SampleGrasps.action):
Action interface for initial grasp sampling.  This is optionally included for implementation of new ROS packages, and
is not used directly in this package.  Intended usage: requires a point cloud and a workspace as input, and returns a list of grasps sampled from
the given point cloud within the given workspace.  For an example, see the [rail_agile](https://github.com/GT-RAIL/rail_agile)
package.

## rail_grasp_calculation_nodes
The primary node of this package, [grasp_sampler](#grasp_sampler), implements grasp clustering and ranking based on a
set of heuristics designed to select effective grasps for man-made objects.  More details on the heuristics used for the
point-of-interest (POI) method can be found in our paper, A Comparison of Remote Robot Teleoperation Interfaces for
General Object Manipulation, published in HRI2017.  More details on the heuristics used for the object and scene methods
are upcoming in a future publication.

### grasp_sampler
This node performs grasp candidate evaluation by computing a set of heuristics on grasp poses sampled from a given set
of grasp hypotheses computed from an outside source, such as the [rail_agile](https://github.com/GT-RAIL/rail_agile)
package.  Relevant parameters, action servers, topics, and services are as follows:
* **Action Servers**
  * `rank_grasps_object`([rail_grasp_calculation_msgs/RankGraspsAction](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/RankGrasps.action))  
  Rank grasps for a specific object.  Requires an unranked list of sampled grasps, a
  point cloud of a scene containing the object of interest, and a segmented point cloud of only the object of interest
  (with the rest of the scene information removed).  Returns a ranked list of grasps, as well as the heuristic values 
  calculated for each grasp.
  * `rank_grasps_scene`([rail_grasp_calculation_msgs/RankGraspsAction](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/RankGrasps.action))  
  Rank grasps calculated over a full scene.  Requires an unranked list of sampled
  grasps, and a point cloud of a scene the grasps were sampled from.  Returns a ranked list of grasps, as well as the
  heuristic values calculated for each grasp.
  * `rank_grasps_poi`([rail_grasp_calculation_msgs/RankGraspsAction](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/RankGrasps.action))  
  Rank grasps calculated around a point-of-interest.  Requires an unranked list of
  sampled grasps, a point cloud of a scene, and a workspace centered on the point-of-interest within which the grasps
  were calculated.  Returns a ranked list of grasps, as well as the heuristic values calculated for each grasp.
* **Parameters**
  * `neighborhood_radius`(double, 0.02)  
  Radius defining the sphere of a grasp's neighborhood, in meters, used for
  clustering grasp hypotheses.
  * `orientation_threshold`(double, 0.1)  
  Maximum difference, in radians, between two grasps' orientations to be clustered.
  * `cluster_size`(int, 5)  
  Maximum number of grasp hypotheses that can compose one cluster.
  * `remove_table`(boolean, false)  
  Option to remove the dominant plane from the point cloud within the given 
  workspace before calculating the object orientaion heuristic. It's recommended to set this to true if the grasps
  are being used for tabletop pick-and-place applications.  NOTE: this is only used for the point-of-interest method.
  * `local_window_size`(float, 0.015f)  
  Edge length of a cropping volume used to create a local window at each grasp to
  calculate heuristics for the scene method.

## Installation
**Standalone**:  
Clone the package into your catkin workspace and build it as follows:
```bash
cd (your catkin workspace)/src
git clone https://github.com/GT-RAIL/rail_grasp_calculation.git
cd ..
catkin_make
```  
  
**With `rail_agile`**:  
Clone both the `rail_grasp_calculation` metapackage and `rail_agile` package into your catkin workspace and build it as
follows:
```bash
cd (your catkin workspace)/src
git clone https://github.com/GT-RAIL/rail_agile.git
git clone https://github.com/GT-RAIL/rail_grasp_calculation.git
cd ..
catkin_make
```

## Usage
**Standalone**:  
Run the grasp sampler node with the following, optionally setting parameters with rosrun:
```bash
rosrun rail_grasp_calculation_nodes grasp_sampler
```
To rank grasps, create an action client that connects to one of the three action servers listed above depending on the
method you would like to use.  Pass in a goal including an unranked list of grasp poses and the required point cloud
information for the method you choose, and get ranked grasps from the result.
  
**With `rail_agile`**:  
This package contains a launch file that will run the grasp sampling node as well as everything you need for antipodal
grasp sampling from the `rail_agile` package:
```bash
roslaunch rail_grasp_calculation_nodes find_grasps.launch
```
To sample and rank grasps, create an action client that connects to the `rail_agile` antipodal grasp sampler as
described in the `rail_agile` package documentation.  Create another action client that connects to the `grasp_sampler`
action server for the method you would like to use.  Pass in a goal containing the grasp list from the antipodal grasp
sampler result and the required point cloud information for the method you choose, and get ranked grasps from the
result.
