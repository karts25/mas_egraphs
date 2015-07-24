#ifndef EGRAPH_XY_NODE_H
#define EGRAPH_XY_NODE_H

#ifndef ROS
#define ROS
#endif

#ifndef SIM
#define SIM
#endif

#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <mas_egraphs/egraph_xy.h>
#include <mas_egraphs/egraphManager.h>
#include <mas_egraphs/egraph_planner.h>
#include <mas_egraphs/egraph_mas_2d_grid_heuristic.h>

#include <mas_egraphs/GetXYThetaPlan.h>
#include <mas_egraphs/GetSensorUpdate.h>
#include <mas_egraphs/MasComm.h>
#include <nav_msgs/Path.h>
typedef struct
{
  //poses of all agents (only pose[agentID_] is fully observed)
  std::vector<pose_cont_t> poses; 
  //goalsVisited[i] = agentID_ of agent that first visited the goal, or -1 if unvisited. (only goalsVisited[agentID_] is fully observed)
  std::vector<int> goalsVisited; // stores indices of agents that visit each goal.
} belief_state_t;

// elements in this struct are always true
typedef struct
{
  //id of last packet sent by all agents (fully observed)
  std::vector<int> lastpacketID_V;
  //obstacle locations seen by this agent since last communication (fully observed)
  std::vector<std::vector<int> > new_obstacles;
  //goals known to be visited
  std::vector<int> goalsVisited;
  //assignments of goals to agents. -1 if unassigned
  std::vector<int> assignments; 
} observed_state_t;

// everything used for visualization
typedef struct
{
  int last_plan_markerID_; // stores number of markers used to publish the previous plan
  int last_obst_markerID_;
}viz_t;

class EGraphXYNode
{
 public:
  EGraphXYNode(costmap_2d::Costmap2DROS* costmap_ros);
  void startMASPlanner(const mas_egraphs::GetXYThetaPlan::ConstPtr& msg);
  void receiveCommunication(const mas_egraphs::MasComm::ConstPtr& msg);
  void sendCommunication();

 private:
  int agentID_;
  bool replan_required_; // true when we need to replan
  belief_state_t belief_state_; 
  observed_state_t observed_state_;
  viz_t viz_;
  int last_plan_markerID_; 
  unsigned char costMapCostToSBPLCost(unsigned char newcost);
  
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */
  std::vector<std::string> primitive_filenames_; /** where to find the motion primitives for the current robots */
  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char sbpl_cost_multiplier_;
  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */
  costmap_2d::Costmap2D cost_map_;        /**< local copy of the costmap underlying cost_map_ros_ */
  std::vector<geometry_msgs::Point> footprint_;
  std::vector<std::vector<bool> > heur_grid_;
  std::vector<char*> sMotPrimFiles_;
  int numagents_;
  int numgoals_;
  double time_per_action_;
  EGraphXY* env_;
  std::vector<EGraph*> egraphs_;
  EGraphMAS2dGridHeuristic* heur_;
  std::vector<std::vector<EGraphMAS2dGridHeuristic*> > heurs_;
  EGraphManager<std::vector<int> >* egraph_mgr_;
  LazyAEGPlanner<std::vector<int> >* planner_;
  EGraphVisualizer* egraph_vis_;
  
  ros::Publisher plan_pub_; // publish plan for Rviz
  //  ros::Publisher footprint_pub_; // publish footprint for Rviz
  ros::Publisher comm_pub_;  // publish communication packet for other robots
  
  //ros::ServiceServer plan_service_;
  ros::ServiceClient sensorupdate_client_;
  ros::Subscriber makeplan_sub_;
  ros::Subscriber interrupt_sub_;
  ros::Subscriber comm_sub_;

  void updatelocalMap(sensor_msgs::PointCloud& pointcloud);
  void updateCosts(int x, int y);
  void updateCosts(int x, int y, unsigned char c);
  void interruptPlannerCallback(std_msgs::EmptyConstPtr);
  void publishfootprints(std::vector<pose_cont_t> poses) const;
  bool makePlan(EGraphReplanParams& params, std::vector<int>& solution_stateIDs);
  
  // loops between execution and replanning until execute signals completion
  bool agentManager(EGraphReplanParams& params);
  // executes plan. returns true if plan executed to completion, else false
  bool execute(const std::vector<int>& solution_stateIDs_V);
  // publish this agent's belief of the world
  void visualizeCommPackets() const;
  void visualizePoses() const;
  void visualizePath(std::vector<int>& solution_stateIDs);
  void contPosetoGUIPose(const pose_cont_t& pose, visualization_msgs::Marker& GUIMarker) const;
  void waitforReplies() const;
};

#endif

