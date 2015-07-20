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
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <mas_egraphs/GetXYThetaPlan.h>
#include <mas_egraphs/egraph_xy.h>
#include <mas_egraphs/egraphManager.h>
#include <mas_egraphs/egraph_planner.h>
#include <mas_egraphs/egraph_mas_2d_grid_heuristic.h>
#include <mas_egraphs/MasComm.h>

typedef struct{
  int agentID;
  unsigned int lastpacketID; // id of last packet sent
  std::vector<std::vector<int> > new_obstacles; // store obstacles found since last communication
  std::vector<pose_cont_t> poses;
  std::vector<pose_cont_t> goals;
  std::vector<int> goalsVisited; 
  std::vector<int> assignments;
} exec_state_t;

class EGraphXYNode{
 public:
  EGraphXYNode(int agentID, costmap_2d::Costmap2DROS* costmap_ros);
  bool startMASPlanner(mas_egraphs::GetXYThetaPlan::Request& req, 
		       mas_egraphs::GetXYThetaPlan::Response& res);
  void receiveCommunication(const mas_egraphs::MasComm::Constptr& msg);
  void sendCommunication() const;

 private:

  bool replan_required_; // true when we need to replan
  std::vector<pose_t> robotposes_;
  exec_state_t state_; // stores state information known to this agent

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
  std::vector<pose_t> agentposes_;
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
  
  ros::ServiceServer plan_service_;
  ros::ServiceClient sensorupdate_client_;
  
  ros::Subscriber interrupt_sub_;
  ros::Subscriber comm_sub_;

  void interruptPlannerCallback(std_msgs::EmptyConstPtr);
  void publishfootprints(std::vector<pose_cont_t> poses) const;
  bool EGraphXYNode::makePlan(EGraphReplanParams& params);
  bool execute(const std::vector<int>& solution_stateIDs);
  // publish this agent's belief of the world
  void contPosetoGUIPose(const pose_cont_t& pose, geometry_msgs::pose& GUIPose);
  void publishPath(std::vector<int>& solution_stateIDs);
};

#endif

