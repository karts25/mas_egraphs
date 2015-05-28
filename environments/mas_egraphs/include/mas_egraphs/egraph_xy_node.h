#ifndef EGRAPH_XY_NODE_H
#define EGRAPH_XY_NODE_H

#ifndef ROS
#define ROS
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

class EGraphXYNode{
 public:
  EGraphXYNode(costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(mas_egraphs::GetXYThetaPlan::Request& req, 
		mas_egraphs::GetXYThetaPlan::Response& res);
  
 private:
  unsigned char costMapCostToSBPLCost(unsigned char newcost);
  
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char sbpl_cost_multiplier_;
  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */
  costmap_2d::Costmap2D cost_map_;        /**< local copy of the costmap underlying cost_map_ros_ */
  std::vector<geometry_msgs::Point> footprint_;
  
  int numagents_;
  int numgoals_;
  EGraphXY* env_;
  std::vector<EGraph*> egraphs_;
  EGraphMAS2dGridHeuristic* heur_;
  std::vector<std::vector<EGraphMAS2dGridHeuristic*> > heurs_;
  EGraphManager<std::vector<int> >* egraph_mgr_;
  LazyAEGPlanner<std::vector<int> >* planner_;
  EGraphVisualizer* egraph_vis_;
  
  ros::Publisher plan_pub_;
  ros::ServiceServer plan_service_;
  
  ros::Subscriber interrupt_sub_;
  void interruptPlannerCallback(std_msgs::EmptyConstPtr);
  bool simulate(std::vector<double> start_x, std::vector<double> start_y, EGraphReplanParams params, mas_egraphs::GetXYThetaPlan::Response& res, int maxtime);
  void publishPath(std::vector<int>& solution_stateIDs, mas_egraphs::GetXYThetaPlan::Response& res);
};

#endif

