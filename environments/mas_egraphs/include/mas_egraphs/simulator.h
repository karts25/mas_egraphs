#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <cotsmap_2d/costmap_2d_ros.h>

class Simulator{
 public:
  Simulator(costmap_2d::Costmap2DROS* costmap_ros);
  setNumAgents(int numagents);
  setNumGoals(int numgoals);
  void simulate(const visualization_msgs::MarkerArray& plan);

 private:
  int numagents_;
  int numgoals_;
  int time_per_step_;
  costmap_2d::Costmap2DROS* costmap_ros; /**< manages the cost map for us */
  costmap_2d::Costmap2D cost_map_; /**< local copy of the costmap underlying cost_map_ros_ */ 
  std::vector<std::vector<int> > costmap_grid_;

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;

  ros::Publisher obstacles_pub_; 
  ros::Subscriber plan_sub_;
}
