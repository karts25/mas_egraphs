#ifndef SENSOR_NODE_H
#define SENSOR_NODE_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <mas_egraphs/GetSensorUpdate.h>

#define SENSOR_RADIUS 5

class SensorNode{
 public:
  SensorNode(costmap_2d::Costmap2DROS* costmap_ros);
  void setNumAgents(int numagents);
  void setNumGoals(int numgoals);
  void sensor_update(mas_egraphs::GetSensorUpdate::Request& req,
		     mas_egraphs::GetSensorUpdate::Response& res);


 private:
  int agentID_;
  int numagents_;
  int numgoals_;
  int time_per_step_;
  costmap_2d::Costmap2DROS* costmap_ros; /**< manages the cost map for us */
  costmap_2d::Costmap2D cost_map_; /**< local copy of the costmap underlying cost_map_ros_ */ 
  std::vector<std::vector<int> > costmap_grid_;
  ros::ServiceServer sensor_update_service_;
}

#endif
