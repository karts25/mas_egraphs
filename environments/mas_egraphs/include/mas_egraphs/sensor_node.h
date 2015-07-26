#ifndef SENSOR_NODE_H
#define SENSOR_NODE_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <cmath>
#include <costmap_2d/costmap_2d_ros.h>
#include <mas_egraphs/GetSensorUpdate.h>

#define SENSOR_RADIUS 6 // in meters

class SensorNode{
 public:
  SensorNode(costmap_2d::Costmap2DROS* costmap_ros);
  bool sensor_update(mas_egraphs::GetSensorUpdate::Request& req,
		     mas_egraphs::GetSensorUpdate::Response& res);


 private:
  int time_per_step_;
  int sensor_radius_; // in cells
  int inflated_radius_; // in cells
  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */
  costmap_2d::Costmap2D cost_map_; /**< local copy of the costmap underlying cost_map_ros_ */ 
  std::vector<std::vector<int> > costmap_grid_;
  ros::ServiceServer sensor_update_service_;
};

#endif
