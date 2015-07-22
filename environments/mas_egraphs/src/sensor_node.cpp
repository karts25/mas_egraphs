#include <mas_egraphs/sensor_node.h>
SensorNode::SensorNode(costmap_2d::Costmap2DROS* costmap_ros){
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  
  numgoals_ = 1;
  numagents_ = 1;
  costmap_ros_ = costmap_ros;
  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cost_map_);
  
  int lethal_obstacle;
  private_nh.param("lethal_obstacle", lethal_obstacle, 20);
  lethal_obstacle_ = (unsigned char) lethal_obstacle;
  inscribed_inflated_obstacle_ = lethal_obstacle_-1;

  costmap_grid_.resize(cost_map_.getSizeInCellsX(), std::vector<int>(cost_map_.getSizeInCellsY()));
  for(unsigned int ix = 0; ix < cost_map_.getSizeInCellsX(); ix++){
    for(unsigned int iy = 0; iy < cost_map_.getSizeInCellsY(); iy++){
      unsigned char c = costMapCostToSBPLCost(cost_map_.getCost(ix,iy));
      env_->UpdateCost(ix, iy, c);
      if(c >= inscribed_inflated_obstacle_)
	costmap_grid_[ix][iy] = true;
    }
  }

  sensor_update_service_ = nh.advertiseService("/mas_egraphs/sensor", 
					       &SensorNode::sensor_update, this);
}

void SensorNode::setNumAgents(int numagents){
  numagents_ = numagents;
}

void SensorNode::setNumGoals(int numgoals){
  numgoals_ = numgoals;
}

void SensorNode::sensor_update(mas_egraphs::GetSensorUpdate::Request& req,
			       mas_egraphs::GetSensorUpdate::Response& res){	

  double x = req.x;
  double y = req.y;
  double z = req.z;
  double theta = req.theta;
  
  res.pointcloud.points.clear();
  res.pointcloud.channels.clear();

  ros::Time sim_time = ros::Time::now();  
  res.header.stamp = sim_time;
  res.header.frame_id = costmap_ros_->getGlobalFrameID();
  res.header.seq = req.agentID;

  // "sense" all obstacles at some radius around the robot. TODO: make raytrace
  int id = 0;
  for(unsigned int ix = std::min(0, x - SENSOR_RADIUS);
      ix < std::min(cost_map_.getSizeInCellsX(), x + SENSOR_RADIUS); ix++){
    for(unsigned int iy = std::min(0, y - SENSOR_RADIUS); 
	iy < std::min(cost_map_.getSizeInCellsY(), y + SENSOR_RADIUS); iy++){
      geometry_msgs::Point32 point;
      sensor_msgs::ChannelFloat32 channel;
      marker.id = id; 
      id++;
      point.x = ix;
      point.y = iy;
      point.z = 0;	
      
      double cost = cost_map_.getCost(ix, iy);
      channel.name = "intensity";
      channel.values.push_back(cost);
    }
    res.pointcloud.points.push_back(point);
    res.pointcloud.channels.push_back(channel);
  } 
}
