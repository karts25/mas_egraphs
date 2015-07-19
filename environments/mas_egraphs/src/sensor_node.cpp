#include <mas_egraphs/simulator.h>

SensorNode::SensorNode(costmap_2d::Costmap2DROS* costmap_ros, int agentID){
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  
  numgoals_ = 1;

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

  obstacles_pub_ = nh.advertise<sensor_msgs::PointCloud>("laserscan");
  plan_sub_ = nh.subscribe("mas_plan", 1000, &Simulator::Simulate, this);
}

void SensorNode::setNumAgents(int agentID){
  agentID_ = agentID;;
}

void SensorNode::setNumGoals(int numgoals){
  numgoals_ = numgoals;
}

void SensorNode::simulate(const visualization_msgs::MarkerArray& plan){	
  visualization_msgs::Marker pos;
  
  for(int t = 0; t < plan.markers.size(); t++){
    ros::Time sim_time = ros::Time::now();  
    sensor_msgs::PointCloud pointcloud;
    pos = plan.markers[t];
    int id = 0;
    
    pointcloud.header.stamp = sim_time;
    pointcloud.header.frame_id = costmap_ros_->getGlobalFrameID();

    // "sense" all obstacles at some radius around the robot. TODO: make raytrace
    for(unsigned int ix = std::min(0, pos.position.x - SENSOR_RADIUS);
	ix < std::min(cost_map_.getSizeInCellsX(), pos.position.x + SENSOR_RADIUS); ix++){
      for(unsigned int iy = std::min(0, pos.position.y - SENSOR_RADIUS); 
	  iy < std::min(cost_map_.getSizeInCellsY(), pos.position.y + SENSOR_RADIUS); iy++){
	geometry_msgs::Point32 point;
	sensor_msgs::ChannelFloat32 channel;
	marker.id = id; 
	id++;
	point.x = ix;
	point.y = iy;
	point.z = 0;	

	double cost = cost_map_.getCost(ix, iy)/255;
	channel.name = "intensity";
	channel.values.push_back(cost);
      }
      pointcloud.push_back(point);
      channels.push_back(channel);
    }

    obstacles_pub.publish(pointcloud);
  }
  std::cin.get();
}
