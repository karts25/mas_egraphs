#include <mas_egraphs/sensor_node.h>
SensorNode::SensorNode(costmap_2d::Costmap2DROS* costmap_ros){
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  costmap_ros_ = costmap_ros;
  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cost_map_);

  sensor_update_service_ = nh.advertiseService("/mas_egraphs/sensor", 
					       &SensorNode::sensor_update, this);
  sensor_radius_ = (int) std::ceil(SENSOR_RADIUS/cost_map_.getResolution());
  inflated_radius_ = sensor_radius_ + 
    (int) std::ceil(cost_map_.getInflationRadius()/cost_map_.getResolution());
  //  printf("SensorNode: sensor_r = %d, inflated_r = %d resolution = %f \n", sensor_radius_, inflated_radius_, cost_map_.getResolution());
}

bool SensorNode::sensor_update(mas_egraphs::GetSensorUpdate::Request& req,
			       mas_egraphs::GetSensorUpdate::Response& res){	
  int x = req.x;
  int y = req.y;
  int z = req.z;
  //double theta = req.theta;
  res.pointcloud.points.clear();
  res.pointcloud.channels.clear();

  ros::Time sim_time = ros::Time::now();  
  res.pointcloud.header.stamp = sim_time;
  res.pointcloud.header.frame_id = costmap_ros_->getGlobalFrameID();
  res.pointcloud.header.seq = req.agentID;

  // "sense" all obstacles at some radius around the robot. TODO: make raytrace
  int id = 0;
  sensor_msgs::ChannelFloat32 channel;
  channel.name = "intensity";
  for(unsigned int ix = std::max(0, x - sensor_radius_);
      ix < std::min((int) cost_map_.getSizeInCellsX(), x + sensor_radius_); ix++){
    for(unsigned int iy = std::max(0, y - sensor_radius_); 
	iy < std::min((int) cost_map_.getSizeInCellsY(), y + sensor_radius_); iy++){
      double cost = cost_map_.getCost(ix, iy);
      if(cost >= costmap_2d::LETHAL_OBSTACLE){
	channel.values.push_back(cost);
	geometry_msgs::Point32 point;
	point.x = ix;
	point.y = iy;
	point.z = z;	
	res.pointcloud.points.push_back(point);
      }
    }
  }
  
  // now get all inflated costs in the sensed region
  for(unsigned int ix = std::max(0, x - inflated_radius_);
      ix < std::min((int) cost_map_.getSizeInCellsX(), x + inflated_radius_); ix++){
    for(unsigned int iy = std::max(0, y - inflated_radius_); 
	iy < std::min((int) cost_map_.getSizeInCellsY(), y + inflated_radius_); iy++){      	
      double cost = cost_map_.getCost(ix, iy);
      if((cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) && 
	 (cost < costmap_2d::LETHAL_OBSTACLE)){	 
	channel.values.push_back(cost);
	geometry_msgs::Point32 point;
	point.x = ix;
	point.y = iy;
	point.z = z;
	res.pointcloud.points.push_back(point);
      }
    }
  }
  res.pointcloud.channels.push_back(channel);
  return true; 
}

int main(int argc, char** argv){
  ros::init(argc, argv, "mas_sensor_node");

  tf::TransformListener tf_;
  costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  costmap->pause();

  SensorNode xy(costmap);

  ros::spin();
  //ros::MultiThreadedSpinner spinner(2);//need 2 threads to catch the interrupt
  //spinner.spin();
}

