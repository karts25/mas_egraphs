#include <mas_egraphs/sensor_node.h>
SensorNode::SensorNode(costmap_2d::Costmap2DROS* costmap_ros){
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  
  numgoals_ = 1;
  numagents_ = 1;
  costmap_ros_ = costmap_ros;
  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cost_map_);
  sensor_update_service_ = nh.advertiseService("/mas_egraphs/sensor", 
					       &SensorNode::sensor_update, this);
}

void SensorNode::setNumAgents(int numagents){
  numagents_ = numagents;
}

void SensorNode::setNumGoals(int numgoals){
  numgoals_ = numgoals;
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
  for(unsigned int ix = std::max(0, x - SENSOR_RADIUS);
      ix < std::min((int) cost_map_.getSizeInCellsX(), x + SENSOR_RADIUS); ix++){
    for(unsigned int iy = std::max(0, y - SENSOR_RADIUS); 
	iy < std::min((int) cost_map_.getSizeInCellsY(), y + SENSOR_RADIUS); iy++){
      geometry_msgs::Point32 point;
      point.x = ix;
      point.y = iy;
      point.z = z;	
      
      double cost = cost_map_.getCost(ix, iy);
      channel.values.push_back(cost);
      res.pointcloud.points.push_back(point);
    }
  }
  res.pointcloud.channels.push_back(channel);
  return true; 
}

int main(int argc, char** argv){
  ros::init(argc, argv, "mas_egraphs_node");

  tf::TransformListener tf_;
  costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  costmap->pause();

  SensorNode xy(costmap);

  ros::spin();
  //ros::MultiThreadedSpinner spinner(2);//need 2 threads to catch the interrupt
  //spinner.spin();
}

