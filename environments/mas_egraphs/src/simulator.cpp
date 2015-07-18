#include <mas_egraphs/simulator.h>

Simulator::Simulator(costmap_2d::Costmap2DROS* costmap_ros){
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  
  numagents_ = 1;
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

void Simulator::setNumAgents(int numagents){
  numagents_ = numagents;
}

void Simulator::setNumGoals(int numgoals){
  numgoals_ = numgoals;
}

void Simulator::Simulate(const visualization_msgs::MarkerArray& plan){
  for(int t = 0; t < plan.markers.size(); t++){
    
  }
}
