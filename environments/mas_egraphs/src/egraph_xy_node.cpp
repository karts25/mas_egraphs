 #include <mas_egraphs/egraph_xy_node.h>
#include <nav_msgs/Path.h>
using namespace std;

EGraphXYNode::EGraphXYNode(int agentID, costmap_2d::Costmap2DROS* costmap_ros) {

  agentID_ = agentID;

  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  
  private_nh.param("num_agents", numagents_, 1);
  primitive_filenames_.reserve(numagents_);
  for(int agent_i = 0; agent_i < numagents_; agent_i++){
    std::string prim_fname_agent("primitive_filename_");
    prim_fname_agent += std::to_string(agent_i+1);
    //SBPL_INFO("reading motion primitives from %s", prim_fname_agent.c_str());
    std::string temp;
    private_nh.param(prim_fname_agent, temp, std::string(""));
    primitive_filenames_.push_back(temp);
  }
  double timetoturn45degsinplace_secs;
  private_nh.param("time_per_action", time_per_action_, 6.0);
  private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);
  //private_nh.param("sMotPrimFiles", sMotPrimFiles_, NULL);
  int lethal_obstacle;
  private_nh.param("lethal_obstacle",lethal_obstacle, 20);
  lethal_obstacle_ = (unsigned char) lethal_obstacle;
  inscribed_inflated_obstacle_ = lethal_obstacle_-1;
  sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
  
  costmap_ros_ = costmap_ros;
  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cost_map_);
  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
  
  env_ = new EGraphXY();

  std::vector<std::vector<sbpl_2Dpt_t> > perimeterptsV(numagents_);
  for(int agent_i=0; agent_i < numagents_; agent_i++){
    perimeterptsV[agent_i].reserve(footprint.size());    
    for (size_t ii(0); ii < footprint.size(); ++ii) {
      sbpl_2Dpt_t pt;
      pt.x = footprint[ii].x;
      pt.y = footprint[ii].y;
      perimeterptsV[agent_i].push_back(pt);
    }
  }
  bool ret;
  try{
    ret = env_->InitializeEnv(costmap_ros_->getSizeInCellsX(), // width
			      costmap_ros_->getSizeInCellsY(), // height
			      0, // mapdata
			      numagents_, // numAgents
			      0.2, 0.2, 0, //goal tolerance of 20 cm
			      perimeterptsV,
    			      costmap_ros_->getResolution(), time_per_action_,
			      primitive_filenames_,
			      cost_map_.getOriginX(), cost_map_.getOriginY());
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception!");
    ret = false;
  }
  if(!ret){
    ROS_ERROR("SBPL initialization failed!");
    exit(1);
  }
  
  for (ssize_t ix(0); ix < costmap_ros_->getSizeInCellsX(); ++ix)
    for (ssize_t iy(0); iy < costmap_ros_->getSizeInCellsY(); ++iy)
      env_->UpdateCost(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)));

  egraphs_.reserve(numagents_);
  for(int agent_i = 0; agent_i < numagents_; agent_i++)
    egraphs_.push_back(new EGraph(env_, 4, 0));
  

  interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1, &EGraphXYNode::interruptPlannerCallback,this);
  plan_pub_ = nh.advertise<visualization_msgs::MarkerArray>("mas_plan", 1);
  plan_service_ = nh.advertiseService("/sbpl_planning/plan_path",
				      &EGraphXYNode::startMASPlanner, this);
  //footprint_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("footprint", 10);

  ros::service::waitForService("/mas_egraphs/sensorupdate",10);
  sensorupdate_client_ = ros::NodeHandle().serviceClient<mas_egraphs::GetSensorUpdate>('/mas_egraphs/sensorupdate', true);

}

void EGraphXYNode::interruptPlannerCallback(std_msgs::EmptyConstPtr){
  ROS_WARN("Planner interrupt received!");
  planner_->interrupt();
}

//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char EGraphXYNode::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;
  else
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
  
}

bool EGraphXYNode::makePlan(EGraphReplanParams& params){
  if (!replan_required_)
    return;
  
  // set start state
  do{
    int retid = env_->SetStart(state_.poses, state_.goalsVisited);
    if(retid < 0 || planner_->set_start(retid) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  }
  
  egraph_mgr_->updateHeuristicGrids(heur_grid_);
  
  // plan!
  std::vector<int> solution_stateIDs;
  bool ret = planner_->replan(&solution_stateIDs, params);
  env_->PrintTimingStats();
  egraph_mgr_->printTimingStats();

  // execute
  if(ret){
    publishPath(solution_stateIDs, res);    
    ret = execute(solution_stateIDs);
  }
  if(!ret)
    return false;
  // unset replan_required
  replan_required_ = false;
}

bool EGraphXYNode::startMASPlanner(mas_egraphs::GetXYThetaPlan::Request& req, 
			    mas_egraphs::GetXYThetaPlan::Response& res){
  ROS_DEBUG("[sbpl_lattice_planner] getting fresh copy of costmap");
  costmap_ros_->clearRobotFootprint();
  ROS_DEBUG("[sbpl_lattice_planner] robot footprint cleared");

  costmap_ros_->getCostmapCopy(cost_map_);
  try{
  bool ret = env_->SetNumGoals(req.num_goals);
  if (!ret)
    {
      SBPL_PRINTF("Invalid number of goals");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the number of goals");
    return false;
  }
  numgoals_ = req.num_goals;

  heurs_.resize(numagents_);
  for(int agent_i=0; agent_i < numagents_; agent_i++){
    heurs_[agent_i].resize(req.num_goals);
    for(int goal_i=0; goal_i < req.num_goals; goal_i ++){
      heurs_[agent_i][goal_i] = new EGraphMAS2dGridHeuristic(*env_, costmap_ros_->getSizeInCellsX(),
							     costmap_ros_->getSizeInCellsY(), 1);
    }
  }

  //publish goals
  visualization_msgs::MarkerArray goals;
  ros::Time req_time = ros::Time::now();
  int id = 0;
  for(int goal_i = 0; goal_i < req.num_goals; goal_i++){
    visualization_msgs::Marker marker;
    marker.id = id; 
    id++;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0;
    marker.color.g = 1;
    marker.color.a = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.header.stamp = req_time;
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    marker.pose.position.x = req.goal_x[goal_i];
    marker.pose.position.y = req.goal_y[goal_i];
    marker.pose.position.z = 0;
    goals.markers.push_back(marker);
  }
   SBPL_INFO("Publishing goals");
   plan_pub_.publish(goals);
 
  // publish start
   if (((int)req.start_x.size() != numagents_) || ((int)req.start_y.size() != numagents_)){
     SBPL_ERROR("Incorrect number of start locations");
     return false;
   }
   
   visualization_msgs::MarkerArray starts;  
   for(int agent_i = 0; agent_i < numagents_; agent_i++){
     visualization_msgs::Marker marker;
     marker.id = id; 
     id++;
     marker.scale.x = 0.4;
     marker.scale.y = 0.4;
     marker.scale.z = 0;
     marker.color.r = 1;
     marker.color.a = 1;
     marker.type = visualization_msgs::Marker::SPHERE;
     marker.header.stamp = req_time;
     marker.header.frame_id = costmap_ros_->getGlobalFrameID();
     marker.pose.position.x = req.start_x[agent_i];
     marker.pose.position.y = req.start_y[agent_i];
     marker.pose.position.z = 0;
     starts.markers.push_back(marker);
   }
   SBPL_INFO("Publishing starts");
   plan_pub_.publish(starts);

   for(int agent_i=0; agent_i < numagents_; agent_i++){
     state_.poses[agent_i].x = req.start_x[agent_i] - cost_map_.getOriginX();
     state_.poses[agent_i].y = req.start_y[agent_i] - cost_map_.getOriginY();
     state_.poses[agent_i].z = req.start_z[agent_i];
     state_.poses[agent_i].theta = req.start_theta[agent_i];
   }

  //publishfootprints(poses_start);

   egraph_mgr_ = new EGraphManager<std::vector<int> > (egraphs_, env_, heurs_,
						       req.num_goals, numagents_);
   planner_ = new LazyAEGPlanner<std::vector<int> >(env_, true, egraph_mgr_);

  try{
    std::vector<pose_cont_t> goal_shifted(req.num_goals);
    for(int goal_i = 0; goal_i < req.num_goals; goal_i++){
	goal_shifted[goal_i].x = req.goal_x[goal_i] - cost_map_.getOriginX();
        goal_shifted[goal_i].y = req.goal_y[goal_i] - cost_map_.getOriginY();
	goal_shifted[goal_i].z = req.goal_z[goal_i];
	goal_shifted[goal_i].theta = req.goal_theta[goal_i];
      }
    int ret = env_->SetGoal(goal_shifted);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }
  
  EGraphReplanParams params(10.0);
  params.initial_eps = req.initial_eps;
  params.dec_eps = req.dec_eps;
  params.final_eps = req.final_eps;
  params.epsE = req.egraph_eps;
  params.dec_epsE = req.dec_egraph_eps;
  params.final_epsE = req.final_egraph_eps;
  params.return_first_solution = false;
  params.use_egraph = req.use_egraph;
  params.feedback_path = req.feedback_path;

  replan_required_ = true;
  bool ret = makePlan(params);
  return ret;  
}


void EGraphXYNode::publishAgentPoses() const{
  visualization_msgs::MarkerArray gui_path;
  int id = numgoals_;
  for(int agent_i = 0; agent_i < numagents_; agent_i++){
    visualization_msgs::Marker marker;
    marker.id = id;
    id++;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0;
    marker.color.b = agentID_%2;
    marker.color.r = agentID_%2 + 1;
    marker.color.a = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();

    contPosetoGUIPose(state_->poses[agent_i].x, marker.pose);  
  }
}

void EGraphXYNode::contPosetoGUIPose(const pose_cont_t& pose, geometry_msgs::pose& GUIPose){
  GUIPose.position.x = pose.x + cost_map_.getOriginX();
  GUIPose.position.y = pose.y + cost_map_.getOriginY();
  GUIPose.position.z = pose.z;
  
  tf::Quarternion temp;
  temp.setEulerZYX(state_->poses[agent_i].theta, 0, 0);
  marker.pose.orientation.x = temp.getX();
  marker.pose.orientation.y = temp.getY();
  marker.pose.orientation.z = temp.getZ();
  marker.pose.orientation.w = temp.getW();
}

void EGraphXYNode::publishPath(std::vector<int>& solution_stateIDs){
  visualization_msgs::MarkerArray gui_path;
  vector<double> coord;
  int id = numgoals_ + numagents_;
  ros::Time plan_time = ros::Time::now();  

  for(int agent_i = 0; agent_i < numagents_; agent_i++)
    {
      for(unsigned int i=0; i < solution_stateIDs.size(); i++){    
	coord.clear();
       	env_->getCoord(solution_stateIDs[i], coord);       

	visualization_msgs::Marker marker;
	marker.id = id; 
	id++;
	marker.scale.x = 0.1; 
	marker.scale.y = 0.1;       
	marker.scale.z = 0;
	marker.color.b = agentID_%2;
	marker.color.r = agentID_%2 + 1;
	marker.color.a = 1;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.header.stamp = plan_time;
	marker.header.frame_id = costmap_ros_->getGlobalFrameID();

	pose_cont_t pose;
	pose.x = coord[4*agent_i];
	pose.y = coord[4*agent_i+1];
	pose.z = coord[4*agent_i+2];
	pose.theta = coord[4*agent_i+3];

	contPosetoGUIPose(pose, marker.pose);
	gui_path.markers.push_back(marker);
      }
    }
  plan_pub_.publish(gui_path);
}

bool EGraphXYNode::execute(const std::vector<int>& solution_stateIDs){
  
  for(unsigned int step = 0; step < solution_stateIDs.size(); step++){
    // get sensor reading for current agent position
#ifdef SIM
    mas_egraphs::GetSensorUpdate::Request req;
    mas_egraphs::GetSensorUpdate::Response res;
    std::vector<double> coord;
    env->getCoord(solution_stateIDs_V[step], poses, goalsVisited, activeAgents);
    req.agentID = agentID_;
    req.x = poses[agentID_].x;
    req.y = poses[agentID_].y;    
    req.z = poses[agentID_].z;
    req.theta = poses[agentID_].theta;
    sensorupdate_client_.call(req, res);

    // update costs according to new sensor information
    updatelocalMap(res.pointcloud);
#endif

    // if old plan is invalid, or a new communication is recieved, we want to replan
    if(!env->isValidPlan(solution_stateIDs_V))
      replan_required_ = true;
   
    ros::Duration(time_per_action_).sleep();    
  }
  return true;
}

void EGraphXYNode::updatelocalMap(sensor_msgs::PointCloud& pointcloud){
  for(unsigned int i = 0; i < pointcloud.points.size(); i++){
    int x = (int) pointcloud.points[i].x;
    int y = (int) pointcloud.points[i].y;
    unsigned char c = costMapCostToSBPLCost(cost_map_.getCost(x, y));
    if(c >= inscribed_inflated_obstacle_){
      if (!heur_grid_[x][y]){ // new obstacle
	std::vector<int> obstacle(2);
	obstacle[1] = x;
	obstacle[2] = y;
	comm_packet_.newobstacles.append(obstacle);       
	updateCosts(x, y, c);	
      }
    }
  }
}

void EGraphXYNoed::updateCosts(int x, int y, unsigned char c = inscribed_inflated_obstacle_) const{
  heur_grid_[x][y] = true;
  env_->UpdateCost(x,y, c);
  egraph_mgr_->updateHeuristicGrids(heur_grid_);
}

void EGraphXYNode::sendCommunication() const{
  mas_msgs::MasComm comm_msg;
  comm_msg.header.seq = state_.lastpacketID;
  comm_msg.header.frame_id = costmap_ros_->getGlobalFrameID(); 
  comm_msg.header.stamp = ros::Time::now();
  comm_msg.agentID = agentID_;
  for(int i = 0; i < state_.new_obstacles.size(); i++){
    comm_msg.obstacles_x.push_back(state_.new_obstacles[i][0]);
    comm_msg.obstacles_y.push_back(state_.new_obstacles[i][1]);
  }
  comm_msg.x = state_.poses[state_.agentID].x;
  comm_msg.y = state_.poses[state_.agentID].y;
  comm_msg.z = state_.poses[state_.agentID].z;
  comm_msg.theta = state_.poses[state_.agentID].theta;
  for(int goal_i = 0; goal_i < numgoals_; goal_i++){
    if(state_.visitedGoals_indices[goal_i] == state_.agentID)
      comm_msg.goalsVisited.push_back(goal_i);
  }
  
  comm_pub_.publish(comm_msg);
  
  // increment packetID and clear new obtacles
  state_.lastpacketID++;
  exec_state_t.new_obstacles.clear();
}

void EGraphXYNode::receiveCommunication(const mas_egraphs::MasComm::ConstPtr& msg){
  if(msg->agentID == agentID_)
    return;
  
  // update robot pose
  state_.poses[msg->agentID].x = msg->x;
  state_.poses[msg->agentID].y = msg->y;
  state_.poses[msg->agentID].z = msg->z;
  state_.poses[msg->agentID].theta = msg->theta;

  // update costs
  for(unsigned int obstacle_i = 0; obstacle_i < msg->obstacles_x.size(); obstacle_i++){
    updateCosts(msg->obstacles_x[obstacle_i], msg->obstacles_y[obstacle_i]);
  }

  // also send back communication, if we haven't replied yet
  if(state_.lastpacketID < msg->header.seq)
    sendCommunication();

  replan_required_ = true;
}

void EGraphXYNode::publishfootprints(std::vector<pose_cont_t> poses) const{
  std::vector<sbpl_2Dpt_t> footprint;
  geometry_msgs::PolygonStamped PolygonStamped;
  ros::Time req_time = ros::Time::now();
  PolygonStamped.header.frame_id = costmap_ros_->getGlobalFrameID();
  for(int agent_i = 0; agent_i < numagents_; agent_i++){
    PolygonStamped.header.stamp =  req_time;
    PolygonStamped.header.seq = agent_i;
    footprint.clear();
    PolygonStamped.polygon.points.clear();
    env_->GetRobotFootprint(agent_i, poses[agent_i], footprint);
    printf("Footprint of Agent %d: ", agent_i);
    for(unsigned int i = 0; i < footprint.size(); i++){
      geometry_msgs::Point32 point;
      point.x = footprint[i].x - cost_map_.getOriginX();
      point.y = footprint[i].y - cost_map_.getOriginY();
      point.z = poses[agent_i].z; 
      PolygonStamped.polygon.points.push_back(point);
      printf("(%f %f %f) ", point.x, point.y, point.z);
    }
    printf("\n");
    footprint_pub_.publish(PolygonStamped);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "mas_egraphs_node");

  tf::TransformListener tf_;
  costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  costmap->pause();

  EGraphXYNode xy(costmap);

  //ros::spin();
  ros::MultiThreadedSpinner spinner(2);//need 2 threads to catch a the interrupt
  spinner.spin();
}

