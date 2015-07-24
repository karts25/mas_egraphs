#include <mas_egraphs/egraph_xy_node.h>

using namespace std;

EGraphXYNode::EGraphXYNode(costmap_2d::Costmap2DROS* costmap_ros) {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  
  private_nh.param("num_agents", numagents_, 1);
  private_nh.param("agentID", agentID_, 1);
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
  
  heur_grid_.resize(cost_map_.getSizeInCellsX());
  for (ssize_t ix(0); ix < costmap_ros_->getSizeInCellsX(); ++ix){
    heur_grid_[ix].resize(cost_map_.getSizeInCellsY());
    for (ssize_t iy(0); iy < costmap_ros_->getSizeInCellsY(); ++iy){
      updateCosts(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)));
    }
  }
  egraphs_.reserve(numagents_);
  for(int agent_i = 0; agent_i < numagents_; agent_i++)
    egraphs_.push_back(new EGraph(env_, 4, 0));

  // create subscribers and publishers
  interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1, &EGraphXYNode::interruptPlannerCallback,this);
  comm_sub_ = nh.subscribe("/mas_egraphs/mas_comm", 1, &EGraphXYNode::receiveCommunication, this);
  makeplan_sub_ = nh.subscribe("/mas_egraphs/mas_plan_req", 1, &EGraphXYNode::startMASPlanner, this);

  plan_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mas_egraphs/mas_plan", 1);
  comm_pub_ = nh.advertise<mas_egraphs::MasComm>("/mas_egraphs/mas_comm", 1);
  viz_.last_plan_markerID_ = 0;
  //footprint_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("footprint", 10);

  ros::service::waitForService("mas_egraphs/sensor",10);
  sensorupdate_client_ = ros::NodeHandle().serviceClient<mas_egraphs::GetSensorUpdate>("/mas_egraphs/sensor", true);

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

bool EGraphXYNode::makePlan(EGraphReplanParams& params, std::vector<int>& solution_stateIDs){
  if (!replan_required_)
    return true;
  printf("Agent %d starting plan with goalsVisited = ", agentID_);
  egraph_mgr_->printVector(belief_state_.goalsVisited);
  int retid = env_->SetStart(belief_state_.poses, belief_state_.goalsVisited);
  if(retid < 0 || planner_->set_start(retid) == 0){
    ROS_ERROR("ERROR: failed to set start state\n");
    return false;
  }
  
  egraph_mgr_->updateHeuristicGrids(heur_grid_);
  
  // plan!
  solution_stateIDs.clear();
  bool ret = planner_->replan(&solution_stateIDs, params);
  //env_->PrintTimingStats();
  //egraph_mgr_->printTimingStats();

  if(ret){
    visualizePath(solution_stateIDs);    
    // unset replan_required
    replan_required_ = false;
    return true;
  }
  else 
    return false;
}

void EGraphXYNode::startMASPlanner(const mas_egraphs::GetXYThetaPlan::ConstPtr& msg){
  ROS_DEBUG("[sbpl_lattice_planner] getting fresh copy of costmap");
  costmap_ros_->clearRobotFootprint();
  ROS_DEBUG("[sbpl_lattice_planner] robot footprint cleared");

  costmap_ros_->getCostmapCopy(cost_map_);
  try{
  bool ret = env_->SetNumGoals(msg->num_goals);
  if (!ret)
    {
      SBPL_PRINTF("Invalid number of goals");
      return;// false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the number of goals");
    return;// false;
  }
  numgoals_ = msg->num_goals;

  heurs_.resize(numagents_);
  for(int agent_i=0; agent_i < numagents_; agent_i++){
    heurs_[agent_i].resize(msg->num_goals);
    for(int goal_i=0; goal_i < msg->num_goals; goal_i ++){
      heurs_[agent_i][goal_i] = new EGraphMAS2dGridHeuristic(*env_, costmap_ros_->getSizeInCellsX(),
							     costmap_ros_->getSizeInCellsY(), 1);
    }
  }

  //publish goals
  visualization_msgs::MarkerArray goals;
  ros::Time msg_time = ros::Time::now();
  int id = 0;
  for(int goal_i = 0; goal_i < msg->num_goals; goal_i++){
    visualization_msgs::Marker marker;
    marker.id = id; 
    id++;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0;
    marker.color.g = 1;
    marker.color.a = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.header.stamp = msg_time;
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    marker.pose.position.x = msg->goal_x[goal_i];
    marker.pose.position.y = msg->goal_y[goal_i];
    marker.pose.position.z = 0;
    goals.markers.push_back(marker);
  }
   SBPL_INFO("Publishing goals");
   plan_pub_.publish(goals);
 
  // publish start
   if (((int)msg->start_x.size() != numagents_) || ((int)msg->start_y.size() != numagents_)){
     SBPL_ERROR("Incorrect number of start locations");
     return;// false;
   }
   // initialize belief state
   belief_state_.poses.resize(numagents_);
   for(int agent_i=0; agent_i < numagents_; agent_i++){
     belief_state_.poses[agent_i].x = msg->start_x[agent_i] - cost_map_.getOriginX();
     belief_state_.poses[agent_i].y = msg->start_y[agent_i] - cost_map_.getOriginY();
     belief_state_.poses[agent_i].z = msg->start_z[agent_i];
     belief_state_.poses[agent_i].theta = msg->start_theta[agent_i];
   }
   visualizePoses();
   belief_state_.goalsVisited.resize(numgoals_, -1);
   
   // initialize observed state
   observed_state_.lastpacketID_V.resize(numagents_, -1);
   observed_state_.new_obstacles.clear();
   observed_state_.goalsVisited.resize(numgoals_, -1);
   observed_state_.assignments.resize(numgoals_, -1);

   egraph_mgr_ = new EGraphManager<std::vector<int> > (egraphs_, env_, heurs_,
						       msg->num_goals, numagents_);
   planner_ = new LazyAEGPlanner<std::vector<int> >(env_, true, egraph_mgr_);

  try{
    std::vector<pose_cont_t> goal_shifted(msg->num_goals);
    for(int goal_i = 0; goal_i < msg->num_goals; goal_i++){
	goal_shifted[goal_i].x = msg->goal_x[goal_i] - cost_map_.getOriginX();
        goal_shifted[goal_i].y = msg->goal_y[goal_i] - cost_map_.getOriginY();
	goal_shifted[goal_i].z = msg->goal_z[goal_i];
	goal_shifted[goal_i].theta = msg->goal_theta[goal_i];
      }
    int ret = env_->SetGoal(goal_shifted);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return;// false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return;// false;
  }
  
  EGraphReplanParams params(10.0);
  params.initial_eps = msg->initial_eps;
  params.dec_eps = msg->dec_eps;
  params.final_eps = msg->final_eps;
  params.epsE = msg->egraph_eps;
  params.dec_epsE = msg->dec_egraph_eps;
  params.final_epsE = msg->final_egraph_eps;
  params.return_first_solution = false;
  params.use_egraph = msg->use_egraph;
  params.feedback_path = msg->feedback_path;

  replan_required_ = true;
  bool ret = agentManager(params);  
}


void EGraphXYNode::visualizePoses() const{
  visualization_msgs::MarkerArray gui_path;
  ros::Time time = ros::Time::now();  
  int id = numgoals_;
  for(int agent_i = 0; agent_i < numagents_; agent_i++){
    visualization_msgs::Marker marker;
    marker.ns = std::to_string(agentID_); 
    marker.id = id;
    id++;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0;
    marker.color.b = agentID_%2;
    marker.color.r = (agentID_+1)%2;
    if(agent_i == agentID_)
      marker.color.a = 1;
    else
      marker.color.a = 0.5;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.header.stamp = time;
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    contPosetoGUIPose(belief_state_.poses[agent_i], marker); 
    gui_path.markers.push_back(marker);
  }
  plan_pub_.publish(gui_path);
}

void EGraphXYNode::contPosetoGUIPose(const pose_cont_t& pose, 
				     visualization_msgs::Marker& GUIMarker) const{
  GUIMarker.pose.position.x = pose.x + cost_map_.getOriginX();
  GUIMarker.pose.position.y = pose.y + cost_map_.getOriginY();
  GUIMarker.pose.position.z = pose.z;
  
  tf::Quaternion temp;
  temp.setEulerZYX(pose.theta, 0, 0);
  GUIMarker.pose.orientation.x = temp.getX();
  GUIMarker.pose.orientation.y = temp.getY();
  GUIMarker.pose.orientation.z = temp.getZ();
  GUIMarker.pose.orientation.w = temp.getW();
}

void EGraphXYNode::visualizeCommPackets() const{
  visualization_msgs::MarkerArray gui_path;
  vector<double> coord;
  int id = numgoals_ + numagents_;
  // publish number of packets sent so far
  visualization_msgs::Marker marker;
  ros::Time plan_time = ros::Time::now();  
  marker.id = id;
  marker.header.stamp = plan_time;
  marker.header.frame_id = costmap_ros_->getGlobalFrameID();
  marker.ns = std::to_string(agentID_);
  id++;
  marker.scale.z = 1;
  marker.color.r = 1;
  marker.color.a = 1;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = std::to_string(observed_state_.lastpacketID_V[agentID_]+1);
  marker.pose.position.x = 1+agentID_ + cost_map_.getOriginX();
  marker.pose.position.y = -1 + cost_map_.getOriginY();
  marker.pose.position.z = 1;
  tf::Quaternion temp;
  temp.setEulerZYX(0, 0, 0);
  marker.pose.orientation.x = temp.getX();
  marker.pose.orientation.y = temp.getY();
  marker.pose.orientation.z = temp.getZ();
  marker.pose.orientation.w = temp.getW();
  gui_path.markers.push_back(marker);
  // publish path
  plan_pub_.publish(gui_path);
}

void EGraphXYNode::visualizePath(std::vector<int>& solution_stateIDs){
  // ids [0: (numgoals_ -1)] used to publish goals
  // ids [numgoals_ : (numgoals_:numagents_-1)] used to publish start poses
  // id numgoals_+numagents_ used to publish number of packets sent by this agent
  
  visualization_msgs::MarkerArray gui_path;
  vector<double> coord;
  int id = numgoals_ + numagents_+1;
  ros::Time plan_time = ros::Time::now();  

  for(int agent_i = 0; agent_i < numagents_; agent_i++){
      for(unsigned int i=0; i < solution_stateIDs.size(); i++){    
	coord.clear();
       	env_->getCoord(solution_stateIDs[i], coord);       

	visualization_msgs::Marker marker;
	marker.id = id; 
	marker.ns = std::to_string(agentID_);
	id++;
	marker.scale.x = 0.1; 
	marker.scale.y = 0.1;       
	marker.scale.z = 0;
	marker.color.b = agentID_%2;
	marker.color.r = (agentID_+1)%2;
	if(agent_i == agentID_)
	  marker.color.a = 1;
	else
	  marker.color.a = 0.5;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.header.stamp = plan_time;
	marker.header.frame_id = costmap_ros_->getGlobalFrameID();

	pose_cont_t pose;
	pose.x = coord[4*agent_i];
	pose.y = coord[4*agent_i+1];
	pose.z = coord[4*agent_i+2];
	pose.theta = coord[4*agent_i+3];

	contPosetoGUIPose(pose, marker);
	gui_path.markers.push_back(marker);
      }
    }

  // delete extra markers from previous round
  for(int erase_id = id; erase_id <= viz_.last_plan_markerID_; erase_id++){
    visualization_msgs::Marker marker;
    marker.id = erase_id;
    marker.ns = std::to_string(agentID_);
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    marker.action = visualization_msgs::Marker::DELETE;
    gui_path.markers.push_back(marker);
  }
  plan_pub_.publish(gui_path);
  viz_.last_plan_markerID_ = id-1;
}

bool EGraphXYNode::execute(const std::vector<int>& solution_stateIDs_V){  
  for(unsigned int step = 0; step < solution_stateIDs_V.size(); step++){
    visualizeCommPackets();
    printf("Agent %d: Executing step %d\n------------------\n", 
	   agentID_, (int)step);
    std::vector<double> coord;
    env_->getCoord(solution_stateIDs_V[step], coord);

    for(int agent_i = 0; agent_i < numgoals_; agent_i++){
      belief_state_.poses[agent_i].x = coord[4*agent_i];
      belief_state_.poses[agent_i].y = coord[4*agent_i+1];
      belief_state_.poses[agent_i].z = coord[4*agent_i+2];
      belief_state_.poses[agent_i].theta = coord[4*agent_i+3];
    }

    for(int goal_i = 0; goal_i < numgoals_; goal_i++){
      belief_state_.goalsVisited[goal_i] = coord[4*numagents_+goal_i];
    }
    
#ifdef SIM
    // localize current agent
    belief_state_.poses[agentID_].x = coord[4*agentID_];
    belief_state_.poses[agentID_].y = coord[4*agentID_+1];
    belief_state_.poses[agentID_].z = coord[4*agentID_+2];
    belief_state_.poses[agentID_].theta = coord[4*agentID_+3];

    // update goals that we know were visited by this agent
    for(int goal_i = 0; goal_i < numgoals_; goal_i++)
      if(belief_state_.goalsVisited[goal_i]==agentID_)
	observed_state_.goalsVisited[goal_i] = agentID_;

    // get sensor reading for current agent position
    mas_egraphs::GetSensorUpdate::Request req;
    mas_egraphs::GetSensorUpdate::Response res;
    req.agentID = agentID_;
    pose_cont_t pose = belief_state_.poses[agentID_];
    int theta;
    env_->PoseContToDisc(pose.x, pose.y, pose.z, pose.theta,
			 req.x, req.y, req.z, theta);
    req.theta = belief_state_.poses[agentID_].theta;
    //printf("Getting sensor info\n");
    sensorupdate_client_.call(req, res);
    //printf("Got sensor info\n");
    // update costs according to new sensor information
    updatelocalMap(res.pointcloud);    
#endif
    visualizePoses();
    // if old plan is invalid, we want to replan
    if(!env_->IsValidPlan(solution_stateIDs_V, step)){
      printf("Agent %d thinks plan is invalid\n", agentID_);
      replan_required_ = true;
    }
    ros::Duration(0.1).sleep();    
    if(replan_required_){
      printf("Agent %d: replan_required_ is true\n", agentID_);
      return false;
    }
    printf("Agent %d: packets sent: ", agentID_);
    for(int agent_i=0; agent_i< numagents_; agent_i++)
      printf("%d ", observed_state_.lastpacketID_V[agent_i]);
    printf("\n-------------------------\n");
  }
  return true;
}


bool EGraphXYNode::agentManager(EGraphReplanParams& params){
  std::vector<int> solution_stateIDs;
  bool InitialPlan = true;
  while(true){
    
    // if we "know" all goals have been visited, we are done
    if (std::all_of(observed_state_.goalsVisited.begin(),
		    observed_state_.goalsVisited.end(), 
		    [](int i){return i >= 0;}))
      break;

    bool PlanExists = makePlan(params, solution_stateIDs);
    
    // if the plan doesn't exist, return false
    if(!PlanExists)
      return false;    

    //Initiate communication if goal assignments have switched,
    
    std::vector<int> new_assignments;
    env_->getAssignments(solution_stateIDs.back(), new_assignments);
    if((!InitialPlan) && 
       (numagents_ > 1) &&
       (observed_state_.assignments != new_assignments)){
      printf("Agent %d initiating communication because assignments switched\n", agentID_);
      ros::Duration(0.1).sleep();
      sendCommunication();
    }
    observed_state_.assignments = new_assignments;
    InitialPlan = false;
    bool isComplete = execute(solution_stateIDs);

    // if we "believe" all goals have been visited, communicate
    if(isComplete){
      printf("Agent %d believes all goals visited\n", agentID_);
      ros::Duration(0.1).sleep();
      sendCommunication();
    }
    int most_recent_packetID = *std::max_element(observed_state_.lastpacketID_V.begin(), 
						 observed_state_.lastpacketID_V.end());
    // Initiate communication because we have not replied to a message sent by another agent
    if(observed_state_.lastpacketID_V[agentID_] < most_recent_packetID){
      printf("Agent %d initating communication to reply\n", agentID_);
      ros::Duration(0.1).sleep();
      sendCommunication();
    }
    ros::Duration(0.1).sleep();    
  }  
  printf("Agent %d is done\n", agentID_);
  return true;
}


void EGraphXYNode::updatelocalMap(sensor_msgs::PointCloud& pointcloud){
  for(unsigned int i = 0; i < pointcloud.points.size(); i++){
    int x = (int) pointcloud.points[i].x;
    int y = (int) pointcloud.points[i].y;
    unsigned char c = costMapCostToSBPLCost(pointcloud.channels[0].values[i]); 
    if(c >= inscribed_inflated_obstacle_){
      if (!heur_grid_[x][y]){ // new obstacle
	printf("new obstacle at (%d, %d) with cost %d\n", x,y, (int) c);
	std::vector<int> obstacle(2);
	obstacle[1] = x;
	obstacle[2] = y;
	observed_state_.new_obstacles.push_back(obstacle);       
      }
      updateCosts(x, y, c);	
    }
  }
}

void EGraphXYNode::updateCosts(int x, int y){
  updateCosts(x, y, inscribed_inflated_obstacle_);
}

void EGraphXYNode::updateCosts(int x, int y, unsigned char c){
  env_->UpdateCost(x,y,c);
  if(c >= inscribed_inflated_obstacle_){
    heur_grid_[x][y] = true; 
  }
}

void EGraphXYNode::sendCommunication(){
  mas_egraphs::MasComm comm_msg;
  // increment packetID and send
  observed_state_.lastpacketID_V[agentID_]++;
  comm_msg.header.seq = observed_state_.lastpacketID_V[agentID_];
  comm_msg.header.frame_id = costmap_ros_->getGlobalFrameID(); 
  comm_msg.header.stamp = ros::Time::now();
  comm_msg.agentID = agentID_;
  for(unsigned int i = 0; i < observed_state_.new_obstacles.size(); i++){
    comm_msg.obstacles_x.push_back(observed_state_.new_obstacles[i][0]);
    comm_msg.obstacles_y.push_back(observed_state_.new_obstacles[i][1]);
  }
  comm_msg.x = belief_state_.poses[agentID_].x;
  comm_msg.y = belief_state_.poses[agentID_].y;
  comm_msg.z = belief_state_.poses[agentID_].z;
  comm_msg.theta = belief_state_.poses[agentID_].theta;
  // send goals that we know this agent has visited
  for(int goal_i = 0; goal_i < numgoals_; goal_i++){
    if(belief_state_.goalsVisited[goal_i] == agentID_)
      comm_msg.goalsVisited.push_back(goal_i);
  }
  printf("Agent %d sending message number %d=%d \n", agentID_, 
	 observed_state_.lastpacketID_V[agentID_],
	 comm_msg.header.seq);
  comm_pub_.publish(comm_msg);
  
  // clear new obtacles
  observed_state_.new_obstacles.clear();
  
  // wait for replies
  //waitforReplies();
}

void EGraphXYNode::receiveCommunication(const mas_egraphs::MasComm::ConstPtr& msg){
  if(msg->agentID == agentID_)
    return;
  printf("Agent %d received message #%d from Agent %d\n", agentID_, msg->header.seq, msg->agentID);
  // update robot pose
  belief_state_.poses[msg->agentID].x = msg->x;
  belief_state_.poses[msg->agentID].y = msg->y;
  belief_state_.poses[msg->agentID].z = msg->z;
  belief_state_.poses[msg->agentID].theta = msg->theta;
  
  // update costs
  for(unsigned int obstacle_i = 0; obstacle_i < msg->obstacles_x.size(); obstacle_i++){
    updateCosts(msg->obstacles_x[obstacle_i], msg->obstacles_y[obstacle_i]);
  }

  // update goalsVisited. Remember msg->goalsVisited contains indices of goals visited by agent msg->agentID   
  for(int goal_i = 0; goal_i < (int) msg->goalsVisited.size(); goal_i++){
    observed_state_.goalsVisited[msg->goalsVisited[goal_i]] = msg->agentID; 
    belief_state_.goalsVisited[msg->goalsVisited[goal_i]] = msg->agentID;
  }

  // register this packetID. 
  observed_state_.lastpacketID_V[msg->agentID] = msg->header.seq;

  replan_required_ = true;
}

void EGraphXYNode::waitforReplies() const{
  // do nothing until we have received at least as many packets as we have sent
  while((observed_state_.lastpacketID_V[agentID_] > 
	    *std::min_element(observed_state_.lastpacketID_V.begin(), 
			      observed_state_.lastpacketID_V.end()))){
    printf("\nAgent %d waiting for replies: ", agentID_);
    egraph_mgr_->printVector(observed_state_.lastpacketID_V);
    // if this agent is done, then don't wait!
    if((*std::min_element(observed_state_.goalsVisited.begin(),
			  observed_state_.goalsVisited.end())) > -1)
      break;
    ros::Duration(1).sleep();
  }
}

/*void EGraphXYNode::publishfootprints(std::vector<pose_cont_t> poses) const{
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
}*/

int main(int argc, char** argv){
  ros::init(argc, argv, "mas_egraphs_node");

  tf::TransformListener tf_;
  costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  costmap->pause();

  EGraphXYNode xy(costmap);

  //ros::spin();
  ros::MultiThreadedSpinner spinner(4);//need 2 threads to catch the interrupt
  spinner.spin();
}

