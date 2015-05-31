#include <mas_egraphs/egraph_xy_node.h>
#include <nav_msgs/Path.h>
using namespace std;

EGraphXYNode::EGraphXYNode(costmap_2d::Costmap2DROS* costmap_ros) {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  
  private_nh.param("primitive_filename",primitive_filename_,string(""));
  double nominalvel_mpersecs, timetoturn45degsinplace_secs;
  private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 1.0);
  private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

  int lethal_obstacle;
  private_nh.param("lethal_obstacle",lethal_obstacle, 20);
  lethal_obstacle_ = (unsigned char) lethal_obstacle;
  inscribed_inflated_obstacle_ = lethal_obstacle_-1;
  sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
  
  costmap_ros_ = costmap_ros;
  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cost_map_);
  env_ = new EGraphXY();

  /*if(!env_->SetEnvParameter("cost_inscribed_thresh", costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
    ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
    exit(1);
  }
  if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(cost_map_.getCircumscribedCost()))){
    ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
    exit(1);
    }
  int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
  */
  bool ret;
  try{
    std::vector<pose_t> start(1);
    start[0].x = 0;
    start[0].y = 0;

    std::vector<pose_t> goal(1);
    goal[0].x = 0;
    goal[0].y = 0;
    
    ret = env_->InitializeEnv(costmap_ros_->getSizeInCellsX(), // width
			      costmap_ros_->getSizeInCellsY(), // height
			      0, // mapdata
			      1, // numAgents
			      start, // start vector of poses (x, y)
			      goal, // goal vector of poses (x, y)
			      0, 0, //goal tolerance
			      costmap_ros_->getResolution(), nominalvel_mpersecs);
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

  //egraph_vis_ = new EGraphVisualizer(egraph_, env_);
  // egraph_vis_->visualize();
   
  interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1, &EGraphXYNode::interruptPlannerCallback,this);
  plan_pub_ = nh.advertise<visualization_msgs::MarkerArray>("plan", 1);
  plan_service_ = nh.advertiseService("/sbpl_planning/plan_path",&EGraphXYNode::makePlan,this);
 
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

bool EGraphXYNode::makePlan(mas_egraphs::GetXYThetaPlan::Request& req, mas_egraphs::GetXYThetaPlan::Response& res){
  ROS_DEBUG("[sbpl_lattice_planner] getting fresh copy of costmap");
  costmap_ros_->clearRobotFootprint();
  ROS_DEBUG("[sbpl_lattice_planner] robot footprint cleared");

  costmap_ros_->getCostmapCopy(cost_map_);
  SBPL_INFO("Received request with numAgents = %d",req.num_agents);
  SBPL_ERROR("Checking error message"); 
  
  try{
    bool ret = env_->SetNumAgents(req.num_agents);
    if (!ret)
      {
	SBPL_PRINTF("Invalid number of agents");
	return false;
      }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the number of agents");
    return false;
  }
  numagents_ = req.num_agents;
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

  heurs_.resize(req.num_agents);
  for(int agent_i=0; agent_i < req.num_agents; agent_i++){
    heurs_[agent_i].resize(req.num_goals);
    for(int goal_i=0; goal_i < req.num_goals; goal_i ++){
      heurs_[agent_i][goal_i] = new EGraphMAS2dGridHeuristic(*env_, costmap_ros_->getSizeInCellsX(), costmap_ros_->getSizeInCellsY(), 1);
    }
  }

  string egraph_filename;
  /*
  private_nh.param<string>("egraph_filename", egraph_filename, "");
  if(egraph_filename.empty())
    egraph_ = new EGraph(env_, 2, 0);
  else
    egraph_ = new EGraph(env_,egraph_filename);
  */
  egraphs_.resize(req.num_agents);
  for(int agent_i = 0; agent_i < req.num_agents; agent_i++)
    egraphs_[agent_i] = new EGraph(env_, 2, 0);
  
  egraph_mgr_ = new EGraphManager<vector<int> > (egraphs_, env_, heurs_, req.num_goals, req.num_agents); //TODO
  planner_ = new LazyAEGPlanner<vector<int> >(env_, true, egraph_mgr_);

  try{
    std::vector<sbpl_xy_theta_pt_t> goal_shifted(req.num_goals);
    for(int goal_i = 0; goal_i < req.num_goals; goal_i++)
      {
	goal_shifted[goal_i].x = req.goal_x[goal_i] - cost_map_.getOriginX();
        goal_shifted[goal_i].y = req.goal_y[goal_i] - cost_map_.getOriginY();
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
  

  std::vector<sbpl_xy_theta_pt_t> start_shifted(req.num_agents);
  int startstateID;
  try{
    for(int agent_i = 0; agent_i < req.num_agents; agent_i++)
      {
	start_shifted[agent_i].x = req.start_x[agent_i] - cost_map_.getOriginX();
	start_shifted[agent_i].y = req.start_y[agent_i] - cost_map_.getOriginY();
	start_shifted[agent_i].theta = req.start_theta[agent_i];
      }
    int ret = env_->SetStart(start_shifted);
    if(ret < 0 || planner_->set_start(ret) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
    startstateID = ret;
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }


  //publish goals
  ros::Time req_time = ros::Time::now();
  visualization_msgs::MarkerArray goals;
  int id = 0;
  
   for(int goal_i = 0; goal_i < req.num_goals; goal_i++){
    visualization_msgs::Marker marker;
	marker.id = id; 
	id++;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
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
   plan_pub_.publish(goals);
   
  EGraphReplanParams params(100.0);
  params.initial_eps = req.initial_eps;
  params.dec_eps = req.dec_eps;
  params.final_eps = req.final_eps;
  params.epsE = req.egraph_eps;
  params.dec_epsE = req.dec_egraph_eps;
  params.final_epsE = req.final_egraph_eps;
  params.return_first_solution = false;
  params.use_egraph = req.use_egraph;
  params.feedback_path = req.feedback_path;

  bool ret = simulate(req.start_x, req.start_y, params, res, 4);
  return ret;  
}


void EGraphXYNode::publishPath(std::vector<int>& solution_stateIDs, mas_egraphs::GetXYThetaPlan::Response& res){
  visualization_msgs::MarkerArray gui_path;
  vector<double> coord;
  res.path.clear();
  int id = numgoals_ + numagents_;
  ros::Time plan_time = ros::Time::now();  
 
  for(int agent_i = 0; agent_i < numagents_; agent_i++)
    {
      for(unsigned int i=0; i < solution_stateIDs.size(); i++){    
	coord.clear();
       	env_->getCoord(solution_stateIDs[i], coord);
	
	//env_->GetFakePath(i, coord); 
	//SBPL_INFO("id %d (x,y) = (%f,%f)", solution_stateIDs[i], coord[0], coord[1]);

	visualization_msgs::Marker marker;
	marker.id = id; 
	id++;
	marker.scale.x = 0.02;
	marker.scale.y = 0.02;
	marker.scale.z = 0;
	marker.color.b = 1;
	marker.color.a = 1;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.header.stamp = plan_time;
	marker.header.frame_id = costmap_ros_->getGlobalFrameID();
	marker.pose.position.x = coord[2*agent_i] + cost_map_.getOriginX();
	marker.pose.position.y = coord[2*agent_i+1] + cost_map_.getOriginY();
	marker.pose.position.z = 0;

	tf::Quaternion temp;
	temp.setEulerZYX(0,0,0);
	marker.pose.orientation.x = temp.getX();
	marker.pose.orientation.y = temp.getY();
	marker.pose.orientation.z = temp.getZ();
	marker.pose.orientation.w = temp.getW();
	gui_path.markers.push_back(marker);

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = marker.header.stamp;
	pose.header.frame_id = marker.header.frame_id;
	pose.pose.orientation.x = marker.pose.orientation.x; 
	pose.pose.orientation.y = marker.pose.orientation.y;
	pose.pose.orientation.z = marker.pose.orientation.z;
	pose.pose.orientation.w = marker.pose.orientation.w;
	pose.pose.position.x = marker.pose.position.x;
	pose.pose.position.y = marker.pose.position.y;
	pose.pose.position.z = marker.pose.position.z;
	res.path.push_back(pose);
      }
    }
  plan_pub_.publish(gui_path);
}

bool EGraphXYNode::simulate(std::vector<double> start_x, std::vector<double> start_y, EGraphReplanParams params, mas_egraphs::GetXYThetaPlan::Response& res, int maxtime){
  std::vector<int> solution_stateIDs;
  // right now, robots follow original plan at different rates
  std::vector<std::vector<double> > r1(maxtime);
  std::vector<std::vector<double> > r2(maxtime);
  std::vector<sbpl_xy_theta_pt_t> start_shifted(numagents_);
  std::vector<double> coord;
  std::vector<int> assignments;
  float r1percents[] = {0.2, 0.4, 0.8, 1};
  float r2percents[] = {0, 0.1, 0.1, 0.1};
  // set start state
  start_shifted[0].x = start_x[0] - cost_map_.getOriginX();
  start_shifted[0].y = start_y[0] - cost_map_.getOriginY();
  start_shifted[1].x = start_x[1] - cost_map_.getOriginX();
  start_shifted[1].y = start_y[1] - cost_map_.getOriginY();

int timestep = 0;
do{
  int retid = env_->SetStart(start_shifted);
  if(retid < 0 || planner_->set_start(retid) == 0){
    ROS_ERROR("ERROR: failed to set start state\n");
    return false;
  }
  
  // publish start
  int id = numgoals_;
  visualization_msgs::MarkerArray startmarkers;
  ros::Time req_time = ros::Time::now();
  for(int agent_i = 0; agent_i < numagents_; agent_i++){
    visualization_msgs::Marker marker;
    marker.id = id; 
    id++;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0;
    marker.color.r = 1;
    marker.color.a = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.header.stamp = req_time;
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    marker.pose.position.x = start_shifted[agent_i].x + cost_map_.getOriginX();
    marker.pose.position.y = start_shifted[agent_i].y + cost_map_.getOriginY();
    marker.pose.position.z = 0;
    startmarkers.markers.push_back(marker);
  }
  plan_pub_.publish(startmarkers);
  
  vector<vector<bool> > heur_grid(cost_map_.getSizeInCellsX(), vector<bool>(cost_map_.getSizeInCellsY(), false));
  for(unsigned int ix = 0; ix < cost_map_.getSizeInCellsX(); ix++){
    for(unsigned int iy = 0; iy < cost_map_.getSizeInCellsY(); iy++){
      unsigned char c = costMapCostToSBPLCost(cost_map_.getCost(ix,iy));
      env_->UpdateCost(ix, iy, c);
      if(c >= inscribed_inflated_obstacle_)
	heur_grid[ix][iy] = true;
    }
  }
  
  egraph_mgr_->updateManager(); // updates start and goal
  egraph_mgr_->updateHeuristicGrids(heur_grid);
  
  // plan!
  solution_stateIDs.clear();
  bool ret = planner_->replan(&solution_stateIDs, params);
  env_->getAssignments(solution_stateIDs, assignments);
  for(int i = 0; i < numgoals_; i++){
    SBPL_INFO("Goal %d assigned to %d", i, assignments[i]);
  }
  if (!ret)
    return false;
  publishPath(solution_stateIDs, res);    
  
  // extract locations from first plan to "simulate" robots
  if(timestep == 0){
    unsigned int planlength = solution_stateIDs.size();
    // r1,r2 at 20% of way
    for(int i = 0; i < maxtime; i ++){
      int r1id = (int) (r1percents[i] * (float) planlength);
      int r2id = (int) (r2percents[i] * (float) planlength);
      SBPL_INFO("r1id = %d, r2id = %d", r1id, r2id);
      env_->getCoord(solution_stateIDs[r1id], coord);
      /*std::vector<pose_t> poses;
      std::vector<bool> goalsVisited;
      env_->GetCoordFromState(solution_stateIDs[r1id], poses, goalsVisited)*/
      r1[i].push_back(coord[0]);
      r1[i].push_back(coord[1]);
      start_shifted[0].x = coord[0];
      start_shifted[0].y = coord[1];
      env_->getCoord(solution_stateIDs[r2id], coord);
      r2[i].push_back(coord[2]);
      r2[i].push_back(coord[3]);
    }
  }

  // reset start states
  start_shifted[0].x = r1[timestep][0] - cost_map_.getOriginX();
  start_shifted[0].y = r1[timestep][1] - cost_map_.getOriginY();
  start_shifted[1].x = r2[timestep][0] - cost_map_.getOriginX();
  start_shifted[1].y = r2[timestep][1] - cost_map_.getOriginY();
  
  SBPL_INFO("Hit any key to forward simulate");
  std::cin.get();
  timestep++;
 }while(timestep < maxtime);

 SBPL_INFO("Simulation done");
 return true;
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

