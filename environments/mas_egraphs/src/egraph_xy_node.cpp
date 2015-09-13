#include <mas_egraphs/egraph_xy_node.h>

using namespace std;

EGraphXYNode::EGraphXYNode(costmap_2d::Costmap2DROS* costmap_ros) {
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    costfunc_ = mas_config::SUM;
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
    initializeNode();
   
    // create subscribers and publishers
    interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1, &EGraphXYNode::interruptPlannerCallback,this);
    comm_sub_ = nh.subscribe("/mas_egraphs/mas_comm", 1, &EGraphXYNode::receiveCommunication, this);
    makeplan_sub_ = nh.subscribe("/mas_egraphs/mas_plan_req", 1, &EGraphXYNode::startMASPlanner, this);

    plan_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mas_egraphs/mas_plan", 1);
    comm_pub_ = nh.advertise<mas_egraphs::MasComm>("/mas_egraphs/mas_comm", 1);
    sensor_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mas_egraphs/sensor", 1);
    stats_pub_ = nh.advertise<mas_egraphs::MasStats>("/mas_egraphs/mas_stats", 1);
    viz_.last_plan_markerID_ = 0;
    //footprint_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("footprint", 10);

    ros::service::waitForService("mas_egraphs/sensor",10);
    sensorupdate_client_ = ros::NodeHandle().serviceClient<mas_egraphs::GetSensorUpdate>("/mas_egraphs/sensor", true);

}

void EGraphXYNode::initializeNode(){
    start_time_ = ros::Time::now();
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
        ret = env_->InitializeEnv(agentID_, costmap_ros_->getSizeInCellsX(), // width
                                  costmap_ros_->getSizeInCellsY(), // height
                                  0, // mapdata
                                  numagents_, // numAgents
                                  0.5, 0.5, 0, //goal tolerance of 50 cm
                                  perimeterptsV,
                                  costmap_ros_->getResolution(), time_per_action_,
                                  primitive_filenames_,
                                  cost_map_.getOriginX(), cost_map_.getOriginY(),
                                  costfunc_);
        
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
    egraphs_.clear();
    egraphs_.reserve(numagents_);
    for(int agent_i = 0; agent_i < numagents_; agent_i++)
        egraphs_.push_back(new EGraph(env_, 4, 0));

    printf("Initialze node done\n");
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

bool EGraphXYNode::makePlan(EGraphReplanParams& params, std::vector<int>& solution_stateIDs,
                            int& solution_cost_i, double& plan_time){
    EGraphReplanParams params_copy(params);
    int startstate_id;
    switch (replan_condition_)
        {
        case NOTREQ:
            return true;
            break;
        case GLOBAL:
            //printf("\n\nAgent %d starting plan with condition = GLOBAL\n", agentID_);             
            egraph_mgr_->clearEGraphs();
            params_copy.feedback_path = true;
            params_copy.use_egraph = false;
            params_copy.return_first_solution = true;
            params_copy.epsE = 1;
            params_copy.final_epsE = 1;           
            startstate_id = env_->SetStart(observed_state_.poses, observed_state_.goalsVisited);
            break;
        case LOCAL:
            //printf("\n\nAgent %d starting plan with condition = LOCAL\n", agentID_);   
            params_copy.return_first_solution = true;
            params_copy.feedback_path = false; // local plans do not update the experience
            params_copy.use_egraph = true;
            startstate_id = env_->SetStart(belief_state_.poses, belief_state_.goalsVisited);
            break;
        }

    if(startstate_id < 0 || planner_->set_start(startstate_id) == 0){
        ROS_ERROR("ERROR: failed to set start state\n");
        return false;
    }
    //env_->PrintState(startstate_id, true);
    //params_copy.print();
    //printf("Hit any key to start planning\n");
    egraph_mgr_->updateHeuristicGrids(heur_grid_);
  
    // plan!
    solution_stateIDs.clear();

    double plan_time_start = ros::Time::now().toSec();
    bool ret = planner_->replan(&solution_stateIDs, params_copy);
    double plan_time_end = ros::Time::now().toSec();
    plan_time = plan_time_end - plan_time_start;
    //env_->PrintTimingStats();
    //egraph_mgr_->printTimingStats();
    std::vector<int> pathcost_per_agent;
    planner_->get_pathcost_per_agent(solution_stateIDs, pathcost_per_agent); 
    solution_cost_i = pathcost_per_agent[agentID_];

    if(ret)
        visualizePath(solution_stateIDs);        
  
    return ret;
}

void EGraphXYNode::startMASPlanner(const mas_egraphs::GetXYThetaPlan::ConstPtr& msg){
    printf("\n.....................\n Agent %d Restarting startMASPlanner\n....................\n", agentID_);
    initializeNode();
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
    heurs_.clear();
    heurs_.resize(numagents_);
    for(int agent_i=0; agent_i < numagents_; agent_i++){
        heurs_[agent_i].clear();
        heurs_[agent_i].resize(msg->num_goals);
        for(int goal_i=0; goal_i < msg->num_goals; goal_i ++){
            heurs_[agent_i][goal_i] = new EGraphMAS2dGridHeuristic(*env_, 
                                                                   costmap_ros_->getSizeInCellsX(), 
                                                                   costmap_ros_->getSizeInCellsY(),
                                                                   1);
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
        marker.scale.x = cost_map_.getInflationRadius();
        marker.scale.y = cost_map_.getInflationRadius();
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
    plan_pub_.publish(goals);
 
    // publish start
    if (((int)msg->start_x.size() != numagents_) || ((int)msg->start_y.size() != numagents_)){
        SBPL_ERROR("Incorrect number of start locations");
        return;// false;
    }

    // initialize belief state
    belief_state_.poses.clear();
    belief_state_.poses.resize(numagents_);
    for(int agent_i=0; agent_i < numagents_; agent_i++){
        belief_state_.poses[agent_i].x = msg->start_x[agent_i] - cost_map_.getOriginX();
        belief_state_.poses[agent_i].y = msg->start_y[agent_i] - cost_map_.getOriginY();
        belief_state_.poses[agent_i].z = msg->start_z[agent_i];
        belief_state_.poses[agent_i].theta = msg->start_theta[agent_i];
    }
    visualizePoses();
    belief_state_.goalsVisited.clear();
    belief_state_.goalsVisited.resize(numgoals_, -1);
   
    // initialize observed state
    observed_state_.lastpacketID_V.clear();
    //printf("Agent %d : All %d packetIDs have been reset", agentID_, numagents_);
    observed_state_.lastpacketID_V.resize(numagents_, -1);
    observed_state_.new_obstacles.clear();
    observed_state_.goalsVisited.clear();
    observed_state_.goalsVisited.resize(numgoals_, -1);
    observed_state_.assignments.resize(numgoals_, -1);
    observed_state_.poses.clear();
    observed_state_.poses.resize(numagents_);
    for(int agent_i=0; agent_i < numagents_; agent_i++){
        observed_state_.poses[agent_i] = belief_state_.poses[agent_i];
    }
    egraph_mgr_ = new EGraphManager<std::vector<int> > (egraphs_, env_, heurs_,
                                                        msg->num_goals, numagents_,
                                                        costfunc_);
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
    params.initial_eps = 2;
    params.dec_eps = 0;
    params.final_eps = 2;
    params.epsE = msg->eps_comm;
    params.final_epsE = msg->eps_comm;
    params.return_first_solution = true;
    replan_condition_ = GLOBAL;

    std::vector<double> plan_times_s;
    double t_start = ros::Time::now().toSec();
    bool ret = agentManager(params, plan_times_s);      
    double t_end = ros::Time::now().toSec();

    // construct stats message
    printf("Agent %d is done. Publishing stats\n", agentID_);
    mas_egraphs::MasStats mas_stats_msg;
    mas_stats_msg.agentID = agentID_;
    mas_stats_msg.success = ret;
    mas_stats_msg.total_time_s = t_end - t_start;
    mas_stats_msg.num_packets_peragent = observed_state_.lastpacketID_V[agentID_]+1;
    mas_stats_msg.plan_times_s.clear();
    for(unsigned int i = 0; i < plan_times_s.size(); i++){
        mas_stats_msg.plan_times_s.push_back(plan_times_s[i]);
    }   
    stats_pub_.publish(mas_stats_msg);
}

bool EGraphXYNode::execute(const std::vector<int>& solution_stateIDs_V, int& cost_traversed_i){
    cost_traversed_i = 0;
    int time_per_action = env_->GetPerActionCost();
    int active_agent_offset = 4*numagents_ + numgoals_ + agentID_;
    for(unsigned int step = 0; step < solution_stateIDs_V.size(); step++){
        //printf("Agent %d: Executing step %d\n------------------\n", 
        //agentID_, (int)step);
        std::vector<double> coord;
        env_->getCoord(solution_stateIDs_V[step], coord);
    
        cost_traversed_i += time_per_action;
        for(int agent_i = 0; agent_i < numagents_; agent_i++){
            belief_state_.poses[agent_i].x = coord[4*agent_i];
            belief_state_.poses[agent_i].y = coord[4*agent_i+1];
            belief_state_.poses[agent_i].z = coord[4*agent_i+2];
            belief_state_.poses[agent_i].theta = coord[4*agent_i+3];
        }

        for(int goal_i = 0; goal_i < numgoals_; goal_i++){
            belief_state_.goalsVisited[goal_i] = coord[4*numagents_+goal_i];
        }
    
#ifdef SIM
        getSensorData(coord);
#endif

        visualizePoses();
        // if old plan is invalid, we want to replan
        if(!env_->IsValidPlan(solution_stateIDs_V, step)){
            printf("Agent %d thinks plan is invalid\n", agentID_);
            replan_condition_ = LOCAL;
        }

        if(replan_condition_ != NOTREQ){
            //printf("Agent %d: replan condition is %d,  replan required\n", agentID_, replan_condition_);
            return false;
        }

        // agent has retired
        if(!coord[active_agent_offset])
            break;

        //printf("\n-------------------------\n");
        ros::Duration(0.25).sleep();    
    }
    // if agent is done with its task, communicate and return
    sendCommunication();
    return true;
}

// TODO: this should really be a Finite State Machine. Currently super hacky.
bool EGraphXYNode::agentManager(EGraphReplanParams& params, std::vector<double> &plan_times_s){
    plan_times_s.clear();
    std::vector<int> solution_stateIDs;
    int cost_plan_local_i; // cost of new plan for this agent
    int cost_plan_global_i; // cost of last communicated plan for this agent
    int cost_traversed_i; // cost of path traversed so far for this agent
    double plan_time_s;
    while(true){    
        visualizeCommPackets();   
        bool planExists = makePlan(params, solution_stateIDs, cost_plan_local_i, plan_time_s);   
        plan_times_s.push_back(plan_time_s);    
        if (replan_condition_ == GLOBAL){
            cost_plan_global_i = cost_plan_local_i;
            cost_traversed_i = 0;
        }
        //printf("cost_local %d cost_global %d cost_traversed %d epsComm %f eps %f\n", cost_plan_local_i, cost_plan_global_i, cost_traversed_i,
        //       params.epsE, params.final_eps);
        //printf("Hit any key to execute");
        //std::cin.get();
        // if the plan doesn't exist, return false
        if(!planExists)
            return false;    
        if(solution_stateIDs.empty())
            return true;
        std::vector<int> new_assignments;
        env_->getAssignments(solution_stateIDs.back(), new_assignments);

        if(replan_condition_ == LOCAL){
            // Initiate communication if goal assignments have switched
            if(observed_state_.assignments != new_assignments){
                //printf("Agent %d initiating communication because assignments switched\n", agentID_);
                //ros::Duration(0.1).sleep();
                sendCommunication();
            }
            // Initiate communication if deviation is too large
            if(cost_plan_local_i >= 
               params.epsE*(cost_plan_global_i - params.final_eps*cost_traversed_i)){
                //printf("LHS %f RHS %f\n", (float) cost_plan_local_i, params.epsE*((cost_plan_global_i - params.final_eps*cost_traversed_i)));
                //printf("Agent %d initiating communication because deviation is too large\n", agentID_);                
                sendCommunication();
            }
        }
        replan_condition_ = NOTREQ;

        observed_state_.assignments = new_assignments;

        // If this agent has something to do, execute
        bool isComplete;
        if(env_->isActive(solution_stateIDs[1], agentID_))
            isComplete = execute(solution_stateIDs, cost_traversed_i);
    
        // sit around and wait for a communication, if we have retired
        while(replan_condition_ == NOTREQ){     
            ros::Duration(0.1).sleep();     
        }

        // Initiate communication because we have not replied to a message sent by another agent
        int most_recent_packetID = *std::max_element(observed_state_.lastpacketID_V.begin(), 
                                                     observed_state_.lastpacketID_V.end());      

        if(observed_state_.lastpacketID_V[agentID_] < most_recent_packetID){
            sendCommunication();
        }      
   
        // if we "know" all goals have been visited, we are done
        if (std::all_of(observed_state_.goalsVisited.begin(),
                        observed_state_.goalsVisited.end(), 
                        [](int i){return i >= 0;}))
            break;

        ros::Duration(0.1).sleep();    
    }  
    //printf("Agent %d is done\n", agentID_);
    return true;
}


void EGraphXYNode::updatelocalMap(sensor_msgs::PointCloud& pointcloud){
    for(unsigned int i = 0; i < pointcloud.points.size(); i++){
        int x = (int) pointcloud.points[i].x;
        int y = (int) pointcloud.points[i].y;
        unsigned char c = costMapCostToSBPLCost(pointcloud.channels[0].values[i]); 
        if(c >= inscribed_inflated_obstacle_){     
            if (!heur_grid_[x][y]){ // new obstacle
                //printf("new obstacle at (%d, %d) = %d ", x, y, (int) c);
                std::vector<int> obstacle(2);
                obstacle[0] = x;
                obstacle[1] = y;
                observed_state_.new_obstacles.push_back(obstacle);       
                updateCosts(x, y, c);	
            }
        }

    }
}


void EGraphXYNode::updateCosts(int x, int y){
    updateCosts(x, y, inscribed_inflated_obstacle_);
}

void EGraphXYNode::updateCosts(int x, int y, unsigned char c){
    env_->UpdateCost(x,y,c);
    if(c >= inscribed_inflated_obstacle_){
        //printf("Agent %d updating cost of (%d, %d) with %d \n", agentID_, x,y, (int) c);
        heur_grid_[x][y] = true; 
    }
}

void EGraphXYNode::sendCommunication(){
    //if(numagents_ == 1)
    //    return;
    mas_egraphs::MasComm comm_msg;
    // increment packetID and send
    observed_state_.lastpacketID_V[agentID_]++;
    //printf("Agent %d: sending comm msg %d\n", agentID_, observed_state_.lastpacketID_V[agentID_]);
    comm_msg.packetID = observed_state_.lastpacketID_V[agentID_];
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
    comm_pub_.publish(comm_msg);
    // clear new obtacles
    observed_state_.new_obstacles.clear();  

    visualizeCommPackets();

    // wait for replies
    waitforReplies();
}

void EGraphXYNode::waitforReplies() const{
    // do nothing until we have received as many messages as we have sent
    while((observed_state_.lastpacketID_V[agentID_] > 
           *std::min_element(observed_state_.lastpacketID_V.begin(), 
                             observed_state_.lastpacketID_V.end()))){
        ros::Duration(0.5).sleep();
    }
    //egraph_mgr_->printVector(observed_state_.lastpacketID_V);
}


void EGraphXYNode::receiveCommunication(const mas_egraphs::MasComm::ConstPtr& msg){
    //printf("Agent %d received message #%d with pose (%f, %f)  from Agent %d\n", agentID_, msg->header.seq, msg->x, msg->y, msg->agentID);
    // TODO: Add locking: not thread safe
    
    // make sure we are not processing something left over int he buffer
    if(msg->header.stamp < start_time_)
        {
            printf("Discarding leftover message");
            return;
        }
    // update robot pose
    observed_state_.poses[msg->agentID].x = msg->x;
    observed_state_.poses[msg->agentID].y = msg->y;
    observed_state_.poses[msg->agentID].z = msg->z;
    observed_state_.poses[msg->agentID].theta = msg->theta;
  
    // update costs
    for(unsigned int obstacle_i = 0; obstacle_i < msg->obstacles_x.size(); obstacle_i++){
        updateCosts(msg->obstacles_x[obstacle_i], msg->obstacles_y[obstacle_i]);
    }
  
    // update goalsVisited. Remember msg->goalsVisited contains indices of goals visited by agent msg->agentID   
    for(int goal_i = 0; goal_i < (int) msg->goalsVisited.size(); goal_i++){
        observed_state_.goalsVisited[msg->goalsVisited[goal_i]] = msg->agentID; 
    }
  
    // register this packetID. 
    //printf("Agent %d: received packet %d sent at time %f. Start time is %f \n",
    //       agentID_, msg->packetID, 
    //       msg->header.stamp.toSec(), start_time_.toSec());
    observed_state_.lastpacketID_V[msg->agentID] = msg->packetID;
    replan_condition_ = GLOBAL;
}

void EGraphXYNode::getSensorData(const std::vector<double>& coord){
    // localize current agent
    belief_state_.poses[agentID_].x = coord[4*agentID_];
    belief_state_.poses[agentID_].y = coord[4*agentID_+1];
    belief_state_.poses[agentID_].z = coord[4*agentID_+2];
    belief_state_.poses[agentID_].theta = coord[4*agentID_+3];

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
    //printf("Got sensor info for %d points \n", res.pointcloud.points.size());
    // update costs according to new sensor information
    //visualizeSensor(res.pointcloud);
    updatelocalMap(res.pointcloud);    
}

void EGraphXYNode::visualizeSensor(const sensor_msgs::PointCloud& pointcloud) const{
    visualization_msgs::MarkerArray obstacles;
    for(int i = 0; i < (int) pointcloud.points.size(); i++){
        if(heur_grid_[pointcloud.points[i].x][pointcloud.points[i].y])
            return;      
        pose_cont_t pose;
        ros::Time time = ros::Time::now();  
        env_->PoseDiscToCont(pointcloud.points[i].x, pointcloud.points[i].y, pointcloud.points[i].z,
                             0, pose.x, pose.y, pose.z, pose.theta);    
        visualization_msgs::Marker marker;
        marker.ns = std::to_string(agentID_);
        marker.id = i;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0;
        marker.color.b = agentID_%2;
        marker.color.r = (agentID_+1)%2;
        marker.color.a = 1;
        contPosetoGUIPose(pose, marker);
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.header.stamp = time;
        marker.header.frame_id = costmap_ros_->getGlobalFrameID();

        obstacles.markers.push_back(marker);
    }
    sensor_pub_.publish(obstacles);
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
        marker.scale.x = cost_map_.getInflationRadius();
        marker.scale.y = cost_map_.getInflationRadius();
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

int main(int argc, char** argv){
    ros::init(argc, argv, "mas_egraphs_node");

    tf::TransformListener tf_;
    costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    costmap->pause();

    EGraphXYNode xy(costmap);
    // 1 thread to plan, execute and send communication; 1 thread to receive communication; 1 thread
    // to catch the interrupts
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
}

