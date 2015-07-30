#include<mas_egraphs/egraph_xy.h>
#include<mas_egraphs/environment_xy.h>

using namespace std;

EGraphXY::EGraphXY(){
}


bool EGraphXY::InitializeEnv(int agentID, int width, int height,
			     const unsigned char* mapdata,
			     int numagents, 
			     double goaltol_x, double goaltol_y, double goaltol_theta,
			     const std::vector<std::vector<sbpl_2Dpt_t> > & perimeterptsV,
			     double cellsize_m, double time_per_action,
			     const std::vector<std::string> sMotPrimFiles,
			     double costmapOriginX, double costmapOriginY){
  bool ret = Environment_xy::InitializeEnv(agentID, width, height, mapdata, numagents, 
					   //numgoals,  start, goal,
					   goaltol_x, goaltol_y, goaltol_theta, perimeterptsV,
					   cellsize_m, time_per_action, sMotPrimFiles,
					   costmapOriginX, costmapOriginY);

  return ret;
  
}

bool EGraphXY::snap(const vector<double>& from, const vector<double>& to, int& id, int& cost){
  return false;
}

//requires a getCoord function which takes a state id (the ids the environment uses) and returns a vector with the coordinate so we can look for shortcuts in the e-graph
//-we will never call getCoord on the goal (because it is possible we don't know what the goal state looks like)

// Coord now looks like [agent1.x agent1.y agent1.z agent1.theta ........ goal1 goal2.. isactive1 isactive2....]
bool EGraphXY::getCoord(int id, vector<double>& coord) const {
  coord.clear();
  EnvXYHashEntry_t* hashEntry = StateID2CoordTable[id];
  vector<int> d_coord_agent(4,0);
  vector<double> coord_agent(4,0);
  coord.resize(EnvXYCfg.numAgents*4 + EnvXYCfg.numGoals + EnvXYCfg.numAgents);
  int i = 0;
  for(int agent_i=0; agent_i < EnvXYCfg.numAgents; agent_i++, i+=4){
    d_coord_agent[0] = hashEntry->poses[agent_i].x;
    d_coord_agent[1] = hashEntry->poses[agent_i].y;
    d_coord_agent[2] = hashEntry->poses[agent_i].z;
    d_coord_agent[3] = hashEntry->poses[agent_i].theta;
    discToCont(d_coord_agent, coord_agent);
    coord[i] = coord_agent[0];
    coord[i+1] = coord_agent[1];
    coord[i+2] = coord_agent[2];
    coord[i+3] = coord_agent[3];
  }

  for(int goal_i = 0; goal_i < EnvXYCfg.numGoals; goal_i++, i++){
    coord[i] = hashEntry->goalsVisited[goal_i];
  }
  
  for(int agent_i=0; agent_i < EnvXYCfg.numAgents; agent_i++, i++){
    coord[i] = hashEntry->activeAgents[agent_i];
  }
  return true;
}

bool EGraphXY::isActive(const int stateID, const int agentID) const{
  std::vector<double> coord;
  getCoord(stateID, coord);
  if(coord[4*EnvXYCfg.numAgents + EnvXYCfg.numGoals + agentID])
    return true;
  else
    return false;
}


int EGraphXY::getStateID(const vector<double>& coord){
  return 0;
  vector<int> d_coord;
  contToDisc(coord,d_coord);
  std::vector<pose_disc_t> poses;
  std::vector<int> goalsVisited;
  std::vector<bool> activeAgents;
  int i =0;
  int ctr = 0;
  for(; i < EnvXYCfg.numAgents; i+=4){
      pose_disc_t pose;
      pose.x = d_coord[i];
      pose.y = d_coord[i+1];
      pose.z = d_coord[i+2];
      pose.theta = d_coord[i+3];
      poses[ctr] = pose;
    }

  for(; i < EnvXYCfg.numGoals; i++){
      goalsVisited[ctr] = coord[i];
      ctr++;
    }

  for(; i < EnvXYCfg.numAgents; i++){
    activeAgents[ctr] = coord[i];
    ctr++;
  }

  return GetStateFromCoord(poses, goalsVisited, activeAgents); 
}


bool EGraphXY::isGoal(int id){
  return Environment_xy::isGoal(id);
  }

void EGraphXY::projectToHeuristicSpace(const vector<double>& coord, vector<int>& dp) const{
  dp.clear();
  for(int i = 0; i < (int) coord.size(); i+=4){
    //SBPL_INFO("projecting (%f, %f)", coord[i], coord[i+1]);
    int x = CONTXY2DISC(coord[i], EnvXYCfg.cellsize_m);
    int y = CONTXY2DISC(coord[i+1], EnvXYCfg.cellsize_m);
    dp.push_back(x);
    dp.push_back(y);
  }
  /*
  for(int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++, i+=2)
    {
      int x = CONTXY2DISC(coord[i], EnvXYCfg.cellsize_m);
      int y = CONTXY2DISC(coord[i+1], EnvXYCfg.cellsize_m);
      dp.push_back(x);
      dp.push_back(y);
    }
  
  for(; i < EnvXYCfg.numGoals; i++)
    dp.push_back(coord[i]);
  */
}

void EGraphXY::projectGoalToHeuristicSpace(vector<int>& dp) const{
  // dp = [x1, y1, x2, y2 ... x_numgoals y_numgoals, 1 1 1 1... numgoals]
  for(int i = 0; i < EnvXYCfg.numGoals; i ++){
    dp.push_back(EnvXYCfg.goal[i].x);
    dp.push_back(EnvXYCfg.goal[i].y);
  }
  for(int i = 0; i < EnvXYCfg.numGoals; i++)
    dp.push_back(1);
}

void EGraphXY::contToDisc(const vector<double>& c, vector<int>& d) const{
  d.resize(c.size());
  int i;
  //SBPL_INFO("in conttodisc with (%f %f)", c[0],c[1]) ;
  for(i = 0; i < (int) c.size(); i +=4){
    PoseContToDisc(c[i], c[i+1], c[i+2], c[i+3], d[i], d[i+1], d[i+2], d[i+3]);
  }
}

void EGraphXY::discToCont(const vector<int>& d, vector<double>& c) const{
  c.resize(d.size());
  int i;
  //SBPL_INFO("in disctocont with (%d %d)", d[0],d[1]) ;
  for(i =0; i < (int) d.size(); i+=4){
    PoseDiscToCont(d[i], d[i+1], d[i+2], d[i+3], c[i], c[i+1], c[i+2], c[i+3]);
  }
}

bool EGraphXY::isValidEdge(const vector<double>& coord, const vector<double>& coord2,
			   bool& change_cost, int& cost){
  return isValidEdge(0, coord, coord2, change_cost, cost);
}

bool EGraphXY::isValidEdge(int agentID, const vector<double>& coord, const vector<double>& coord2,
			   bool& change_cost, int& cost){
  /*
   we assume that an edge cost will change only if either vertex becomes invalid. 
   TODO:
   We cannot directly invoke getsuccs because our egraphs are formed by splitting up a plan into its multiple agents. 
   Which means we cannot recompute the ID given coord for just one agent. We should keep track of ids while creating the egraph,
   and then use GetSuccsForAgent to recompute the cost.
  */
  return (isValidVertex(agentID, coord) && isValidVertex(agentID, coord2));    
}
			     
bool EGraphXY::isValidVertex(int agentID, const vector<double>& coord){
  vector<int> d_coord;
  contToDisc(coord, d_coord);
  pose_disc_t d_pose;
  d_pose.x = d_coord[0];
  d_pose.y = d_coord[1];
  d_pose.z = d_coord[2];
  d_pose.theta = d_coord[3];
  return IsValidPose(agentID, d_pose);
}

bool EGraphXY::isValidVertex(const vector<double>& coord){
  return isValidVertex(0, coord);
}

void EGraphXY::getAssignments(int solution_stateID, std::vector<int>& assignments) const{
  std::vector<pose_disc_t> poses;
  std::vector<bool> activeAgents;
  GetCoordFromState(solution_stateID, poses, assignments, activeAgents);
}

