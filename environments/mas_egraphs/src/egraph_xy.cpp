#include<mas_egraphs/egraph_xy.h>
#include<mas_egraphs/environment_xy.h>

using namespace std;

EGraphXY::EGraphXY(){
}


bool EGraphXY::InitializeEnv(int width, int height,
				  const unsigned char* mapdata,
				  int numagents,
				  std::vector<pose_t> start,
				  std::vector<pose_t> goal,				  
				  double goaltol_x, double goaltol_y,
			          double cellsize_m, double nominalvel_mpersecs){

  bool ret = Environment_xy::InitializeEnv(width, height, mapdata, numagents, start,
			   goal, goaltol_x, goaltol_y,
			   cellsize_m, nominalvel_mpersecs);
  return ret;
  
}

int EGraphXY::GetNumGoals() const
{
  return EnvXYCfg.numGoals;
}

int EGraphXY::GetNumAgents() const
{
  return EnvXYCfg.numAgents;
}


bool EGraphXY::collisionCheckPose(int x, int y, int& cost){
  if(!IsValidCell(x, y))
    return false;

  if(EnvXYCfg.Grid2D[x][y] >= EnvXYCfg.obsthresh)
    return false;

  cost = EnvXYCfg.Grid2D[x][y]; 
  return true;
}


bool EGraphXY::snap(const vector<double>& from, const vector<double>& to, int& id, int& cost){
  return false;
}


//requires a getCoord function which takes a state id (the ids the environment uses) and returns a vector with the coordinate so we can look for shortcuts in the e-graph
//-we will never call getCoord on the goal (because it is possible we don't know what the goal state looks like)

// getCoord now looks like [agent1.x agent1.y agent1.theta ........ goal1 goal2....]
bool EGraphXY::getCoord(int id, vector<double>& coord){
  coord.clear();
  EnvXYHashEntry_t* hashEntry = StateID2CoordTable[id];
  vector<int> d_coord_agent(2,0);
  vector<double> coord_agent(2,0);
  coord.resize(EnvXYCfg.numAgents*2 + EnvXYCfg.numGoals);
  int i = 0;
  for(int agent_i=0; agent_i < EnvXYCfg.numAgents; agent_i++, i+=2){
    d_coord_agent[0] = hashEntry->poses[agent_i].x;
    d_coord_agent[1] = hashEntry->poses[agent_i].y;
    discToCont(d_coord_agent, coord_agent);
    coord[i] = coord_agent[0];
    coord[i+1] = coord_agent[1];
  }

  for(int goal_i = 0; goal_i < EnvXYCfg.numGoals; goal_i++, i++){
    coord[i] = hashEntry->goalsVisited[goal_i];
  }
  
  return true;
}

int EGraphXY::getStateID(const vector<double>& coord){
  return 0;
  vector<int> d_coord;
  contToDisc(coord,d_coord);
  std::vector<pose_t> poses;
  std::vector<bool> goalsVisited;
  int i =0;
  int ctr = 0;
  for(; i < EnvXYCfg.numAgents; i+=2)
    {
      pose_t pose;
      pose.x = d_coord[i];
      pose.y = d_coord[i+1];
      poses[ctr] = pose;
    }
  for(; i < EnvXYCfg.numGoals; i++)
    {
      goalsVisited[ctr] = coord[i];
    }
  return GetStateFromCoord(poses, goalsVisited); 
}

bool EGraphXY::isGoal(int id){
  EnvXYHashEntry_t* HashEntry = StateID2CoordTable[id];
  for(int i = 0; i < EnvXYCfg.numGoals; i++)
    {
      if (!HashEntry->goalsVisited[i])
	return false;
    }
  return true;
}

void EGraphXY::projectToHeuristicSpace(const vector<double>& coord, vector<int>& dp) const{
  int i = 0;
  dp.clear();
  for(int i = 0; i < (int) coord.size(); i+=2){
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

// Hack: assume there are n poses, and a vector of goals visited. At some point, 
// need to convert this to pose_t and bool vector instead of a vector<int>
void EGraphXY::contToDisc(const vector<double>& c, vector<int>& d){
  d.resize(c.size());
  int i;
  //SBPL_INFO("in conttodisc with (%f %f)", c[0],c[1]) ;
  for(i = 0; i < (int) c.size(); i +=2){
    PoseContToDisc(c[i], c[i+1], d[i], d[i+1]);
  }
}

void EGraphXY::discToCont(const vector<int>& d, vector<double>& c){
  c.resize(d.size());
  int i;
  //SBPL_INFO("in disctocont with (%d %d)", d[0],d[1]) ;
  for(i =0; i < (int) d.size(); i+=2){
    PoseDiscToCont(d[i], d[i+1], c[i], c[i+1]);
  }
}

bool EGraphXY::isValidEdge(const vector<double>& coord, const vector<double>& coord2, bool& change_cost, int& cost){
  return true; // assume static environment
  int id1 = getStateID(coord);
  int id2 = getStateID(coord2);

  vector<int> children;
  vector<int> costs;

  GetSuccs(id1, &children, &costs);
  for(unsigned int i=0; i<children.size(); i++){
    if(children[i]==id2){
      change_cost = true;
      cost = costs[i];
      return true;
    }
  }

  GetSuccs(id2, &children, &costs);
  for(unsigned int i=0; i<children.size(); i++){
    if(children[i]==id1){
      change_cost = true;
      cost = costs[i];
      return true;
    }
  }

  int id;
  if(snap(coord, coord2, id, cost)){
    change_cost = true;
    return true;
  }

  change_cost = false;
  return false;
}

bool EGraphXY::isValidVertex(const vector<double>& coord){
  vector<int> d_coord;
  contToDisc(coord,d_coord);
  int temp_cost;
  return collisionCheckPose(d_coord[0],d_coord[1],temp_cost);
}

//TODO: This can be made much faster by just changing goalsVisited to a vector of assignment ints
void EGraphXY::getAssignments(const std::vector<int>& solution_stateIDs_V, std::vector<int>& assignments) const{
  assignments.clear();
  assignments.resize(EnvXYCfg.numGoals, -1);
  std::vector<bool> goalsVisited(EnvXYCfg.numGoals, false);
  for(unsigned int i = 0; i < solution_stateIDs_V.size(); i++){
    std::vector<pose_t> poses;
    GetCoordFromState(solution_stateIDs_V[i], poses, goalsVisited);
    for(unsigned int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++){
      int x = poses[agent_i].x;
      int y = poses[agent_i].y;
      for(unsigned int goal_i = 0; goal_i < EnvXYCfg.numGoals; goal_i++){
	if((assignments[goal_i] == -1) && (x == EnvXYCfg.goal[goal_i].x) && (y == EnvXYCfg.goal[goal_i].y)){
	  //SBPL_INFO("Assigning goal %d to agent %d", goal_i, agent_i);
	  assignments[goal_i] = agent_i;
	}
      }
    }
  }
}

visualization_msgs::MarkerArray EGraphXY::stateToVisualizationMarker(vector<double> coord){
  // coord looks like [x1,y1,theta1,x2,y2,theta2....]
  visualization_msgs::MarkerArray ma;
  for(int i = 0; i < coord.size(); i+=2)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.pose.position.x = coord[i]; 
      marker.pose.position.y = coord[i+1]; 
      marker.pose.position.z = 0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.scale.z = 0.02;
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;

      ma.markers.push_back(marker);
    }
  return ma;
}


visualization_msgs::MarkerArray EGraphXY::stateToDetailedVisualizationMarker(vector<double> coord){
  return stateToVisualizationMarker(coord);
}



visualization_msgs::MarkerArray EGraphXY::edgeToVisualizationMarker(vector<double> coord, vector<double> coord2){
  visualization_msgs::MarkerArray ma;
  for(int i = 0; i < coord.size(); i+=2)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.scale.x = 0.01;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      geometry_msgs::Point p;
      p.x = coord[i];
      p.y = coord[i+1];
      p.z = 0;
      marker.points.push_back(p);
      p.x = coord2[i];
      p.y = coord2[i+1];
      marker.points.push_back(p);

      ma.markers.push_back(marker);
    }
  return ma;
}
   
