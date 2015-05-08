#include<mas_egraphs/egraph_xytheta.h>
#include<mas_egraphs/environment_masnavxythetalat.h>

using namespace std;

EGraphXYTheta::EGraphXYTheta(){
}


bool EGraphXYTheta::InitializeEnv(int width, int height,
				  const unsigned char* mapdata,
				  int numagents,
				  std::vector<pose_t> start,
				  std::vector<pose_t> goal,				  
				  double goaltol_x, double goaltol_y, double goaltol_theta,
				  const vector<sbpl_2Dpt_t> & perimeterptsV,
				  double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
				  unsigned char obsthresh,  const char* sMotPrimFile){

  bool ret = EnvironmentMASNAVXYTHETALAT::InitializeEnv(width, height, mapdata, numagents, start,
                                              goal, goaltol_x, goaltol_y, goaltol_theta,
                                              perimeterptsV, cellsize_m, nominalvel_mpersecs, 
                                              timetoturn45degsinplace_secs, obsthresh, sMotPrimFile);

  if(ret){
    footprints.resize(EnvMASNAVXYTHETALATCfg.NumThetaDirs);
    for(int i=0; i<EnvMASNAVXYTHETALATCfg.NumThetaDirs; i++){
      sbpl_xy_theta_pt_t pt;
      pt.x = 0;
      pt.y = 0;
      pt.theta = DiscTheta2Cont(i, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
      get_2d_footprint_cells(EnvMASNAVXYTHETALATCfg.FootprintPolygon, &(footprints[i]), pt, EnvMASNAVXYTHETALATCfg.cellsize_m);
    }
  }
  return ret;
}

bool EGraphXYTheta::collisionCheckPose(int x, int y, int theta, int& cost){
  if(!IsValidCell(x, y))
    return false;

  if(EnvMASNAVXYTHETALATCfg.Grid2D[x][y] >= EnvMASNAVXYTHETALATCfg.cost_inscribed_thresh)
    return false;

  //check collisions that for the particular footprint orientation along the action
  if(EnvMASNAVXYTHETALATCfg.FootprintPolygon.size() > 1 && EnvMASNAVXYTHETALATCfg.Grid2D[x][y] >= EnvMASNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh){ 
    for(unsigned int i=0; i<footprints[theta].size(); i++){ 
      //check validity
      if(!IsValidCell(x+footprints[theta][i].x, y+footprints[theta][i].y))
        return false;
    }
  }

  cost = EnvMASNAVXYTHETALATCfg.Grid2D[x][y]; 
  return true;
}


bool EGraphXYTheta::snap(const vector<double>& from, const vector<double>& to, int& id, int& cost){
  return false;
}


//requires a getCoord function which takes a state id (the ids the environment uses) and returns a vector with the coordinate so we can look for shortcuts in the e-graph
//-we will never call getCoord on the goal (because it is possible we don't know what the goal state looks like)
bool EGraphXYTheta::getCoord(int id, vector<double>& coord){
  /*
  return true;
  EnvMASNAVXYTHETALATHashEntry_t* hashEntry = StateID2CoordTable[id];
  vector<int> d_coord(3,0);
  d_coord[0] = hashEntry->X;
  d_coord[1] = hashEntry->Y;
  d_coord[2] = hashEntry->Theta;
  discToCont(d_coord,coord);
  return true;
  */
}

int EGraphXYTheta::getStateID(const vector<double>& coord){
  return 0;
  /*
  vector<int> d_coord;
  contToDisc(coord,d_coord);
  return GetStateFromCoord(d_coord[0],d_coord[1],d_coord[2]);
  */
}

bool EGraphXYTheta::isGoal(int id){
  EnvMASNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[id];
  for(int i = 0; i < EnvMASNAVXYTHETALATCfg.NumGoals; i++)
    {
      if (!HashEntry->goalsVisited[i])
	return false;
    }
  return true;
}

void EGraphXYTheta::projectToHeuristicSpace(const vector<double>& coord, vector<int>& dp) const{
  int x = CONTXY2DISC(coord[0], EnvMASNAVXYTHETALATCfg.cellsize_m);
  int y = CONTXY2DISC(coord[1], EnvMASNAVXYTHETALATCfg.cellsize_m);
  dp.clear();
  dp.push_back(x);
  dp.push_back(y);
  //ROS_INFO("project (%f %f) -> (%d %d)",coord[0],coord[1],dp[0],dp[1]);
}

void EGraphXYTheta::projectGoalToHeuristicSpace(vector<int>& dp) const{
  EnvMASNAVXYTHETALATHashEntry_t* hashEntry = StateID2CoordTable[EnvMASNAVXYTHETALAT.goalstateid];
  dp.clear();
  //ROS_INFO("project goal -> (%d %d)",dp[0],dp[1]);
}

void EGraphXYTheta::contToDisc(const vector<double>& c, vector<int>& d){
  d.resize(3);
  PoseContToDisc(c[0],c[1],c[2],d[0],d[1],d[2]);
}

void EGraphXYTheta::discToCont(const vector<int>& d, vector<double>& c){
  c.resize(3);
  PoseDiscToCont(d[0],d[1],d[2],c[0],c[1],c[2]);
}

bool EGraphXYTheta::isValidEdge(const vector<double>& coord, const vector<double>& coord2, bool& change_cost, int& cost){
  int id1 = getStateID(coord);
  int id2 = getStateID(coord2);

  vector<int> children;
  vector<int> costs;

  GetSuccs(id1, &children, &costs, NULL);
  for(unsigned int i=0; i<children.size(); i++){
    if(children[i]==id2){
      change_cost = true;
      cost = costs[i];
      return true;
    }
  }
  GetSuccs(id2, &children, &costs, NULL);
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

  //TODO: and demonstrations?

  change_cost = false;
  return false;
}

bool EGraphXYTheta::isValidVertex(const vector<double>& coord){
  vector<int> d_coord;
  contToDisc(coord,d_coord);
  int temp_cost;
  return collisionCheckPose(d_coord[0],d_coord[1],d_coord[2],temp_cost);
}

visualization_msgs::MarkerArray EGraphXYTheta::stateToVisualizationMarker(vector<double> coord){
  // coord looks like [x1,y1,theta1,x2,y2,theta2....]
  visualization_msgs::MarkerArray ma;
  for(int i = 0; i < coord.size(); i+=3)
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

visualization_msgs::MarkerArray EGraphXYTheta::stateToDetailedVisualizationMarker(vector<double> coord){
  return stateToVisualizationMarker(coord);
}

visualization_msgs::MarkerArray EGraphXYTheta::edgeToVisualizationMarker(vector<double> coord, vector<double> coord2){
  visualization_msgs::MarkerArray ma;
  for(int i = 0; i < coord.size(); i+=3)
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
