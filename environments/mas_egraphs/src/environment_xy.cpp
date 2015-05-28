#include <cmath>
#include <cstring>
#include <ctime>
#include <mas_egraphs/environment_xy.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/utils/key.h>

using namespace std;

#ifndef DEBUG
#define DEBUG
#endif

Environment_xy::Environment_xy()
{
  EnvXYCfg.numAgents = 1;
  grid2Dsearchfromstart = NULL;
  grid2Dsearchfromgoal = NULL;
  bNeedtoRecomputeStartHeuristics = true;
  bNeedtoRecomputeGoalHeuristics = true;
  iteration = 0;
  
  //no memory allocated in cfg yet
  EnvXYCfg.Grid2D = NULL;
  
  HashTableSize = 0;
  Coord2StateIDHashTable = NULL;
  EnvXYCfg.actionwidth = DEFAULTACTIONWIDTH;
}

Environment_xy::~Environment_xy()
{
    SBPL_INFO("destroying XYLATTICE\n");
    if (grid2Dsearchfromstart != NULL) delete grid2Dsearchfromstart;
    grid2Dsearchfromstart = NULL;

    if (grid2Dsearchfromgoal != NULL) delete grid2Dsearchfromgoal;
    grid2Dsearchfromgoal = NULL;

    if (EnvXYCfg.Grid2D != NULL) {
        for (int x = 0; x < EnvXYCfg.EnvWidth_c; x++)
            delete[] EnvXYCfg.Grid2D[x];
        delete[] EnvXYCfg.Grid2D;
        EnvXYCfg.Grid2D = NULL;
    }
    
    //delete the states themselves first
    for (int i = 0; i < (int)StateID2CoordTable.size(); i++) {
        delete StateID2CoordTable.at(i);
        StateID2CoordTable.at(i) = NULL;
    }
    StateID2CoordTable.clear();

    //delete hashtable
    if (Coord2StateIDHashTable != NULL) {
        delete[] Coord2StateIDHashTable;
        Coord2StateIDHashTable = NULL;
    }
}


static unsigned int inthash(unsigned int key)
{
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

//TODO: does not use goalsVisited in hash computation
unsigned int Environment_xy::GETHASHBIN(std::vector<pose_t> poses, std::vector<bool> goalsVisited)
{
  unsigned int hashbin = 0;
  for(int i = 0; i < (int) poses.size(); ++i)
    {
      hashbin += inthash(inthash(poses[i].x) + (inthash(poses[i].y) << 1));
    }
  return inthash(hashbin) & (HashTableSize - 1);
}

bool Environment_xy::InitializeEnv()
{
    EnvXYHashEntry_t* HashEntry;

    //int maxsize = EnvMASNAVXYCfg.EnvWidth_c * EnvMASNAVXYCfg.EnvHeight_c * EnvMASNAVXYCfg.NumThetaDirs;
    
    SBPL_INFO("environment stores states in hashtable\n");
    
    //initialize the map from Coord to StateID
    HashTableSize = 4 * 1024 * 1024; //should be power of two
    Coord2StateIDHashTable = new vector<EnvXYHashEntry_t*> [HashTableSize];
    GetHashEntry = &Environment_xy::GetHashEntry_hash;
    CreateNewHashEntry = &Environment_xy::CreateNewHashEntry_hash;
    
    //initialize the map from StateID to Coord
    StateID2CoordTable.clear();

    //create start state
    std::vector<bool> goalsVisited(EnvXYCfg.numGoals,false);
    if ((HashEntry = (this->*GetHashEntry)(EnvXYCfg.start, goalsVisited)) == NULL) {
      //have to create a new entry
      HashEntry = (this->*CreateNewHashEntry)(EnvXYCfg.start, goalsVisited);
    }
    EnvXY.startstateid = HashEntry->stateID;

    //create goal state
    if ((HashEntry = (this->*GetHashEntry)(EnvXYCfg.start, goalsVisited))==NULL) {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvXYCfg.start,
						goalsVisited);
    }
    EnvXY.goalstateid = HashEntry->stateID;

    //initialized
    EnvXY.bInitialized = true;
    return true;
}


bool Environment_xy::InitializeEnv(int width, int height, const unsigned char* mapdata, int numagents, 
				   std::vector<pose_t> start, std::vector<pose_t> goal,
				   double goaltol_x, double goaltol_y,
				   double cellsize_m, double nominalvel_mpersecs)
						    
{
    SBPL_INFO("env: initialize with width=%d height=%d "
                "cellsize=%.3f nomvel=%.3f\n",
                width, height, cellsize_m, nominalvel_mpersecs);

    //TODO - need to set the tolerance as well
    for(int i = 0; i < (int)start.size(); i++){
      start[i].x = CONTXY2DISC(start[i].x, cellsize_m);
      start[i].y = CONTXY2DISC(start[i].y, cellsize_m);
    }
    
    for(int i = 0; i < (int)goal.size(); i++){
      goal[i].x = CONTXY2DISC(goal[i].x, cellsize_m);
      goal[i].y = CONTXY2DISC(goal[i].y, cellsize_m);
    }
    
    SetConfiguration(width, height, mapdata, start, goal,
                     cellsize_m, nominalvel_mpersecs);

    InitializeEnv();
    return true;
}

bool Environment_xy::PoseContToDisc(double px, double py, int &ix, int &iy) const
{
    ix = CONTXY2DISC(px, EnvXYCfg.cellsize_m);
    iy = CONTXY2DISC(py, EnvXYCfg.cellsize_m);
    return (ix >= 0) && (ix < EnvXYCfg.EnvWidth_c) &&
           (iy >= 0) && (iy < EnvXYCfg.EnvHeight_c);
}

bool Environment_xy::PoseDiscToCont(int ix, int iy, double &px, double &py) const
{
    px = DISCXY2CONT(ix, EnvXYCfg.cellsize_m);
    py = DISCXY2CONT(iy, EnvXYCfg.cellsize_m);
    return (ix >= 0) && (ix < EnvXYCfg.EnvWidth_c)
      && (iy >= 0) && (iy < EnvXYCfg.EnvHeight_c);
}

unsigned char Environment_xy::GetMapCost(int x, int y)
{
    return EnvXYCfg.Grid2D[x][y];
}

void Environment_xy::SetConfiguration(int width, int height, const unsigned char* mapdata,
				      std::vector<pose_t> start, std::vector<pose_t> goal,
				      double cellsize_m, double nominalvel_mpersecs){
    EnvXYCfg.EnvWidth_c = width;
    EnvXYCfg.EnvHeight_c = height;
    EnvXYCfg.start = start;

    EnvXYCfg.goal = goal;

    EnvXYCfg.nominalvel_mpersecs = nominalvel_mpersecs;
    EnvXYCfg.cellsize_m = cellsize_m;
    EnvXYCfg.obsthresh = ENVXY_DEFAULTOBSTHRESH;
    //allocate the 2D environment
    EnvXYCfg.Grid2D = new unsigned char*[EnvXYCfg.EnvWidth_c];
    for (int x = 0; x < EnvXYCfg.EnvWidth_c; x++) {
        EnvXYCfg.Grid2D[x] = new unsigned char[EnvXYCfg.EnvHeight_c];
    }

    //environment:
    if (0 == mapdata) {
        for (int y = 0; y < EnvXYCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvXYCfg.EnvWidth_c; x++) {
                EnvXYCfg.Grid2D[x][y] = 0;
            }
        }
    }
    else {
        for (int y = 0; y < EnvXYCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvXYCfg.EnvWidth_c; x++) {
                EnvXYCfg.Grid2D[x][y] = mapdata[x + y * width];
            }
        }
    }
}

bool Environment_xy::UpdateCost(int x, int y, unsigned char newcost)
{
  //SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,EnvXYCfg.Grid2D[x][y], newcost);
  
  EnvXYCfg.Grid2D[x][y] = newcost;
  return true;
}


// -------------- unnecessary functions

bool Environment_xy::InitializeEnv(const char* cfgFile)
{
  return false;
}

void Environment_xy::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
}

void Environment_xy::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
}

bool Environment_xy::InitializeMDPCfg(MDPConfig *MDPCfg){
  return true;}

void Environment_xy::ReadConfiguration(FILE* fCfg){
  SBPL_INFO("Undefined");
};

int Environment_xy::GetGoalHeuristic(int stateID)
{
  SBPL_INFO("TODO: GetGoalHeuristic not defined");
  return 0;
}

int Environment_xy::GetStartHeuristic(int stateID)
{
  SBPL_INFO("TODO: GetStartHeuristic not defined");
  return 0;
}


//--------------------- End of unnecessary functions

bool Environment_xy::IsObstacle(int x, int y)
{
#if DEBUG
    SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,EnvXYCfg.Grid2D[x][y]);
#endif

    return (EnvXYCfg.Grid2D[x][y] >= EnvXYCfg.obsthresh);
}

bool Environment_xy::SetMap(const unsigned char* mapdata)
{
    int xind = -1, yind = -1;

    for (xind = 0; xind < EnvXYCfg.EnvWidth_c; xind++) {
        for (yind = 0; yind < EnvXYCfg.EnvHeight_c; yind++) {
            EnvXYCfg.Grid2D[xind][yind] = mapdata[xind + yind * EnvXYCfg.EnvWidth_c];
        }
    }

    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;

    return true;
}

void Environment_xy::GetCoordFromState(int stateID, vector<pose_t>& poses, vector<bool>& goalsVisited) const
{
    EnvXYHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    poses = HashEntry->poses;
    goalsVisited = HashEntry->goalsVisited;
}

int Environment_xy::GetStateFromCoord(vector<pose_t>& poses, vector<bool> goalsVisited) 
{
  EnvXYHashEntry_t* OutHashEntry;
  if ((OutHashEntry = (this->*GetHashEntry)(poses, goalsVisited)) == NULL) {
    //have to create a new entry
    OutHashEntry = (this->*CreateNewHashEntry)(poses, goalsVisited);
  }
  return OutHashEntry->stateID;
}

int Environment_xy::GetNumGoals() const
{
  return EnvXYCfg.numGoals;
}

int Environment_xy::GetNumAgents() const
{
  return EnvXYCfg.numAgents;
} 

bool Environment_xy::SetNumGoals(int numgoals)
{
  if (numgoals < 0)
    {
      SBPL_INFO("ERROR: Number of goals cannot be negative");
      return false;
    }
  EnvXYCfg.numGoals = numgoals;
  return true;
}


bool Environment_xy::SetNumAgents(int numagents)
{
  if (numagents < 0)
    {
      SBPL_INFO("ERROR: Number of agents cannot be negative");
      return false;
    }
  EnvXYCfg.numAgents = numagents;
  return true;
}

bool Environment_xy::isGoal(int id){
  // any state with all goals visited is a goal state
  EnvXYHashEntry_t* HashEntry = StateID2CoordTable[id];
  for(int i = 0; i < EnvXYCfg.numGoals; i++)
    {
      if (!HashEntry->goalsVisited[i])
	return false;
    }
  return true;
}

int Environment_xy::SetGoal(std::vector<sbpl_xy_theta_pt_t> goal_m)
{
  std::vector<pose_t> goal(EnvXYCfg.numGoals);
  for (int goal_i = 0; goal_i < EnvXYCfg.numGoals; goal_i ++)
    {
      int x = CONTXY2DISC(goal_m[goal_i].x, EnvXYCfg.cellsize_m);
      int y = CONTXY2DISC(goal_m[goal_i].y, EnvXYCfg.cellsize_m);

      SBPL_INFO("env: setting goal to %.3f %.3f (%d %d)", goal_m[goal_i].x, goal_m[goal_i].y, x, y);

      if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x, y);
        return -1;
      }
      goal[goal_i].x = x;
      goal[goal_i].y = y;
    }
      
    if (!IsValidConfiguration(goal)) {
        SBPL_INFO("WARNING: goal configuration is invalid\n");
    }
    
    EnvXYHashEntry_t* OutHashEntry;
    std::vector<bool> goalsVisited(EnvXYCfg.numGoals, true);
    if ((OutHashEntry = (this->*GetHashEntry)(goal, goalsVisited)) == NULL) {
      //have to create a new entry
      OutHashEntry = (this->*CreateNewHashEntry)(goal, goalsVisited);
    }

    //need to recompute start heuristics?
    if (EnvXY.goalstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
        bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
    }
    // note that goal is unique
    EnvXY.goalstateid = OutHashEntry->stateID;

    EnvXYCfg.goal = goal;

    return EnvXY.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int Environment_xy::SetStart(std::vector<sbpl_xy_theta_pt_t> start_m)
{
  std::vector<pose_t> start(EnvXYCfg.numAgents);
  std::vector<int> x(EnvXYCfg.numAgents), y(EnvXYCfg.numAgents);
  for (int agent_i = 0; agent_i < EnvXYCfg.numAgents; ++agent_i)
	   {
	     int x_agent = CONTXY2DISC(start_m[agent_i].x, EnvXYCfg.cellsize_m);
	     int y_agent = CONTXY2DISC(start_m[agent_i].y, EnvXYCfg.cellsize_m);
	     
	     if (!IsWithinMapCell(x_agent, y_agent)) {
	       SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x_agent, y_agent);
	       return -1;
	     }
	     
	     SBPL_INFO("env: setting start of Agent %d to %.3f %.3f (%d %d)\n", (agent_i+1), start_m[agent_i].x, start_m[agent_i].y, x_agent, y_agent);
	     start[agent_i].x = x_agent;
	     start[agent_i].y = y_agent;
	   }  

  if (!IsValidConfiguration(start)) {
    SBPL_INFO("WARNING: start configuration is invalid\n");
  }
	   
    EnvXYHashEntry_t* OutHashEntry; 
    std::vector<bool> goalsvisitedsofar(EnvXYCfg.numGoals, false);
    std::vector<bool> goalsvisited = getGoalsVisited(start, goalsvisitedsofar);
    if ((OutHashEntry = (this->*GetHashEntry)(start, goalsvisited)) == NULL) {
      //have to create a new entry
      OutHashEntry = (this->*CreateNewHashEntry)(start, goalsvisited);
    }

    //need to recompute start heuristics?
    if (EnvXY.startstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true;
        //because termination condition can be not all states TODO - make it dependent on term. condition
        bNeedtoRecomputeGoalHeuristics = true; 
    }

    //set start
    EnvXY.startstateid = OutHashEntry->stateID;
    EnvXYCfg.start = start;
    return EnvXY.startstateid;
}



// Hashing functions
EnvXYHashEntry_t* Environment_xy::GetHashEntry_hash(std::vector<pose_t>& poses, std::vector<bool> GoalsVisited)
{
  
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    int binid = GETHASHBIN(poses, GoalsVisited);

#if DEBUG
    if ((int)Coord2StateIDHashTable[binid].size() > 5)
    {
      SBPL_INFO(fDeb, "WARNING: Hash table has a bin %d of size %d\n",
		binid, (int)Coord2StateIDHashTable[binid].size());
      //PrintHashTableHist(fDeb);
    }
#endif
    
    //iterate over the states in the bin and select the perfect match
    vector<EnvXYHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
    for (int ind = 0; ind < (int)binV->size(); ind++) {
#if TIME_DEBUG	
      time_gethash += clock()-currenttime;
#endif
      EnvXYHashEntry_t* hashentry = binV->at(ind);
      bool matchfound = IsEqualHashEntry(hashentry, poses, GoalsVisited);
      if (matchfound)
	return hashentry;
    }
    return NULL;
}

bool Environment_xy::IsEqualHashEntry(EnvXYHashEntry_t* hashentry, std::vector<pose_t>& poses, std::vector<bool> GoalsVisited)
{
  bool matchfound = true;
  for (int agent_i =0; agent_i <  EnvXYCfg.numAgents; agent_i++){
    if ( (hashentry->poses[agent_i].x == poses[agent_i].x) && (hashentry->poses[agent_i].y == poses[agent_i].y) && (hashentry->goalsVisited == GoalsVisited)) {
      }
    else {
      matchfound = false;
      break;
    }
  }
  return matchfound;
}


EnvXYHashEntry_t* Environment_xy::CreateNewHashEntry_hash(std::vector<pose_t>& poses, std::vector<bool> GoalsVisited)
{
  int i;

#if TIME_DEBUG	
    clock_t currenttime = clock();
#endif

    EnvXYHashEntry_t* HashEntry = new EnvXYHashEntry_t;

    HashEntry->poses = poses;
    HashEntry->goalsVisited = GoalsVisited;
    HashEntry->iteration = 0;

    HashEntry->stateID = StateID2CoordTable.size();
   
    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);
    //get the hash table bin
    i = GETHASHBIN(HashEntry->poses, HashEntry->goalsVisited);

    //insert the entry into the bin
    Coord2StateIDHashTable[i].push_back(HashEntry);

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif
    return HashEntry;
}

void Environment_xy::GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<std::vector<int> >* PerAgentCostV, std::vector<bool>* isTrueCost){
  GetSuccs(SourceStateID, SuccIDV, PerAgentCostV, CostV);
  for(int i = 0; i < (int)SuccIDV->size(); i++)
    isTrueCost->push_back(true);
}

//lazysuccswithuniqueids returns true succs
void Environment_xy::GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){
  GetSuccsWithUniqueIds(SourceStateID, SuccIDV, CostV);
  for(int i = 0; i < (int)SuccIDV->size(); i++)
    isTrueCost->push_back(true);
}

void Environment_xy::GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){
  GetSuccs(SourceStateID, SuccIDV, CostV);
}

void Environment_xy::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){
  // Keep virtual functions happy
}

void Environment_xy::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<std::vector<int> >* PerAgentCostV, std::vector<int>* CostV)
{
  //clear the successor array
  SuccIDV->clear();
  CostV->clear();

  SuccIDV->reserve(EnvXYCfg.actionwidth);
  CostV->reserve(EnvXYCfg.actionwidth);

  //get X, Y for the state
  EnvXYHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  // goal state should be absorbing
  std::vector<bool> allgoalsVisited(EnvXYCfg.numGoals,true);
  if (HashEntry->goalsVisited == allgoalsVisited)
    return;
  
  // Make numAgents x numActions structure of poses and costs
  std::vector<std::vector<pose_t>> allnewPoses(EnvXYCfg.numAgents);
  std::vector<std::vector<int>> allnewCosts(EnvXYCfg.numAgents);
  int cost;
  const int numActions = EnvXYCfg.actionwidth;
  for(int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++){   
      allnewPoses[agent_i] = std::vector<pose_t> (numActions);
      allnewCosts[agent_i] = std::vector<int>(numActions);
    }

  //iterate through agents
  for(int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++){
    pose_t newPose;    
    // stop
    newPose.x = HashEntry->poses[agent_i].x;
    newPose.y = HashEntry->poses[agent_i].y;
    cost = 1/EnvXYCfg.nominalvel_mpersecs;
    allnewPoses[agent_i][4] = newPose;
    allnewCosts[agent_i][4] = 0;  // incur zero cost for stopping

    // right
    newPose.x = HashEntry->poses[agent_i].x + 1;
    newPose.y = HashEntry->poses[agent_i].y;
    if(!IsValidCell(newPose.x, newPose.y))
      cost = INFINITECOST;
    else
      cost = 1/EnvXYCfg.nominalvel_mpersecs; //TODO: cost (in time) = 1/nominalvelocity
    allnewPoses[agent_i][0] = newPose;
    allnewCosts[agent_i][0] = cost;
    
    //left
    newPose.x = HashEntry->poses[agent_i].x - 1;
    newPose.y = HashEntry->poses[agent_i].y;
    if(!IsValidCell(newPose.x, newPose.y))
      cost = INFINITECOST;
    else
      cost = 1/EnvXYCfg.nominalvel_mpersecs;
    allnewPoses[agent_i][1] = newPose;
    allnewCosts[agent_i][1] = cost;

    // up
    newPose.x = HashEntry->poses[agent_i].x;
    newPose.y = HashEntry->poses[agent_i].y + 1;
    if(!IsValidCell(newPose.x, newPose.y))
      cost = INFINITECOST;
    else
      cost = 1/EnvXYCfg.nominalvel_mpersecs;
    allnewPoses[agent_i][2] = newPose;
    allnewCosts[agent_i][2] = cost;

    // down
    newPose.x = HashEntry->poses[agent_i].x;
    newPose.y = HashEntry->poses[agent_i].y - 1;
    if(!IsValidCell(newPose.x, newPose.y))
      cost = INFINITECOST;
    else
      cost = 1/EnvXYCfg.nominalvel_mpersecs;
    allnewPoses[agent_i][3] = newPose;
    allnewCosts[agent_i][3] = cost;

  }

  EnvXYHashEntry_t* OutHashEntry;
  std::vector<pose_t> poses;
  // Make successors from all possible combinations of agents and actions
  // TODO: this can be precomputed
  // We have numActions^ numagents primitives
  int numPrimitives = pow(numActions, EnvXYCfg.numAgents);
  for(int i = 0; i < numPrimitives; i++){
    poses.clear();
    
    // action index is i in base numActions
      int index = i;
      cost = 0;
      std::vector<int> costperagent;
      for(int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++){
	int action_i = index % numActions;
	index = (int) index/numActions;	
	cost = std::max(cost, allnewCosts[agent_i][action_i]); // all costs are 1 anyway
	poses.push_back(allnewPoses[agent_i][action_i]);
	costperagent.push_back(allnewCosts[agent_i][action_i]);
      }
      if (cost >= INFINITECOST){
	  //SBPL_INFO("Found an infinite cost action");
	  continue;
	}
      if (cost == 0){
	// TODO: we don't want all robots stopped as an action
	continue;
      }
      PerAgentCostV->push_back(costperagent);
      std::vector<bool> goalsVisited = getGoalsVisited(poses, HashEntry->goalsVisited);
      if ((OutHashEntry = (this->*GetHashEntry)(poses, goalsVisited)) == NULL) {
	//have to create a new entry
	OutHashEntry = (this->*CreateNewHashEntry)(poses, goalsVisited);
      }
      
      SuccIDV->push_back(OutHashEntry->stateID);
      CostV->push_back(cost);
      //SBPL_INFO("Peragentcost is %d %d", costperagent[0], costperagent[1]);
  }
}

std::vector<bool> Environment_xy::getGoalsVisited(std::vector<pose_t> poses, std::vector<bool> goalsVisitedSoFar)
{
  std::vector<bool> goalsVisited = goalsVisitedSoFar;
  for(int i = 0; i < (int)poses.size(); i++){
    int x = poses[i].x;
    int y = poses[i].y;
    for(int j = 0; j < EnvXYCfg.numGoals; j++)
      {
	if((x == EnvXYCfg.goal[j].x) && (y == EnvXYCfg.goal[j].y))
	  goalsVisited[j] = true;
      }
  }
  return goalsVisited;
}

bool Environment_xy::IsValidCell(int X, int Y)
{
  return (X >= 0 && X < EnvXYCfg.EnvWidth_c && Y >= 0 && Y < EnvXYCfg.EnvHeight_c &&
	  EnvXYCfg.Grid2D[X][Y] < EnvXYCfg.obsthresh);
}


bool Environment_xy::IsValidConfiguration(std::vector<pose_t> pos)
{
  // collision check robots
  for(unsigned int agent_i = 0; agent_i < pos.size(); agent_i++){
    for(unsigned int agent2_i = agent_i+1; agent2_i < pos.size(); agent2_i++){
      if ((pos[agent_i].x == pos[agent2_i].x) && (pos[agent2_i].y == pos[agent2_i].y))
	return false;
    }      
  }

  for(int i = 0; i < (int) pos.size(); i++){
    if (!IsValidCell(pos[i].x, pos[i].y)){
	return false;
    }
  }
  return true;
}

int Environment_xy::SizeofCreatedEnv()
{
    return (int)StateID2CoordTable.size();
}

int Environment_xy::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  SBPL_INFO("TODO: GetStartHeuristic not define");
  return 0;
}

bool Environment_xy::IsWithinMapCell(int X, int Y)
{
    return (X >= 0 && X < EnvXYCfg.EnvWidth_c && Y >= 0 && Y < EnvXYCfg.EnvHeight_c);
}

int Environment_xy::GetEnvParameter(const char* parameter)
{  
  if (strcmp(parameter, "cost_obsthresh") == 0) 
      return (int)EnvXYCfg.obsthresh;
  else
    return 0; //TODO
}

bool Environment_xy::SetEnvParameter(const char* parameter, int value)
{
    if (EnvXY.bInitialized == true) {
        SBPL_ERROR("ERROR: all parameters must be set before initialization of the environment\n");
        return false;
    }

    SBPL_INFO("setting parameter %s to %d\n", parameter, value);

    if (strcmp(parameter, "cost_obsthresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvXYCfg.obsthresh = (unsigned char)value;
    }
    else if (strcmp(parameter, "numAgents") == 0){
      if (value < 0){
	SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
	return false;
      }
      EnvXYCfg.numAgents = value;
    }
    else {
        SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
        return false;
    } 
    return true;
}

bool Environment_xy::GetFakePlan(int startstateID, std::vector<int>& solutionstateIDs)
{
  // apply motion primitives
  std::vector<int> CostV;
  std::vector<int> SuccIDV;
  std::vector<bool> isTrueCost;
  int id = startstateID;
  for(int i = 0; i < 50; i++){
    //PrintState(id, true);
    solutionstateIDs.push_back(id);
    GetLazySuccsWithUniqueIds(id, &SuccIDV, &CostV, &isTrueCost);
    int newsucc_i = rand() % SuccIDV.size(); 
    for(int j = 0; j < SuccIDV.size(); j++){
	SBPL_INFO("Successor %d cost %d\n", j, CostV[j]);
	PrintState(SuccIDV[j],true);
	solutionstateIDs.push_back(SuccIDV[j]);
      }
    id = SuccIDV[newsucc_i];
  }
  return true;
}

void Environment_xy::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
    if(stateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvXY... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL) fOut = stdout;

    EnvXYHashEntry_t* HashEntry = StateID2CoordTable[stateID];

    if (stateID == EnvXY.goalstateid && bVerbose) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    for(int i = 0; i < EnvXYCfg.numAgents; i++){
      SBPL_INFO("Agent %d (X=%d Y=%d)", i, HashEntry->poses[i].x, HashEntry->poses[i].y);
    }

    SBPL_INFO("Goals Visited:");
    for(int i = 0; i < EnvXYCfg.numGoals; i++){
      SBPL_INFO("%d ", (int) HashEntry->goalsVisited[i]);
    } 
}
