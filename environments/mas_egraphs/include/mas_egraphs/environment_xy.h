/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ENVIRONMENT_XY_H_
#define __ENVIRONMENT_XY_H_

#ifndef ROS
#define ROS
#endif

#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/utils.h>

#define ENVXY_DEFAULTOBSTHRESH 20	//see explanation of the value below
#define DEFAULTACTIONWIDTH 6

class SBPL2DGridSearch;

typedef struct
{
  int x;
  int y;
} pose_t;

typedef struct
{
  pose_t pose;
  std::vector<bool> goalsVisited;
} AgentState_t;

typedef struct
{
  int stateID;
  std::vector<bool> goalsVisited; 
  std::vector<bool> activeAgents;
  std::vector<pose_t> poses;
  int iteration;
} EnvXYHashEntry_t;


typedef struct
{
    int startstateid;
    int goalstateid;

    bool bInitialized;

    //any additional variables
} Environment_xy_t;


typedef struct ENV_XY_CONFIG
{
  int numAgents;
  int numGoals;
  int EnvWidth_c;
  int EnvHeight_c;
  int actionwidth;
  std::vector<pose_t> start;
  std::vector<pose_t> goal;
  unsigned char** Grid2D;
  unsigned char obsthresh;
  double cellsize_m;
  double nominalvel_mpersecs;
}EnvXYConfig_t;

class EnvXY_InitParms
{
public:
  unsigned int numAgents;
  const unsigned char* mapdata;
  std::vector<pose_t> start;
  std::vector<pose_t> goal;
  double goaltol_x;
  double goaltol_y;
  double goaltol_theta;
};


class Environment_xy: public DiscreteSpaceInformation
{
 public:
  /**                                                                                                                                                                                            
   * \brief mapping from hashentry stateID (used in environment to contain                                                                                                                       
   *        the coordinates of a state, say x,y or x,y,theta)                                                                                                                                    
   *        to an array of state indices used in searches.                                                                                                                                       
   * If a single search is done, then it is a single entry.  So                                                                                                                                  
   * StateID2IndexMapping[100][0] = 5 means that hashentry with stateID 100                                                                                                                      
   * is mapped onto search index = 5 in search 0 The value of -1 means that                                                                                                                      
   * no search state has been created yet for this hashentry                                                                                                                                     
   */
  std::vector<int*> StateID2IndexMapping;

  Environment_xy();
  
  virtual ~Environment_xy();
  /**
   * \brief see comments on the same function in the parent class
   */
  virtual bool InitializeEnv();
  
  virtual bool InitializeEnv(const char* sEnvFile);

  virtual bool InitializeEnv(int width, int height, const unsigned char* mapdata, int numagents, 
			     std::vector<pose_t> start, std::vector<pose_t> goal,
			     double goaltol_x, double goaltol_y,
			     double cellsize_m, double nominalvel_mpersecs);
  
  void SetConfiguration(int width, int height, const unsigned char* mapdata,
			std::vector<pose_t> start, std::vector<pose_t> goal,
			double cellsize_m, double nominalvel_mpersecs);

  void ReadConfiguration(FILE* fCfg);

  void SetAllActionsandAllOutcomes(CMDPSTATE* state);
  /**
   * \brief way to set up various parameters. For a list of parameters, see
   *        the body of the function - it is pretty straightforward
   */
    virtual bool SetEnvParameter(const char* parameter, int value);
    
    /**
     * \brief returns the value of specific parameter - see function body for the list of parameters
     */
    virtual int GetEnvParameter(const char* parameter);
    
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);
    
    virtual bool UpdateCost(int x, int y, unsigned char newcost);
    
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetGoalHeuristic(int stateID);

    virtual bool IsValidCell(int X, int Y);    

    virtual bool IsValidConfiguration(std::vector<pose_t> pos);
    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetStartHeuristic(int stateID);

    /**
     * \brief sets number of goals
     */
    virtual bool SetNumGoals(int numgoals);

    virtual bool IsObstacle(int x, int y);
    /**
     * \brief sets number of agents
     */
    virtual bool SetNumAgents(int numagents);
    
    virtual void SetAllPreds(CMDPSTATE* state){};
    
    virtual void PrintEnv_Config(FILE* fOut){};

    void GetCoordFromState(int stateID, std::vector<pose_t>& poses, std::vector<bool>& goalsVisited, std::vector<bool>& activeAgents) const;

    virtual int GetStateFromCoord(std::vector<pose_t>& poses, std::vector<bool> goalsVisited,
				  std::vector<bool> activeAgents);
    /**
     * \brief sets start in meters/radians
     */
    virtual int SetStart(std::vector<sbpl_xy_theta_pt_t> start);

    bool SetMap(const unsigned char* mapdata);

    /**
     * \brief sets goal in meters/radians
     */
    virtual int SetGoal(std::vector<sbpl_xy_theta_pt_t> goal);
    
    virtual bool GetFakePlan(int startstateID, std::vector<int>& solutionstateIDs);
   
    /**
     * \brief gets number of goals
     */
    virtual int GetNumGoals() const;

    /**
     * \brief gets number of agents
     */
    virtual int GetNumAgents() const;

    virtual bool isGoal(int id);
    virtual bool isGoal(const pose_t pose);
    
    virtual bool isStart(int id);

    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost);

    virtual std::vector<bool> getGoalsVisited(const std::vector<pose_t>& poses, std::vector<bool> goalsVisitedSoFar);

    virtual unsigned int GETHASHBIN(std::vector<pose_t> pose, std::vector<bool> goalsVisited, std::vector<bool> activeAgents);
    
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

    /**
     * \brief returns the cost corresponding to the cell <x,y>
     */
    virtual unsigned char GetMapCost(int x, int y);

    /**
     * \brief returns true if cell is within map
     */
    virtual bool IsWithinMapCell(int X, int Y);

    /**
     * \brief Transform a pose into discretized form. 
     * \note Even if this method returns false, you can still use the
     *       computed indices, for example to figure out how big your map
     *       should have been.
     *
     * \return true if the resulting indices lie within the grid bounds
     *         and the angle was valid.
     */
    virtual bool PoseContToDisc(double px, double py, int &ix, int &iy) const;

    /** \brief Transform grid indices into a continuous pose. The computed
     *         angle lies within 0<=pth<2pi.
     *
     * \note Even if this method returns false, you can still use the
     *      computed indices, for example to figure out poses that lie
     *      outside of your current map.
     *
     * \return true if all the indices are within grid bounds.
     */
    virtual bool PoseDiscToCont(int ix, int iy, double &px, double &py) const;
    virtual int SizeofCreatedEnv();

 protected:
    // member data
    EnvXYConfig_t EnvXYCfg;
    Environment_xy_t EnvXY;
    std::vector<sbpl_xy_theta_cell_t> affectedsuccstatesV; //arrays of states whose outgoing actions cross cell 0,0
    std::vector<sbpl_xy_theta_cell_t> affectedpredstatesV; //arrays of states whose incoming actions cross cell 0,0
    int iteration;

    //2D search for heuristic computations
    bool bNeedtoRecomputeStartHeuristics; //set whenever grid2Dsearchfromstart needs to be re-executed
    bool bNeedtoRecomputeGoalHeuristics; //set whenever grid2Dsearchfromgoal needs to be re-executed
    SBPL2DGridSearch* grid2Dsearchfromstart; //computes h-values that estimate distances from start x,y to all cells
    SBPL2DGridSearch* grid2Dsearchfromgoal; //computes h-values that estimate distances to goal x,y from all cells

    //hash table of size x_size*y_size. Maps from coords to stateId
    int HashTableSize;
    std::vector<EnvXYHashEntry_t*>* Coord2StateIDHashTable;
    //vector that maps from stateID to coords
    std::vector<EnvXYHashEntry_t*> StateID2CoordTable;


    virtual EnvXYHashEntry_t* GetHashEntry_hash(std::vector<pose_t>& pose, std::vector<bool> goalsVisited, std::vector<bool> activeAgents);
    virtual EnvXYHashEntry_t* CreateNewHashEntry_hash(std::vector<pose_t>& pose, std::vector<bool> goalsVisited, std::vector<bool> activeAgents);
    virtual bool IsEqualHashEntry(EnvXYHashEntry_t* hashentry, std::vector<pose_t>& poses, 
				  std::vector<bool> GoalsVisited, std::vector<bool> activeAgents) const;
    
    //pointers to functions
    EnvXYHashEntry_t* (Environment_xy::*GetHashEntry)(std::vector<pose_t>& pose, std::vector<bool> goalsVisited, std::vector<bool> activeAgents);
    EnvXYHashEntry_t* (Environment_xy::*CreateNewHashEntry)(std::vector<pose_t>& pose, std::vector<bool> goalsVisited, std::vector<bool> activeAgents);
    
};

#endif
