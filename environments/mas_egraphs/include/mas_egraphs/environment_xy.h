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
#include <sbpl/headers.h>
//#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/utils.h>
#include <string>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <stdlib.h>

#define ENVXY_DEFAULTOBSTHRESH 20	//see explanation of the value below
#define DEFAULTACTIONWIDTH 8

class SBPL2DGridSearch;

typedef struct
{
  int x;
  int y;
  int z;
  char theta;
} pose_disc_t;

typedef struct
{
  double x;
  double y;
  double z;
  double theta;
} pose_cont_t;

typedef struct
{
  int stateID;
  std::vector<pose_disc_t> poses;
  std::vector<int> goalsVisited; // -1 if unvisited, index of agent that first visits each goal
  std::vector<bool> activeAgents; // true if active, false otherwise
  int iteration;
} EnvXYHashEntry_t;


typedef struct
{
    int startstateid;
    int goalstateid;
    bool bInitialized;
    //any additional variables
} Environment_xy_t;

typedef struct
{
  unsigned char aind; //index of the action (unique for given starttheta)                                                                                                                                  
  char starttheta;
  char dX;
  char dY;
  char endtheta;
  unsigned int cost;
  std::vector<sbpl_2Dcell_t> intersectingcellsV;
  //start at 0,0,starttheta and end at endcell in continuous domain 
  //with half-bin less to account for 0,0 start
  std::vector<sbpl_xy_theta_pt_t> intermptV;
  //start at 0,0,starttheta and end at endcell in discrete domain
  std::vector<sbpl_xy_theta_cell_t> interm3DcellsV;
} EnvXYAction_t;

/*
typedef struct
{
int motprimID;
unsigned char starttheta_c;
sbpl_xy_theta_cell_t endcell;
//intermptV start at 0,0,starttheta and end at endcell in continuous      
//domain with half-bin less to account for 0,0 start                                             
std::vector<sbpl_xy_theta_pt_t> intermptV;
} SBPL_xytheta_mprimitive;
*/

typedef struct
{
  int actionwidth;  
  std::vector<sbpl_2Dpt_t> FootprintPolygon;
  std::vector<SBPL_xytheta_mprimitive> mprimV;
  EnvXYAction_t** ActionsV;
  std::vector<EnvXYAction_t*>* PredActionsV;
}RobotConfig_t;

typedef struct ENV_XY_CONFIG
{
  int numAgents;
  int numGoals;
  int EnvWidth_c;
  int EnvHeight_c;
  int NumThetaDirs;
  std::vector<pose_disc_t> start;
  std::vector<pose_disc_t> goal;
  std::vector<RobotConfig_t> robotConfigV;
  unsigned char** Grid2D;  
  unsigned char obsthresh;
  double cellsize_m;
  double time_per_action;
  int goaltol_x;
  int goaltol_y;
  char goaltol_theta; // currently unused
}EnvXYConfig_t;

typedef struct VIZ_CONFIG
{
double costmap_originX;
double costmap_originY;
}VizConfig_t;

/*
class EnvXY_InitParms
{
public:
  unsigned int numAgents;
  const unsigned char* mapdata;
  std::vector<pose_disc_t> start;
  std::vector<pose_disc_t> goal;
  double goaltol_x;
  double goaltol_y;
  double goaltol_theta;
};*/


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

  virtual bool InitializeEnv(int width, int height, const unsigned char* mapdata, 
			     int numagents, //int numgoals,
			     //std::vector<pose_cont_t> start, std::vector<pose_cont_t> goal,
			     double goaltol_x, double goaltol_y, double goaltol_theta,
			     const std::vector<std::vector<sbpl_2Dpt_t> > & perimeterptsV,
			     double cellsize_m, double time_per_action,
			     const std::vector<std::string> sMotPrimFiles,
			     double costmapOriginX, double costmapOriginY);

  void InitializeAgentConfig(int agentID, std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);

  bool InitGeneral(std::vector<std::vector<SBPL_xytheta_mprimitive> >*
				   motionprimitiveV);

  void SetConfiguration(int width, int height, const unsigned char* mapdata,
			//std::vector<pose_disc_t> start, std::vector<pose_disc_t> goal,
			int numagents,
			double cellsize_m, double time_per_action,
			const std::vector<std::vector<sbpl_2Dpt_t> >& robot_perimeterV,
			double goaltol_x, double goaltol_y, double goaltol_theta);

  bool ReadMotionPrimitive_agent(FILE* fMotPrims, int agentId);
  bool ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim,
			     FILE* fIn);
  virtual bool ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn);
  virtual bool ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn);
  void PrecomputeActionswithCompleteMotionPrimitive(int agent_i,
						    std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV);

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
    
    void GetRobotFootprint(int agentId, pose_cont_t pose, 
			   std::vector<sbpl_2Dpt_t>& FootprintPolygon) const;

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetGoalHeuristic(int stateID);

    virtual bool IsValidCell(int X, int Y) const;    

    virtual bool IsValidConfiguration(std::vector<pose_disc_t> pos) const;
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

    void GetCoordFromState(int stateID, std::vector<pose_disc_t>& poses, std::vector<int>& goalsVisited, std::vector<bool>& activeAgents) const;

    virtual int GetStateFromCoord(std::vector<pose_disc_t>& poses, std::vector<int> goalsVisited,
				  std::vector<bool> activeAgents);
    /**
     * \brief sets start in meters/radians
     */
    virtual int SetStart(std::vector<pose_cont_t> start);

    bool SetMap(const unsigned char* mapdata);

    /**
     * \brief sets goal in meters/radians
     */
    virtual int SetGoal(std::vector<pose_cont_t> goal);
    
    virtual bool GetFakePlan(int startstateID, std::vector<int>& solutionstateIDs);
   
    /**
     * \brief gets number of goals
     */
    virtual int GetNumGoals() const;

    /**
     * \brief gets number of agents
     */
    virtual int GetNumAgents() const;

    virtual bool isGoal(int id) const;
    virtual bool isAGoal(const pose_disc_t& pose) const;
    
    virtual bool isStart(int id);

    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);
    virtual void GetSuccsForAgent(int agentID, pose_disc_t pose, std::vector<pose_disc_t>& newPosesV,
				  std::vector<int>& costV) const;
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost);

    virtual void getGoalsVisited(const std::vector<pose_disc_t>& poses, std::vector<int>& goalsVisited) const;

    virtual unsigned int GETHASHBIN(std::vector<pose_disc_t> pose, std::vector<int> goalsVisited, std::vector<bool> activeAgents);
    
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);
    void VisualizeState(int stateID) const;
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
    virtual bool PoseContToDisc(double px, double py, double pz, double pth, 
				int &ix, int &iy, int &iz, int &ith) const;

    /** \brief Transform grid indices into a continuous pose. The computed
     *         angle lies within 0<=pth<2pi.
     *
     * \note Even if this method returns false, you can still use the
     *      computed indices, for example to figure out poses that lie
     *      outside of your current map.
     *
     * \return true if all the indices are within grid bounds.
     */
    virtual bool PoseDiscToCont(int ix, int iy, int iz, int ith, 
				double &px, double &py, double &pz, double &pth) const;
    virtual int SizeofCreatedEnv();

 protected:
    ros::Publisher state_pub_;
    // member data
    EnvXYConfig_t EnvXYCfg;
    Environment_xy_t EnvXY;
    VizConfig_t VizCfg; 
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

    virtual EnvXYHashEntry_t* GetHashEntry_hash(std::vector<pose_disc_t>& pose, 
						std::vector<int>& goalsVisited, 
						std::vector<bool> activeAgents);
    virtual EnvXYHashEntry_t* CreateNewHashEntry_hash(std::vector<pose_disc_t>& pose, 
						      std::vector<int>& goalsVisited, 
						      std::vector<bool> activeAgents);
    virtual bool IsEqualHashEntry(EnvXYHashEntry_t* hashentry, 
				  std::vector<pose_disc_t>& poses, 
				  std::vector<int>& GoalsVisited, 
				  std::vector<bool> activeAgents) const;
    
    //pointers to functions
    EnvXYHashEntry_t* (Environment_xy::*GetHashEntry)(std::vector<pose_disc_t>& pose, 
						      std::vector<int>& goalsVisited, 
						      std::vector<bool> activeAgents);
    EnvXYHashEntry_t* (Environment_xy::*CreateNewHashEntry)(std::vector<pose_disc_t>& pose,
							    std::vector<int>& goalsVisited,
							    std::vector<bool> activeAgents);
    
};

#endif
