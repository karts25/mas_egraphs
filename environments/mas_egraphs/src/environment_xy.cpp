#include <cmath>
#include <cstring>
#include <ctime>
#include <mas_egraphs/environment_xy.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/utils/key.h>

using namespace std;

/*
  #ifndef DEBUG_ENV
  #define DEBUG_ENV
  #endif
*/

#ifndef VIZ_EXPANSIONS
#define VIZ_EXPANSIONS
#endif

/*
  #ifndef DEBUG
  #define DEBUG
  #endif
*/

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
    EnvXYCfg.robotConfigV.resize(1);
    EnvXYCfg.robotConfigV[0].actionwidth = DEFAULTACTIONWIDTH;
    GetSuccsForAgentClock = 0;
    GetSuccsClock = 0;
    GetSuccsPruningClock = 0;
    CreateHashEntryClock = 0;
    GetLazySuccsWithUniqueIdsClock = 0;
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
unsigned int Environment_xy::GETHASHBIN(std::vector<pose_disc_t> poses, 
                                        std::vector<int> goalsVisited, 
                                        std::vector<bool> activeAgents)
{
    unsigned int hashbin = 0;
    for(int i = 0; i < EnvXYCfg.numAgents; ++i)
        hashbin += inthash(inthash(poses[i].x) + (inthash(poses[i].y) << 1) + 
                           (inthash(poses[i].z) << 2) + (inthash(poses[i].theta << 3)));
    
    for(int j = 0; j < EnvXYCfg.numGoals; j++)
        hashbin += inthash(goalsVisited[j]);
    return inthash(hashbin) & (HashTableSize - 1);
}

bool Environment_xy::InitializeEnv()
{
    ros::NodeHandle nh;
    //initialize the map from Coord to StateID
    HashTableSize = 4 * 1024 * 1024; //should be power of two
    Coord2StateIDHashTable = new vector<EnvXYHashEntry_t*> [HashTableSize];
    GetHashEntry = &Environment_xy::GetHashEntry_hash;
    CreateNewHashEntry = &Environment_xy::CreateNewHashEntry_hash;
    
    //initialize the map from StateID to Coord
    StateID2CoordTable.clear();
    
    //initialize publisher to visualize states
    state_pub_ = nh.advertise<visualization_msgs::MarkerArray>("mas_egraphs/expandedstates", 1);
    //initialized
    EnvXY.bInitialized = true;
    return true;
}


void Environment_xy::InitializeAgentConfig(int agentID, 
                                           std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
    //aditional to configuration file initialization of EnvXYCfg if necessary              
    /*
    //dXY dirs                                                                               
    EnvXYCfg.dXY[0][0] = -1;
    EnvXYCfg.dXY[0][1] = -1;
    EnvXYCfg.dXY[1][0] = -1;
    EnvXYCfg.dXY[1][1] = 0;
    EnvXYCfg.dXY[2][0] = -1;
    EnvXYCfg.dXY[2][1] = 1;
    EnvXYCfg.dXY[3][0] = 0;
    EnvXYCfg.dXY[3][1] = -1;
    EnvXYCfg.dXY[4][0] = 0;
    EnvXYCfg.dXY[4][1] = 1;
    EnvXYCfg.dXY[5][0] = 1;
    EnvXYCfg.dXY[5][1] = -1;
    EnvXYCfg.dXY[6][0] = 1;
    EnvXYCfg.dXY[6][1] = 0;
    EnvXYCfg.dXY[7][0] = 1;
    EnvXYCfg.dXY[7][1] = 1;
    */
    sbpl_xy_theta_pt_t temppose;
    temppose.x = 0.0;
    temppose.y = 0.0;
    temppose.theta = 0.0;

    /*  
        #ifdef DEBUG_ENV
        std::vector<sbpl_2Dcell_t> footprint;
        get_2d_footprint_cells(EnvXYCfg.robotConfigV[agentID].FootprintPolygon, 
        &footprint, temppose, EnvXYCfg.cellsize_m);
  
        for (vector<sbpl_2Dcell_t>::iterator it = footprint.begin(); it != footprint.end(); ++it) {
        SBPL_INFO("Footprint cell at (%d, %d)\n", it->x, it->y);
        }
        #endif
    */
    PrecomputeActionswithCompleteMotionPrimitive(agentID, motionprimitiveV);
}

bool Environment_xy::InitializeEnv(int agentID, int width, int height, const unsigned char* mapdata,
                                   int numagents,// int numgoals,
                                   double goaltol_x, double goaltol_y, double goaltol_theta,
                                   const std::vector<std::vector<sbpl_2Dpt_t> > & perimeterptsV,
                                   double cellsize_m, double time_per_action,
                                   const std::vector<std::string> sMotPrimFiles,
                                   double costmapOriginX, double costmapOriginY,
                                   mas_config::costfunc costfunc){
    SBPL_INFO("env: initialize with width=%d height=%d "
              "cellsize=%.3f timeperaction=%.3f\n",
              width, height, cellsize_m, time_per_action);
    
    VizCfg.costmap_originX = costmapOriginX;
    VizCfg.costmap_originY = costmapOriginY;
    VizCfg.vizmarker_id_ = 0;
    SetConfiguration(agentID, width, height, mapdata, numagents, //start_disc, goal_disc,
                     cellsize_m, time_per_action, perimeterptsV,
                     goaltol_x, goaltol_y, goaltol_theta,
                     costfunc);

    for(int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++){
        FILE* fMotPrim = fopen(sMotPrimFiles[agent_i].c_str(), "r");
        if (fMotPrim == NULL) {
            SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFiles[agent_i].c_str());
            throw new SBPL_Exception();
        }
      
        if (ReadMotionPrimitive_agent(fMotPrim, agent_i) == false) {
            SBPL_ERROR("ERROR: failed to read in motion primitive file for Agent %d\n", agent_i);
            throw new SBPL_Exception();
        }
        fclose(fMotPrim);    
        InitializeAgentConfig(agent_i, &EnvXYCfg.robotConfigV[agent_i].mprimV);
    }
    InitializeEnv();
    return true;
}


bool Environment_xy::ReadMotionPrimitive_agent(FILE* fMotPrims, int agentId)
{
    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    //SBPL_INFO("Reading in motion primitives for agent %d", agentId);

    //read in the resolution                                                                          
    strcpy(sExpected, "resolution_m:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0) return false;
    if (fabs(fTemp - EnvXYCfg.cellsize_m) > ERR_EPS) {
        SBPL_ERROR("ERROR: invalid resolution %f (instead of %f) in the dynamics file\n", fTemp,
                   EnvXYCfg.cellsize_m);
        return false;
    }
  
    //read in the angular resolution                                                                                                                                                                         
    strcpy(sExpected, "numberofangles:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &dTemp) == 0) return false;
    /* if (dTemp != EnvXYCfg.NumThetaDirs) {
       SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n",
       dTemp, EnvXYCfg.NumThetaDirs);
       return false;
       }*/
    EnvXYCfg.NumThetaDirs = dTemp;
    //read in the total number of actions                                                                                                                    
    strcpy(sExpected, "totalnumberofprimitives:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0) {
        return false;
    }

    for (int i = 0; i < totalNumofActions; i++) {
        SBPL_xytheta_mprimitive motprim;

        if (Environment_xy::ReadinMotionPrimitive(&motprim, fMotPrims) == false)
            return false;
        EnvXYCfg.robotConfigV[agentId].mprimV.push_back(motprim);
    }
    return true;
}

bool Environment_xy::ReadinMotionPrimitive(SBPL_xytheta_mprimitive* pMotPrim,
                                           FILE* fIn)
{
    char sTemp[1024];
    int dTemp;
    char sExpected[1024];
    int numofIntermPoses;

    //read in actionID                                                                                                               
    strcpy(sExpected, "primID:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &pMotPrim->motprimID) != 1) return false;

    //read in start angle                                                                                                                                                                                    
    strcpy(sExpected, "startangle_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) == 0) {
        SBPL_ERROR("ERROR reading startangle\n");
        return false;
    }
    pMotPrim->starttheta_c = dTemp;

    //read in end pose                                                                                                                                                                                       
    strcpy(sExpected, "endpose_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }

    if (ReadinCell(&pMotPrim->endcell, fIn) == false) {
        SBPL_ERROR("ERROR: failed to read in endsearchpose\n");
        return false;
    }

    //read in action cost                                                                            
    strcpy(sExpected, "additionalactioncostmult:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) != 1) return false;
    pMotPrim->additionalactioncostmult = dTemp;

    //read in intermediate poses                                                                                                                                                                          
    strcpy(sExpected, "intermediateposes:");
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &numofIntermPoses) != 1) return false;
    //all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done                                                                                         
    //after the action is rotated by initial orientation                                                                                                                                                     
    for (int i = 0; i < numofIntermPoses; i++) {
        sbpl_xy_theta_pt_t intermpose;
        if (ReadinPose(&intermpose, fIn) == false) {
            SBPL_ERROR("ERROR: failed to read in intermediate poses\n");
            return false;
        }
        pMotPrim->intermptV.push_back(intermpose);
    }

    //check that the last pose corresponds correctly to the last pose                                 
    sbpl_xy_theta_pt_t sourcepose;
    sourcepose.x = DISCXY2CONT(0, EnvXYCfg.cellsize_m);
    sourcepose.y = DISCXY2CONT(0, EnvXYCfg.cellsize_m);
    sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, EnvXYCfg.NumThetaDirs);
    double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x;
    double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y;
    double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta;
    int endx_c = CONTXY2DISC(mp_endx_m, EnvXYCfg.cellsize_m);
    int endy_c = CONTXY2DISC(mp_endy_m, EnvXYCfg.cellsize_m);
    int endtheta_c = ContTheta2Disc(mp_endtheta_rad, EnvXYCfg.NumThetaDirs);
 
    if (endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y ||
        endtheta_c != pMotPrim->endcell.theta) {
        SBPL_ERROR( "ERROR: incorrect primitive %d with startangle=%d "
                    "last interm point %f %f %f does not match end pose %d %d %d\n",
                    pMotPrim->motprimID, pMotPrim->starttheta_c,
                    pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x,
                    pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y,
                    pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta,
                    pMotPrim->endcell.x, pMotPrim->endcell.y,
                    pMotPrim->endcell.theta);
        return false;
    }
    return true;
}

bool Environment_xy::ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->x = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->y = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->theta = atoi(sTemp);

    //normalize the angle                                                
    cell->theta = NORMALIZEDISCTHETA(cell->theta, EnvXYCfg.NumThetaDirs);

    return true;
}

bool Environment_xy::ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->x = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->y = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    pose->theta = atof(sTemp);

    pose->theta = normalizeAngle(pose->theta);

    return true;
}

//here motionprimitivevector contains actions for all angles                                       
void Environment_xy::PrecomputeActionswithCompleteMotionPrimitive(int agent_i,
                                                                  std::vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
    SBPL_PRINTF("Pre-computing action data using motion primitives for every angle...\n");
    EnvXYCfg.robotConfigV[agent_i].ActionsV = new EnvXYAction_t*[EnvXYCfg.NumThetaDirs];
    EnvXYCfg.robotConfigV[agent_i].PredActionsV = new vector<EnvXYAction_t*> [EnvXYCfg.NumThetaDirs];
    vector<sbpl_2Dcell_t> footprint;

    if (motionprimitiveV->size() % EnvXYCfg.NumThetaDirs != 0) {
        SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
        throw new SBPL_Exception();
    }

    EnvXYCfg.robotConfigV[agent_i].actionwidth = ((int)motionprimitiveV->size()) / EnvXYCfg.NumThetaDirs;

    //iterate over source angles                                                                                               
    int maxnumofactions = 0;
    for (int tind = 0; tind < EnvXYCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, EnvXYCfg.NumThetaDirs);

        EnvXYCfg.robotConfigV[agent_i].ActionsV[tind] = new EnvXYAction_t[EnvXYCfg.robotConfigV[agent_i].actionwidth];

        //compute sourcepose                                                                                                    
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvXYCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvXYCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvXYCfg.NumThetaDirs);
        //iterate over motion primitives                                                               
        int numofactions = 0;
        int aind = -1;

        for (int mind = 0; mind < (int)motionprimitiveV->size(); mind++) {
            //find a motion primitive for this angle                                                                                                                                                         
            if (motionprimitiveV->at(mind).starttheta_c != tind) continue;

            aind++;
            numofactions++;

            //action index                                                                                                                                                                                   
            EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].aind = aind;

            //start angle                                                                                                                                                                                    
            EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].starttheta = tind;

            //compute dislocation                                                                                                                                                                            
            EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
            EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
            EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;

            //compute and store interm points as well as intersecting cells                                                                                                                                  
            EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intersectingcellsV.clear();
            EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intermptV.clear();
            EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].interm3DcellsV.clear();

            sbpl_xy_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.x = 0;
            previnterm3Dcell.y = 0;

            // Compute all the intersected cells for this action (intermptV and interm3DcellsV)                                                                                                              
            for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++) {
                sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
                EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intermptV.push_back(intermpt);

                // also compute the intermediate discrete cells if not there already                                                                                                                         
                sbpl_xy_theta_pt_t pose;
                pose.x = intermpt.x + sourcepose.x;
                pose.y = intermpt.y + sourcepose.y;
                pose.theta = intermpt.theta;

                sbpl_xy_theta_cell_t intermediate2dCell;
                intermediate2dCell.x = CONTXY2DISC(pose.x, EnvXYCfg.cellsize_m);
                intermediate2dCell.y = CONTXY2DISC(pose.y, EnvXYCfg.cellsize_m);

                // add unique cells to the list                                                                                                                                                              
                if (EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].interm3DcellsV.size() == 0 || intermediate2dCell.x
                    != previnterm3Dcell.x || intermediate2dCell.y != previnterm3Dcell.y) {
                    EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].interm3DcellsV.push_back(intermediate2dCell);
                }

                previnterm3Dcell = intermediate2dCell;
            }

            //compute linear and angular time                                                                                                                                                                
            double linear_distance = 0;
            for (unsigned int i = 1; i < EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intermptV.size(); i++) {
                double x0 = EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intermptV[i - 1].x;
                double y0 = EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intermptV[i - 1].y;
                double x1 = EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intermptV[i].x;
                double y1 = EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intermptV[i].y;
                double dx = x1 - x0;
                double dy = y1 - y0;
                linear_distance += sqrt(dx * dx + dy * dy);
            }
            /*
              double linear_time = linear_distance / EnvXYCfg.nominalvel_mpersecs;
              double angular_distance =
              fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(EnvXYCfg.ActionsV[tind][aind].endtheta,
              EnvXYCfg.NumThetaDirs),
              DiscTheta2Cont(EnvXYCfg.ActionsV[tind][aind].starttheta,
              EnvXYCfg.NumThetaDirs)));
              double angular_time = angular_distance / ((PI_CONST / 4.0) /
              EnvXYCfg.timetoturn45degsinplace_secs);
      
              //make the cost the max of the two times                                                                                                                                                         
              EnvXYCfg.ActionsV[tind][aind].cost =
              (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM * max(linear_time, angular_time)));
      
              EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].cost = linear_time;
            */                                                                                                           
            EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].cost = EnvXYCfg.time_per_action;// * motionprimitiveV->at(mind).additionalactioncostmult;
      
            //now compute the intersecting cells for this motion (including ignoring the source footprint)                                                                                                   
            get_2d_motion_cells(EnvXYCfg.robotConfigV[agent_i].FootprintPolygon, 
                                motionprimitiveV->at(mind).intermptV,
                                &EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intersectingcellsV,
                                EnvXYCfg.cellsize_m);
#if DEBUG
            SBPL_FPRINTF(fDeb,
                         "action tind=%2d aind=%2d: dX=%3d dY=%3d endtheta=%3d (%6.2f degs -> %6.2f degs) "
                         "cost=%4d (mprimID %3d: %3d %3d %3d) numofintermcells = %d numofintercells=%d\n",
                         tind,
                         aind,
                         EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].dX,
                         EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].dY,
                         EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].endtheta,
                         EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intermptV[0].theta * 180 / PI_CONST,
                         EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intermptV[EnvXYCfg.ActionsV[tind][aind].intermptV.size() - 1].theta * 180 / PI_CONST, EnvXYCfg.ActionsV[tind][aind].\
                         cost,
                         motionprimitiveV->at(mind).motprimID, motionprimitiveV->at(mind).endcell.x,
                         motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.theta,
                         (int)EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].interm3DcellsV.size(),
                         (int)EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].intersectingcellsV.size());
#endif

            //add to the list of backward actions                                                        
            int targettheta = EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind].endtheta;
            if (targettheta < 0) 
                targettheta = targettheta + EnvXYCfg.NumThetaDirs;
            EnvXYCfg.robotConfigV[agent_i].PredActionsV[targettheta].push_back(&(EnvXYCfg.robotConfigV[agent_i].ActionsV[tind][aind]));
        }

        if (maxnumofactions < numofactions) maxnumofactions = numofactions;
    }
    //at this point we don't allow nonuniform number of actions                                                                                                                                              
    if (motionprimitiveV->size() != (size_t)(EnvXYCfg.NumThetaDirs * maxnumofactions)) {
        SBPL_ERROR("ERROR: nonuniform number of actions is not supported "
                   "(maxnumofactions=%d while motprims=%d thetas=%d\n",
                   maxnumofactions, (unsigned int)motionprimitiveV->size(), EnvXYCfg.NumThetaDirs);
        throw new SBPL_Exception();
    }
  
    //now compute replanning data    
    //ComputeReplanningData();
  
    SBPL_PRINTF("done pre-computing action data based on motion primitives\n");
}
     
  
bool Environment_xy::PoseContToDisc(const double px, const double py, const double pz, 
                                    const double pth,
                                    int &ix, int &iy, int &iz, int &ith) const
{
    ix = CONTXY2DISC(px, EnvXYCfg.cellsize_m);
    iy = CONTXY2DISC(py, EnvXYCfg.cellsize_m);
    iz = CONTXY2DISC(pz, EnvXYCfg.cellsize_m);; // todo
    ith = ContTheta2Disc(pth, EnvXYCfg.NumThetaDirs);
    return (ix >= 0) && (ix < EnvXYCfg.EnvWidth_c) &&
        (iy >= 0) && (iy < EnvXYCfg.EnvHeight_c) &&
        (iz >= 0) && (pth >= -2 * PI_CONST) && (pth <= 2 * PI_CONST);
}

bool Environment_xy::PoseDiscToCont(const int ix, const int iy, const int iz, const int ith, 
                                    double &px, double &py, double &pz, double &pth) const
{
    px = DISCXY2CONT(ix, EnvXYCfg.cellsize_m);
    py = DISCXY2CONT(iy, EnvXYCfg.cellsize_m);
    pz = DISCXY2CONT(iz, EnvXYCfg.cellsize_m); // todo
    pth = normalizeAngle(DiscTheta2Cont(ith, EnvXYCfg.NumThetaDirs));
    return (ith >= 0) && (ith < EnvXYCfg.NumThetaDirs) && (ix >= 0) &&
        (ix < EnvXYCfg.EnvWidth_c) && (iy >= 0) && (iy < EnvXYCfg.EnvHeight_c) && (iz > 0);
}

unsigned char Environment_xy::GetMapCost(int x, int y) const
{
    return EnvXYCfg.Grid2D[x][y];
}

void Environment_xy::SetConfiguration(int agentID, int width, int height, 
                                      const unsigned char* mapdata,	    
                                      int numagents,
                                      double cellsize_m, double time_per_action,
                                      const std::vector<std::vector<sbpl_2Dpt_t> >& robot_perimeterV,
                                      double goaltol_x, double goaltol_y, double goaltol_theta,
                                      mas_config::costfunc costfunc){
    EnvXYCfg.agentID = agentID;
    EnvXYCfg.EnvWidth_c = width;
    EnvXYCfg.EnvHeight_c = height;
    EnvXYCfg.numAgents = numagents;
    EnvXYCfg.time_per_action = time_per_action;
    EnvXYCfg.cellsize_m = cellsize_m;
    EnvXYCfg.obsthresh = ENVXY_DEFAULTOBSTHRESH;
    EnvXYCfg.costfunc = costfunc;
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

    // crete vector of robot configuration params
    EnvXYCfg.robotConfigV.resize(EnvXYCfg.numAgents);
    for(int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++){
        EnvXYCfg.robotConfigV[agent_i].FootprintPolygon = robot_perimeterV[agent_i];
    }
    
    // set goal tolerances
    EnvXYCfg.goaltol_x = 1 + CONTXY2DISC(goaltol_x, EnvXYCfg.cellsize_m);
    EnvXYCfg.goaltol_y = 1 + CONTXY2DISC(goaltol_y, EnvXYCfg.cellsize_m);
    //printf("Goal tolerances set to (%f, %f) = (%d, %d)\n", 
    //goaltol_x, goaltol_y,
    //EnvXYCfg.goaltol_x, EnvXYCfg.goaltol_y);
}

bool Environment_xy::UpdateCost(int x, int y, unsigned char newcost)
{
    //printf("Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,EnvXYCfg.Grid2D[x][y], newcost);
  
    EnvXYCfg.Grid2D[x][y] = newcost;
    return true;
}


// -------------- unimplemented functions

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

void Environment_xy::GetCoordFromState(int stateID, vector<pose_disc_t>& poses, 
                                       vector<int>& goalsVisited,
                                       vector<bool>& activeAgents) const
{
    EnvXYHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    poses = HashEntry->poses;
    goalsVisited = HashEntry->goalsVisited;
    activeAgents = HashEntry->activeAgents;
}

int Environment_xy::GetStateFromCoord(vector<pose_disc_t>& poses, vector<int> goalsVisited,
                                      std::vector<bool> activeAgents) 
{
    EnvXYHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(poses, goalsVisited, activeAgents)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(poses, goalsVisited, activeAgents);
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

int Environment_xy::GetNumActiveAgents(int stateID) const
{
    EnvXYHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    std::vector<bool> activeAgents = HashEntry->activeAgents;
    return std::accumulate(activeAgents.begin(), activeAgents.end(),0);    
}

bool Environment_xy::isActive(const int stateID, const int agentID) const
{
    EnvXYHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    return HashEntry->activeAgents[agentID];
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

bool Environment_xy::isGoal(int id) const {
    // any state with all goals visited is a goal state
    EnvXYHashEntry_t* HashEntry = StateID2CoordTable[id];
    if (std::all_of(HashEntry->goalsVisited.begin(),
                    HashEntry->goalsVisited.end(), 
                    [](int i){return i >= 0;}))
        return true;
    else
        return false;
}

bool Environment_xy::isStart(int id){
    EnvXYHashEntry_t* HashEntry = StateID2CoordTable[id];
    for(int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++){
        if((HashEntry->poses[agent_i].x != EnvXYCfg.start[agent_i].x) ||
           (HashEntry->poses[agent_i].y != EnvXYCfg.start[agent_i].y))
            return false;
    }
    return true;
}

int Environment_xy::SetGoal(std::vector<pose_cont_t> goal_m)
{
    std::vector<pose_disc_t> goal(EnvXYCfg.numGoals);
    int x,y,z,theta;
    for (int goal_i = 0; goal_i < EnvXYCfg.numGoals; goal_i ++)
        {
            PoseContToDisc(goal_m[goal_i].x, goal_m[goal_i].y, goal_m[goal_i].z, goal_m[goal_i].theta,  x, y, z, theta);
            /* SBPL_INFO("env: setting goal to %.3f %.3f %0.3f %0.3f (%d %d %d %d)", 
                      goal_m[goal_i].x, goal_m[goal_i].y, goal_m[goal_i].z, goal_m[goal_i].theta,
                      x, y, z, theta);
            */
            if (!IsWithinMapCell(x, y)) {
                SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x, y);
                return -1;
            }
            goal[goal_i].x = x;
            goal[goal_i].y = y;
            goal[goal_i].z = z;
            goal[goal_i].theta = theta;                  
        }
    
    EnvXYHashEntry_t* OutHashEntry;
    std::vector<int> goalsVisited(EnvXYCfg.numGoals, 0); // anything other than -1 means goal has been visited
    std::vector<bool> activeAgents(EnvXYCfg.numAgents, true);
    if ((OutHashEntry = (this->*GetHashEntry)(goal, goalsVisited, activeAgents)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(goal, goalsVisited, activeAgents);
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


bool Environment_xy::IsValidPlan(const std::vector<int>& solution_stateIDs_V, int step) const{
    for(int i = step; i < (int) solution_stateIDs_V.size(); i++){
        EnvXYHashEntry_t* HashEntry = StateID2CoordTable[solution_stateIDs_V[i]];
        if(!IsValidConfiguration(HashEntry->poses)){
            return false;
        }
    }
    return true;
}

//returns the stateid if success, and -1 otherwise
int Environment_xy::SetStart(const std::vector<pose_cont_t>& start_m, 
                             const std::vector<int>& goalsAlreadyVisited)
{
    std::vector<pose_disc_t> start(EnvXYCfg.numAgents);
    std::vector<int> x(EnvXYCfg.numAgents), y(EnvXYCfg.numAgents);
    int x_agent, y_agent, z_agent, theta_agent;
    for (int agent_i = 0; agent_i < EnvXYCfg.numAgents; ++agent_i){
        PoseContToDisc(start_m[agent_i].x, start_m[agent_i].y, start_m[agent_i].z, 
                       start_m[agent_i].theta,
                       x_agent, y_agent, z_agent, theta_agent);
        /*
        SBPL_INFO("env: setting start of agent %d to %.3f %.3f %0.3f %0.3f (%d %d %d %d)", 
                  agent_i,
                  start_m[agent_i].x,start_m[agent_i].y,start_m[agent_i].z,start_m[agent_i].theta,
                  x_agent, y_agent, z_agent, theta_agent);
        */
        if (!IsWithinMapCell(x_agent, y_agent)) {
            SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x_agent, y_agent);
            return -1;
        }
      
        //SBPL_INFO("env: setting start of Agent %d to %.3f %.3f %.3f %.3f (%d %d %d %d)\n",
        //(agent_i+1), start_m[agent_i].x, start_m[agent_i].y, start_m[agent_i].z,
        //start_m[agent_i].theta,
        //x_agent, y_agent, z_agent, theta_agent);
        start[agent_i].x = x_agent;
        start[agent_i].y = y_agent;
        start[agent_i].z = z_agent;
        start[agent_i].theta = theta_agent;
    }  
  
    if (!IsValidConfiguration(start)) {
        SBPL_ERROR("ERROR: start configuration is invalid\n");
        return -1;
    }
	   
    EnvXYHashEntry_t* OutHashEntry; 
    std::vector<int> goalsVisited = goalsAlreadyVisited;
    getGoalsVisited(start, goalsVisited);
    std::vector<bool> activeAgents(EnvXYCfg.numAgents, true);
    if ((OutHashEntry = (this->*GetHashEntry)(start, goalsVisited, activeAgents)) == NULL) {
        //have to create a new entry
        OutHashEntry = (this->*CreateNewHashEntry)(start, goalsVisited, activeAgents);
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
EnvXYHashEntry_t* Environment_xy::GetHashEntry_hash(std::vector<pose_disc_t>& poses, 
                                                    std::vector<int>& goalsVisited,
                                                    std::vector<bool>& activeAgents)
{
  
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    int binid = GETHASHBIN(poses, goalsVisited, activeAgents);

#if DEBUG
    if ((int)Coord2StateIDHashTable[binid].size() > 5){
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
        bool matchfound = IsEqualHashEntry(hashentry, poses, goalsVisited, activeAgents);
        if (matchfound)
            return hashentry;
    }
    return NULL;
}

bool Environment_xy::IsEqualHashEntry(EnvXYHashEntry_t* hashentry,
                                      std::vector<pose_disc_t>& poses,
                                      std::vector<int>& goalsVisited, 
                                      std::vector<bool>& activeAgents) const
{
    for(int goal_i = 0; goal_i < EnvXYCfg.numGoals; goal_i++){
        int lhs_visited = hashentry->goalsVisited[goal_i];
        int rhs_visited = goalsVisited[goal_i];
        if((lhs_visited == -1) && (rhs_visited != -1))
            return false;
        if((rhs_visited != -1) && (rhs_visited == -1))
            return false;
    }

    if(hashentry->activeAgents != activeAgents)
        return false;
    for (int agent_i =0; agent_i <  EnvXYCfg.numAgents; agent_i++){
        if ((hashentry->poses[agent_i].x != poses[agent_i].x)
            || (hashentry->poses[agent_i].y != poses[agent_i].y)) 
            return false;
    }
    return true;
}


EnvXYHashEntry_t* Environment_xy::CreateNewHashEntry_hash(std::vector<pose_disc_t>& poses,
                                                          std::vector<int>& goalsVisited,
                                                          std::vector<bool>& activeAgents)
{
    int i;

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    EnvXYHashEntry_t* HashEntry = new EnvXYHashEntry_t;

    HashEntry->poses = poses;
    HashEntry->goalsVisited = goalsVisited;
    HashEntry->activeAgents = activeAgents;
    HashEntry->iteration = 0;

    HashEntry->stateID = StateID2CoordTable.size();
   
    //insert into the tables
    StateID2CoordTable.push_back(HashEntry);
    //get the hash table bin
    i = GETHASHBIN(HashEntry->poses, HashEntry->goalsVisited, HashEntry->activeAgents);

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

void Environment_xy::GetLazySuccsWithUniqueIds(int SourceStateID,
                                               std::vector<int>* SuccIDV, std::vector<int>* CostV,
                                               std::vector<bool>* isTrueCost){ 
    clock_t GetLazySuccsWithUniqueIds_t0 = clock();
    GetSuccs(SourceStateID, SuccIDV, CostV);
    isTrueCost->clear();
    isTrueCost->resize((int)SuccIDV->size(), true);
    GetLazySuccsWithUniqueIdsClock += clock() - GetLazySuccsWithUniqueIds_t0;
}


void Environment_xy::GetSuccsWithUniqueIds(int SourceStateID, 
                                           std::vector<int>* SuccIDV, 
                                           std::vector<int>* CostV){
    GetSuccs(SourceStateID, SuccIDV, CostV);
}


void Environment_xy::GetSuccs(int SourceStateID,
                              std::vector<int>* SuccIDV,
                              std::vector<int>* CostV)
{

    clock_t GetSuccs_t0 = clock();
#ifdef DEBUG_ENV
    printf("\n[Environment]: In GetSuccs\n");
#endif
    //get X, Y for the state
    EnvXYHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
    // goal state should be absorbing
    if (std::all_of(HashEntry->goalsVisited.begin(),
                    HashEntry->goalsVisited.end(), 
                    [](int i){return i >= 0;}))
        return;
  
#ifdef VIZ_EXPANSIONS
    VisualizeState(SourceStateID);
#endif

    // We want only as many active agents as there are unvisited goals in the parent state
    int numgoals_unvisited_parent = std::count(HashEntry->goalsVisited.begin(), 
                                               HashEntry->goalsVisited.end(), -1);
    
    // find number and indices of active agents
    std::vector<int> activeAgents_indices;
    for(int i = 0; i < EnvXYCfg.numAgents; i++)
        if(HashEntry->activeAgents[i]){
            activeAgents_indices.push_back(i);
        }
    int numActiveAgents = activeAgents_indices.size();

    // Make numActiveAgents x numActions structure of poses and costs
    std::vector<std::vector<pose_disc_t> > allnewPoses(numActiveAgents);
    std::vector<std::vector<int> > allnewCosts(numActiveAgents);
    int cost;

    //iterate through active agents
    int numPrimitives = 1;
    int activeagent_i = 0;
    for(int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++){
        if (!HashEntry->activeAgents[agent_i])
            continue;
        GetSuccsForAgent(SourceStateID, agent_i, HashEntry->poses[agent_i],
                         allnewPoses[activeagent_i], 
                         allnewCosts[activeagent_i]);
        activeagent_i++;
        numPrimitives*= (EnvXYCfg.robotConfigV[agent_i].actionwidth + 1);
    }
    //ROS_INFO("Numprimitives = %d", numPrimitives);
    EnvXYHashEntry_t* OutHashEntry;
    std::vector<pose_disc_t> poses(EnvXYCfg.numAgents);
    std::vector<bool> activeAgents(EnvXYCfg.numAgents, false);


    // Make successors from all possible combinations of active agents and actions
    // We have [agent1.actionwidth*agent2.actionwidth ....]  primitives
    //clear the successor array
    SuccIDV->clear();
    CostV->clear();    
    SuccIDV->reserve(numPrimitives);
    CostV->reserve(numPrimitives);
    for(int primitive_i = 0; primitive_i < numPrimitives; primitive_i++){
        poses = HashEntry->poses;
        activeAgents = HashEntry->activeAgents;
        // action index is primitive_i written in base agent_ctr.actionwidth
        int index = primitive_i;
        cost = 0;
        for(int agent_ctr = 0; agent_ctr < numActiveAgents; agent_ctr++){
            int numActions = EnvXYCfg.robotConfigV[agent_ctr].actionwidth+1;
            int action_i = index % numActions;
            index = (int) index/numActions;	
            switch(EnvXYCfg.costfunc)
                {
                case mas_config::MAX:
                    cost = std::max(cost, allnewCosts[agent_ctr][action_i]);
                    break;
                case mas_config::SUM:
                    cost += allnewCosts[agent_ctr][action_i];
                    if(cost < 0)
                        ROS_INFO("Switch statement: allnewcosts of agent %d, action %d = %d, cost is %d",
                                 agent_ctr, action_i,
                                 allnewCosts[agent_ctr][action_i],
                                 cost);
                    break;
                }
            int agent_index = activeAgents_indices[agent_ctr];
            poses[agent_index] = allnewPoses[agent_ctr][action_i];
            if (action_i == numActions-1){ 
                // last action is always to retire agent
                activeAgents[agent_index] = 0;
            }
        }
        clock_t GetSuccsPruning_t0 = clock();
        if (cost >= mas_config::MAS_INFINITECOST)
            continue;
  
        int numActiveAgents_successor = std::accumulate(activeAgents.begin(),
                                                        activeAgents.end(),
                                                        0);
        // don't want all robots retired as an action
        if(numActiveAgents_successor == 0)
            continue;
   
        // don't want more agents active than the number of goals unvisited by parent
        // Note: we need to compare against parent since "retiring" is an action like any other 
        // primitive. This means that if the agent reaches a goal in a successor state of 
        // sourcestateID, it can only retire at the next successor state. 
        if (numgoals_unvisited_parent < numActiveAgents_successor)
            continue;

        clock_t GetSuccsPruning_t1 = clock();
        GetSuccsPruningClock += GetSuccsPruning_t1 - GetSuccsPruning_t0;
        clock_t CreateHashEntry_t0 = clock();
        
        std::vector<int> goalsVisited = HashEntry->goalsVisited;
        getGoalsVisited(poses, goalsVisited);
    
        if ((OutHashEntry = (this->*GetHashEntry)(poses, goalsVisited, activeAgents)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(poses, goalsVisited, activeAgents);
        }
        if(cost <= 0)
            ROS_INFO("2. cost is %d", cost);

    
        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        clock_t CreateHashEntry_t1 = clock();
        CreateHashEntryClock += CreateHashEntry_t1 - CreateHashEntry_t0;
#ifdef DEBUG_ENV    
        PrintState(OutHashEntry->stateID, true);    
#endif
    }
    clock_t GetSuccs_t1 = clock();
    GetSuccsClock += GetSuccs_t1 - GetSuccs_t0;
}

void Environment_xy::GetSuccsForAgent(int SourceStateID, int agentID, pose_disc_t pose, 
                                      std::vector<pose_disc_t>& newPosesV,			    
                                      std::vector<int>& costV){
    clock_t GetSuccsForAgent_t0 = clock();
    RobotConfig_t robotConfig = EnvXYCfg.robotConfigV[agentID];
    costV.clear();
    costV.reserve(robotConfig.actionwidth + 1);
    newPosesV.clear();
    newPosesV.reserve(robotConfig.actionwidth + 1);
    int cost;

    for(int action_i = 0; action_i < robotConfig.actionwidth; action_i++){
        pose_disc_t newPose = pose;    
        EnvXYAction_t* action = &EnvXYCfg.robotConfigV[agentID].ActionsV[pose.theta][action_i];
        newPose.x = pose.x + action->dX;
        newPose.y = pose.y + action->dY;
        newPose.z = pose.z; // assume planar movement
        newPose.theta = NORMALIZEDISCTHETA(action->endtheta, EnvXYCfg.NumThetaDirs);
        if(!IsValidPose(agentID, newPose))
            cost = mas_config::MAS_INFINITECOST;
        else
            cost = action->cost;
    
        newPosesV.push_back(newPose);
        costV.push_back(cost);   
    }
    // allow agent to retire with 0 cost if agent is at a goal or state is the starting state
    if((SourceStateID == EnvXY.startstateid) || isAGoal(pose)){
        cost = 0;
    }
    else
        cost = mas_config::MAS_INFINITECOST;
    newPosesV.push_back(pose);
    costV.push_back(cost);

    clock_t GetSuccsForAgent_t1 = clock();
    GetSuccsForAgentClock += GetSuccsForAgent_t1 - GetSuccsForAgent_t0;
}

void Environment_xy::getGoalsVisited(const std::vector<pose_disc_t>& poses,
                                     std::vector<int>& goalsVisited) const
{
    for(int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++){
        int x = poses[agent_i].x;
        int y = poses[agent_i].y;
        for(int goal_i = 0; goal_i < EnvXYCfg.numGoals; goal_i++){
            // if goal is unvisited so far, and a robot is at the goal, record its index
            if((goalsVisited[goal_i] == -1) && (abs(x - EnvXYCfg.goal[goal_i].x) <= EnvXYCfg.goaltol_x)
               && (abs(y - EnvXYCfg.goal[goal_i].y) <= EnvXYCfg.goaltol_y))
                goalsVisited[goal_i] = agent_i;
        }
    }
}

bool Environment_xy::isAGoal(const pose_disc_t &pose) const
{
    for(int goal_i = 0; goal_i < EnvXYCfg.numGoals; goal_i++){
        if((abs(pose.x - EnvXYCfg.goal[goal_i].x) <= EnvXYCfg.goaltol_x) && 
           (abs(pose.y - EnvXYCfg.goal[goal_i].y) <= EnvXYCfg.goaltol_y))
            return true;
    }
    return false;
}

bool Environment_xy::IsValidCell(int X, int Y) const
{
    return (X >= 0 && X < EnvXYCfg.EnvWidth_c && Y >= 0 && Y < EnvXYCfg.EnvHeight_c &&
            EnvXYCfg.Grid2D[X][Y] < EnvXYCfg.obsthresh);
}

bool Environment_xy::IsValidPose(int agent_i, const pose_disc_t& pos) const
{
    std::vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;
    //compute continuous pose                                                        
    pose.x = DISCXY2CONT(pos.x, EnvXYCfg.cellsize_m);
    pose.y = DISCXY2CONT(pos.y, EnvXYCfg.cellsize_m);
    pose.theta = DiscTheta2Cont(pos.theta, EnvXYCfg.NumThetaDirs);

    // compute footprint cells
    get_2d_footprint_cells(EnvXYCfg.robotConfigV[agent_i].FootprintPolygon, &footprint, 
                           pose, EnvXYCfg.cellsize_m);
    // iterate over all footprint cells
    for(int find =0; find < (int)footprint.size(); find++){
        int x = footprint.at(find).x;
        int y = footprint.at(find).y;
        if(!IsValidCell(x,y)){
            return false;
        }
    }

    return true;
}

bool Environment_xy::IsValidConfiguration(const std::vector<pose_disc_t>& pos) const
{
    // collision check poses. TODO: Use footprint
    for(int agent_i = 0; agent_i < EnvXYCfg.numAgents; agent_i++){
        for(int agent2_i = agent_i+1; agent2_i < EnvXYCfg.numAgents; agent2_i++){
            if ((pos[agent_i].x == pos[agent2_i].x) && 
                (pos[agent_i].y == pos[agent2_i].y) && 
                (pos[agent_i].z == pos[agent2_i].z))
                {
                    SBPL_WARN("Robots are in collision");
                    return false;
                } 
        }
        if(!IsValidPose(agent_i, pos[agent_i]))
            {
                return false;
            }
    }
    return true;
}
    
void Environment_xy::GetRobotFootprint(int agentId, pose_cont_t pose,
                                       std::vector<sbpl_2Dpt_t>& FootprintPolygon) const{
    std::vector<sbpl_2Dpt_t> FootprintPolygon_zero = EnvXYCfg.robotConfigV[agentId].FootprintPolygon;
    FootprintPolygon = EnvXYCfg.robotConfigV[agentId].FootprintPolygon;
    for(unsigned int i =0; i < FootprintPolygon.size(); i++){
        // rotate points around pose.theta
        FootprintPolygon[i].x = FootprintPolygon_zero[i].x*cos(pose.theta) - FootprintPolygon_zero[i].y*sin(pose.theta);
        FootprintPolygon[i].y = FootprintPolygon_zero[i].x*sin(pose.theta) + FootprintPolygon_zero[i].y*cos(pose.theta);
        // add offset
        FootprintPolygon[i].x += pose.x;
        FootprintPolygon[i].y += pose.y;
    }
}

int Environment_xy::SizeofCreatedEnv()
{
    return (int)StateID2CoordTable.size();
}

int Environment_xy::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    SBPL_INFO("TODO: GetStartHeuristic not defined");
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

void Environment_xy::VisualizeState(const int stateID) {

    if(VizCfg.vizmarker_id_ > 5000){
        //printf("Hit a key to continue search\n");
        //std::cin.get();
        VizCfg.vizmarker_id_ = 0;
    }
    EnvXYHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    visualization_msgs::MarkerArray agentmarkers;
    for(int agent_i=0; agent_i < EnvXYCfg.numAgents; agent_i++){
        visualization_msgs::Marker marker;
        pose_disc_t pose = HashEntry->poses[agent_i];
        //vizmarker_id_ = stateID;
        marker.id = VizCfg.vizmarker_id_; //stateID*EnvXYCfg.numAgents + agent_i; 
        VizCfg.vizmarker_id_++;
        marker.ns = std::to_string(agent_i);
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0;
        marker.color.g = 1;
        marker.color.b = 1;
        if(!HashEntry->activeAgents[agent_i])
            marker.color.r = 1;
        marker.color.a = 1;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "/map";//costmap_ros_->getGlobalFrameID();
        double x,y,z,theta;
        PoseDiscToCont(pose.x, pose.y, pose.z, pose.theta, x, y, z, theta);
        marker.pose.position.x = x - VizCfg.costmap_originX;
        marker.pose.position.y = y - VizCfg.costmap_originY;
        marker.pose.position.z = z;
        /*SBPL_INFO("expanding (%d, %d, %d, %d), (%f, %f, %f, %f) ",
	      pose.x, pose.y, pose.z, pose.theta,
	      x,y,z,theta);
        */
        tf::Quaternion temp;
        temp.setEulerZYX(theta, 0, 0);
        marker.pose.orientation.x = temp.getX();
        marker.pose.orientation.y = temp.getY();
        marker.pose.orientation.z = temp.getZ();
        marker.pose.orientation.w = temp.getW();
        agentmarkers.markers.push_back(marker);
    }
    state_pub_.publish(agentmarkers);
}

void Environment_xy::PrintTimingStats(){
    printf("Environment Timing Stats:\n");
    printf("GetSuccs:                    %.2f\n", double(GetSuccsClock)/CLOCKS_PER_SEC);
    printf("GetLazySuccsWithUniqueIds:   %.2f\n",
           double(GetLazySuccsWithUniqueIdsClock)/CLOCKS_PER_SEC);
    printf("GetSuccsForAgent:            %.2f\n", double(GetSuccsForAgentClock)/CLOCKS_PER_SEC);
    printf("GetSuccsPruning:             %.2f\n", double(GetSuccsPruningClock)/CLOCKS_PER_SEC);
    printf("CreateHashEntry:             %.2f\n", double(CreateHashEntryClock)/CLOCKS_PER_SEC);
}

void Environment_xy::PrintState(const int stateID, bool bVerbose, FILE* fOut /*=NULL*/) {
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
    printf("State id: %d\n", stateID);
    for(int i = 0; i < EnvXYCfg.numAgents; i++){
        printf("Agent %02d (X=%03d Y=%03d Z=%03d Theta=%02d) isActive: %d\n", i, 
               (int) HashEntry->poses[i].x, (int) HashEntry->poses[i].y, 
               (int) HashEntry->poses[i].z,
               (int) HashEntry->poses[i].theta, (int) HashEntry->activeAgents[i]);
    }
    printf("Goals Visited: ");
    for(int i = 0; i < (int) HashEntry->goalsVisited.size(); i++)
        printf("%d ", (int) HashEntry->goalsVisited[i]);
    printf("\n\n");
}
