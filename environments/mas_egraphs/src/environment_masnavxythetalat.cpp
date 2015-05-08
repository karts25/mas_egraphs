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

#include <cmath>
#include <cstring>
#include <ctime>
#include <mas_egraphs/environment_masnavxythetalat.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

using namespace std;

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

static long int checks = 0;

#define XYTHETA2INDEX(X,Y,THETA) (THETA + X*EnvMASNAVXYTHETALATCfg.NumThetaDirs + \
                                  Y*EnvMASNAVXYTHETALATCfg.EnvWidth_c*EnvMASNAVXYTHETALATCfg.NumThetaDirs)

//-----------------constructors/destructors-------------------------------


EnvironmentMASNAVXYTHETALAT::EnvironmentMASNAVXYTHETALAT()
{
  EnvMASNAVXYTHETALATCfg.NumAgents = 1;
    EnvMASNAVXYTHETALATCfg.obsthresh = ENVMASNAVXYTHETALAT_DEFAULTOBSTHRESH;
    //the value that pretty much makes it disabled
    EnvMASNAVXYTHETALATCfg.cost_inscribed_thresh = EnvMASNAVXYTHETALATCfg.obsthresh; 
    //the value that pretty much makes it disabled
    EnvMASNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = -1; 

    grid2Dsearchfromstart = NULL;
    grid2Dsearchfromgoal = NULL;
    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;
    iteration = 0;

    EnvMASNAVXYTHETALAT.bInitialized = false;

    EnvMASNAVXYTHETALATCfg.actionwidth = MASNAVXYTHETALAT_DEFAULT_ACTIONWIDTH;

    EnvMASNAVXYTHETALATCfg.NumThetaDirs = MASNAVXYTHETALAT_THETADIRS;

    //no memory allocated in cfg yet
    EnvMASNAVXYTHETALATCfg.Grid2D = NULL;
    EnvMASNAVXYTHETALATCfg.ActionsV = NULL;
    EnvMASNAVXYTHETALATCfg.PredActionsV = NULL;

    HashTableSize = 0;
    Coord2StateIDHashTable = NULL;
    Coord2StateIDHashTable_lookup = NULL;
}

EnvironmentMASNAVXYTHETALAT::~EnvironmentMASNAVXYTHETALAT()
{
    SBPL_INFO("destroying XYTHETALATTICE\n");
    if (grid2Dsearchfromstart != NULL) delete grid2Dsearchfromstart;
    grid2Dsearchfromstart = NULL;

    if (grid2Dsearchfromgoal != NULL) delete grid2Dsearchfromgoal;
    grid2Dsearchfromgoal = NULL;

    if (EnvMASNAVXYTHETALATCfg.Grid2D != NULL) {
        for (int x = 0; x < EnvMASNAVXYTHETALATCfg.EnvWidth_c; x++)
            delete[] EnvMASNAVXYTHETALATCfg.Grid2D[x];
        delete[] EnvMASNAVXYTHETALATCfg.Grid2D;
        EnvMASNAVXYTHETALATCfg.Grid2D = NULL;
    }

    //delete actions
    if (EnvMASNAVXYTHETALATCfg.ActionsV != NULL) {
        for (int tind = 0; tind < EnvMASNAVXYTHETALATCfg.NumThetaDirs; tind++)
            delete[] EnvMASNAVXYTHETALATCfg.ActionsV[tind];
        delete[] EnvMASNAVXYTHETALATCfg.ActionsV;
        EnvMASNAVXYTHETALATCfg.ActionsV = NULL;
    }
    if (EnvMASNAVXYTHETALATCfg.PredActionsV != NULL) {
        delete[] EnvMASNAVXYTHETALATCfg.PredActionsV;
        EnvMASNAVXYTHETALATCfg.PredActionsV = NULL;
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
    if (Coord2StateIDHashTable_lookup != NULL) {
        delete[] Coord2StateIDHashTable_lookup;
        Coord2StateIDHashTable_lookup = NULL;
    }
}

//---------------------------------------------------------------------

//-------------------problem specific and local functions---------------------

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

// -------------- unnecessary functions
bool EnvironmentMASNAVXYTHETALAT::InitializeMDPCfg(MDPConfig *MDPCfg){
  return true;}

EnvMASNAVXYTHETALATHashEntry_t* EnvironmentMASNAVXYTHETALAT::GetHashEntry_lookup(std::vector<pose_t> pose, std::vector<bool> goalsVisited){
return NULL;
}

EnvMASNAVXYTHETALATHashEntry_t* EnvironmentMASNAVXYTHETALAT::CreateNewHashEntry_lookup(std::vector<pose_t> pose, std::vector<bool> goalsVisited){
return NULL;
}

void EnvironmentMASNAVXYTHETALAT::ReadConfiguration(FILE* fCfg){
  SBPL_INFO("Undefined");
};


//----------------------------

void EnvironmentMASNAVXYTHETALAT::SetConfiguration(int width, int height, const unsigned char* mapdata,
						       std::vector<pose_t> start, std::vector<pose_t> goal,
						       double cellsize_m, double nominalvel_mpersecs,
						       double timetoturn45degsinplace_secs,
						       const vector<sbpl_2Dpt_t> & robot_perimeterV)
{
    EnvMASNAVXYTHETALATCfg.EnvWidth_c = width;
    EnvMASNAVXYTHETALATCfg.EnvHeight_c = height;
    EnvMASNAVXYTHETALATCfg.start = start;

    EnvMASNAVXYTHETALATCfg.end = goal;

    EnvMASNAVXYTHETALATCfg.FootprintPolygon = robot_perimeterV;

    EnvMASNAVXYTHETALATCfg.nominalvel_mpersecs = nominalvel_mpersecs;
    EnvMASNAVXYTHETALATCfg.cellsize_m = cellsize_m;
    EnvMASNAVXYTHETALATCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;

    //allocate the 2D environment
    EnvMASNAVXYTHETALATCfg.Grid2D = new unsigned char*[EnvMASNAVXYTHETALATCfg.EnvWidth_c];
    for (int x = 0; x < EnvMASNAVXYTHETALATCfg.EnvWidth_c; x++) {
        EnvMASNAVXYTHETALATCfg.Grid2D[x] = new unsigned char[EnvMASNAVXYTHETALATCfg.EnvHeight_c];
    }

    //environment:
    if (0 == mapdata) {
        for (int y = 0; y < EnvMASNAVXYTHETALATCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvMASNAVXYTHETALATCfg.EnvWidth_c; x++) {
                EnvMASNAVXYTHETALATCfg.Grid2D[x][y] = 0;
            }
        }
    }
    else {
        for (int y = 0; y < EnvMASNAVXYTHETALATCfg.EnvHeight_c; y++) {
            for (int x = 0; x < EnvMASNAVXYTHETALATCfg.EnvWidth_c; x++) {
                EnvMASNAVXYTHETALATCfg.Grid2D[x][y] = mapdata[x + y * width];
            }
        }
    }
}

bool EnvironmentMASNAVXYTHETALAT::ReadinCell(sbpl_xy_theta_cell_t* cell, FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->x = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->y = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) return false;
    cell->theta = atoi(sTemp);

    //normalize the angle
    cell->theta = NORMALIZEDISCTHETA(cell->theta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);

    return true;
}

bool EnvironmentMASNAVXYTHETALAT::ReadinPose(sbpl_xy_theta_pt_t* pose, FILE* fIn)
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

bool EnvironmentMASNAVXYTHETALAT::ReadinMotionPrimitive(SBPL_masxytheta_mprimitive* pMotPrim, FILE* fIn)
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
    sourcepose.x = DISCXY2CONT(0, EnvMASNAVXYTHETALATCfg.cellsize_m);
    sourcepose.y = DISCXY2CONT(0, EnvMASNAVXYTHETALATCfg.cellsize_m);
    sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
    double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x;
    double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y;
    double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta;
    int endx_c = CONTXY2DISC(mp_endx_m, EnvMASNAVXYTHETALATCfg.cellsize_m);
    int endy_c = CONTXY2DISC(mp_endy_m, EnvMASNAVXYTHETALATCfg.cellsize_m);
    int endtheta_c = ContTheta2Disc(mp_endtheta_rad, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
    if (endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y || endtheta_c != pMotPrim->endcell.theta) {
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

bool EnvironmentMASNAVXYTHETALAT::ReadMotionPrimitives(FILE* fMotPrims)
{
    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    SBPL_INFO("Reading in motion primitives...");

    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0) return false;
    if (fabs(fTemp - EnvMASNAVXYTHETALATCfg.cellsize_m) > ERR_EPS) {
        SBPL_ERROR("ERROR: invalid resolution %f (instead of %f) in the dynamics file\n", fTemp,
                   EnvMASNAVXYTHETALATCfg.cellsize_m);
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
    if (dTemp != EnvMASNAVXYTHETALATCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n",
                   dTemp, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
        return false;
    }

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
        SBPL_masxytheta_mprimitive motprim;

        if (EnvironmentMASNAVXYTHETALAT::ReadinMotionPrimitive(&motprim, fMotPrims) == false) return false;

        EnvMASNAVXYTHETALATCfg.mprimV.push_back(motprim);
    }
    SBPL_INFO("done ");

    return true;
}

void EnvironmentMASNAVXYTHETALAT::ComputeReplanningDataforAction(EnvMASNAVXYTHETALATAction_t* action)
{
    int j;

    //iterate over all the cells involved in the action
    sbpl_xy_theta_cell_t startcell3d, endcell3d;
    for (int i = 0; i < (int)action->intersectingcellsV.size(); i++) {
        //compute the translated affected search Pose - what state has an
        //outgoing action whose intersecting cell is at 0,0
        startcell3d.theta = action->starttheta;
        startcell3d.x = -action->intersectingcellsV.at(i).x;
        startcell3d.y = -action->intersectingcellsV.at(i).y;

        //compute the translated affected search Pose - what state has an
        //incoming action whose intersecting cell is at 0,0
        endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
        endcell3d.x = startcell3d.x + action->dX;
        endcell3d.y = startcell3d.y + action->dY;

        //store the cells if not already there
        for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
            if (affectedsuccstatesV.at(j) == endcell3d) break;
        }
        if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

        for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
            if (affectedpredstatesV.at(j) == startcell3d) break;
        }
        if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);
    }//over intersecting cells

    //add the centers since with h2d we are using these in cost computations
    //---intersecting cell = origin
    //compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -0;
    startcell3d.y = -0;

    //compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
    endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    //store the cells if not already there
    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) break;
    }
    if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) break;
    }
    if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);

    //---intersecting cell = outcome state
    //compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -action->dX;
    startcell3d.y = -action->dY;

    //compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
    endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) break;
    }
    if (j == (int)affectedsuccstatesV.size()) affectedsuccstatesV.push_back(endcell3d);

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) break;
    }
    if (j == (int)affectedpredstatesV.size()) affectedpredstatesV.push_back(startcell3d);
}

//computes all the 3D states whose outgoing actions are potentially affected
//when cell (0,0) changes its status it also does the same for the 3D states
//whose incoming actions are potentially affected when cell (0,0) changes its
//status
void EnvironmentMASNAVXYTHETALAT::ComputeReplanningData()
{
    //iterate over all actions
    //orientations
    for (int tind = 0; tind < EnvMASNAVXYTHETALATCfg.NumThetaDirs; tind++) {
        //actions
        for (int aind = 0; aind < EnvMASNAVXYTHETALATCfg.actionwidth; aind++) {
            //compute replanning data for this action 
            ComputeReplanningDataforAction(&EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind]);
        }
    }
}

//here motionprimitivevector contains actions only for 0 angle
void EnvironmentMASNAVXYTHETALAT::PrecomputeActionswithBaseMotionPrimitive(
        vector<SBPL_masxytheta_mprimitive>* motionprimitiveV)
{
    SBPL_PRINTF("Pre-computing action data using base motion primitives...\n");
    EnvMASNAVXYTHETALATCfg.ActionsV = new EnvMASNAVXYTHETALATAction_t*[EnvMASNAVXYTHETALATCfg.NumThetaDirs];
    EnvMASNAVXYTHETALATCfg.PredActionsV = new vector<EnvMASNAVXYTHETALATAction_t*> [EnvMASNAVXYTHETALATCfg.NumThetaDirs];
    vector<sbpl_2Dcell_t> footprint;

    //iterate over source angles
    for (int tind = 0; tind < EnvMASNAVXYTHETALATCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
        EnvMASNAVXYTHETALATCfg.ActionsV[tind] = new EnvMASNAVXYTHETALATAction_t[motionprimitiveV->size()];

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvMASNAVXYTHETALATCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvMASNAVXYTHETALATCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvMASNAVXYTHETALATCfg.NumThetaDirs);

        //iterate over motion primitives
        for (size_t aind = 0; aind < motionprimitiveV->size(); aind++) {
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
            double mp_endx_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size() - 1].x;
            double mp_endy_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size() - 1].y;
            double mp_endtheta_rad =
                    motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size() - 1].theta;

            double endx = sourcepose.x + (mp_endx_m * cos(sourcepose.theta) - mp_endy_m * sin(sourcepose.theta));
            double endy = sourcepose.y + (mp_endx_m * sin(sourcepose.theta) + mp_endy_m * cos(sourcepose.theta));

            int endx_c = CONTXY2DISC(endx, EnvMASNAVXYTHETALATCfg.cellsize_m);
            int endy_c = CONTXY2DISC(endy, EnvMASNAVXYTHETALATCfg.cellsize_m);

            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = ContTheta2Disc(mp_endtheta_rad + sourcepose.theta,
                                                                               EnvMASNAVXYTHETALATCfg.NumThetaDirs);
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX = endx_c;
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY = endy_c;
            if (EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY != 0 || EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX != 0)
                EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(ceil(MASNAVXYTHETALAT_COSTMULT_MTOMM
                    * EnvMASNAVXYTHETALATCfg.cellsize_m / EnvMASNAVXYTHETALATCfg.nominalvel_mpersecs
                    * sqrt((double)(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX
                        * EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX + EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY
                        * EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY))));
            else
                //cost of turn in place
                EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(MASNAVXYTHETALAT_COSTMULT_MTOMM
                    * EnvMASNAVXYTHETALATCfg.timetoturn45degsinplace_secs
                    * fabs(computeMinUnsignedAngleDiff(mp_endtheta_rad, 0)) / (PI_CONST / 4.0));

            //compute and store interm points as well as intersecting cells
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.clear();
            sbpl_xy_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.theta = previnterm3Dcell.x = previnterm3Dcell.y = 0;

            for (int pind = 0; pind < (int)motionprimitiveV->at(aind).intermptV.size(); pind++) {
                sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(aind).intermptV[pind];

                //rotate it appropriately
                double rotx = intermpt.x * cos(sourcepose.theta) - intermpt.y * sin(sourcepose.theta);
                double roty = intermpt.x * sin(sourcepose.theta) + intermpt.y * cos(sourcepose.theta);
                intermpt.x = rotx;
                intermpt.y = roty;
                intermpt.theta = normalizeAngle(sourcepose.theta + intermpt.theta);

                //store it (they are with reference to 0,0,stattheta (not
                //sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
                EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);
            }
            //now compute the intersecting cells for this motion (including ignoring the source footprint)
            get_2d_motion_cells(EnvMASNAVXYTHETALATCfg.FootprintPolygon,
                                EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV,
                                &EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV,
                                EnvMASNAVXYTHETALATCfg.cellsize_m);

#if DEBUG
            SBPL_FPRINTF(fDeb,
                         "action tind=%d aind=%d: dX=%d dY=%d endtheta=%d (%.2f degs -> %.2f degs) "
                         "cost=%d (mprim: %.2f %.2f %.2f)\n",
                         tind,
                         (int)aind,
                         EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX,
                         EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
                         EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta,
                         sourcepose.theta * 180.0 / PI_CONST,
                         EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size() - 1].theta * 180.0 / PI_CONST,
                         EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost,
                         mp_endx_m,
                         mp_endy_m,
                         mp_endtheta_rad);
#endif

            //add to the list of backward actions
            int targettheta = EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) targettheta = targettheta + EnvMASNAVXYTHETALATCfg.NumThetaDirs;
            EnvMASNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind]));
        }
    }

    //set number of actions
    EnvMASNAVXYTHETALATCfg.actionwidth = motionprimitiveV->size();

    //now compute replanning data
    ComputeReplanningData();

    SBPL_INFO("done pre-computing action data based on motion primitives\n");
}

//here motionprimitivevector contains actions for all angles
void EnvironmentMASNAVXYTHETALAT::PrecomputeActionswithCompleteMotionPrimitive(
        vector<SBPL_masxytheta_mprimitive>* motionprimitiveV)
{
    SBPL_INFO("Pre-computing action data using motion primitives for every angle...\n");
    EnvMASNAVXYTHETALATCfg.ActionsV = new EnvMASNAVXYTHETALATAction_t*[EnvMASNAVXYTHETALATCfg.NumThetaDirs];
    EnvMASNAVXYTHETALATCfg.PredActionsV = new vector<EnvMASNAVXYTHETALATAction_t*> [EnvMASNAVXYTHETALATCfg.NumThetaDirs];
    vector<sbpl_2Dcell_t> footprint;

    if (motionprimitiveV->size() % EnvMASNAVXYTHETALATCfg.NumThetaDirs != 0) {
        SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
        throw new SBPL_Exception();
    }

    EnvMASNAVXYTHETALATCfg.actionwidth = ((int)motionprimitiveV->size()) / EnvMASNAVXYTHETALATCfg.NumThetaDirs;

    //iterate over source angles
    int maxnumofactions = 0;
    for (int tind = 0; tind < EnvMASNAVXYTHETALATCfg.NumThetaDirs; tind++) {
        SBPL_INFO("pre-computing for angle %d out of %d angles\n", tind, EnvMASNAVXYTHETALATCfg.NumThetaDirs);

        EnvMASNAVXYTHETALATCfg.ActionsV[tind] = new EnvMASNAVXYTHETALATAction_t[EnvMASNAVXYTHETALATCfg.actionwidth];

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvMASNAVXYTHETALATCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvMASNAVXYTHETALATCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvMASNAVXYTHETALATCfg.NumThetaDirs);

        //iterate over motion primitives
        int numofactions = 0;
        int aind = -1;
        for (int mind = 0; mind < (int)motionprimitiveV->size(); mind++) {
            //find a motion primitive for this angle
            if (motionprimitiveV->at(mind).starttheta_c != tind) continue;

            aind++;
            numofactions++;

            //action index
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;

            //start angle
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;

            //compute dislocation
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;

            //compute and store interm points as well as intersecting cells
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.clear();

            sbpl_xy_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.x = 0;
            previnterm3Dcell.y = 0;

            // Compute all the intersected cells for this action (intermptV and interm3DcellsV)
            for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++) {
                sbpl_xy_theta_pt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
                EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

                // also compute the intermediate discrete cells if not there already
                sbpl_xy_theta_pt_t pose;
                pose.x = intermpt.x + sourcepose.x;
                pose.y = intermpt.y + sourcepose.y;
                pose.theta = intermpt.theta;

                sbpl_xy_theta_cell_t intermediate2dCell;
                intermediate2dCell.x = CONTXY2DISC(pose.x, EnvMASNAVXYTHETALATCfg.cellsize_m);
                intermediate2dCell.y = CONTXY2DISC(pose.y, EnvMASNAVXYTHETALATCfg.cellsize_m);

                // add unique cells to the list
                if (EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.size() == 0 || intermediate2dCell.x
                    != previnterm3Dcell.x || intermediate2dCell.y != previnterm3Dcell.y) {
                    EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.push_back(intermediate2dCell);
                }

                previnterm3Dcell = intermediate2dCell;
            }

            //compute linear and angular time
            double linear_distance = 0;
            for (unsigned int i = 1; i < EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size(); i++) {
                double x0 = EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i - 1].x;
                double y0 = EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i - 1].y;
                double x1 = EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i].x;
                double y1 = EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[i].y;
                double dx = x1 - x0;
                double dy = y1 - y0;
                linear_distance += sqrt(dx * dx + dy * dy);
            }
            double linear_time = linear_distance / EnvMASNAVXYTHETALATCfg.nominalvel_mpersecs;
            double angular_distance =
                    fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta,
                                                                    EnvMASNAVXYTHETALATCfg.NumThetaDirs),
                                                     DiscTheta2Cont(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta,
                                                                    EnvMASNAVXYTHETALATCfg.NumThetaDirs)));
            double angular_time = angular_distance / ((PI_CONST / 4.0) /
                                  EnvMASNAVXYTHETALATCfg.timetoturn45degsinplace_secs);
            //make the cost the max of the two times
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost =
                    (int)(ceil(MASNAVXYTHETALAT_COSTMULT_MTOMM * max(linear_time, angular_time)));
            //use any additional cost multiplier
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;

            //now compute the intersecting cells for this motion (including ignoring the source footprint)
            get_2d_motion_cells(EnvMASNAVXYTHETALATCfg.FootprintPolygon, motionprimitiveV->at(mind).intermptV,
                                &EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV,
                                EnvMASNAVXYTHETALATCfg.cellsize_m);

#if DEBUG
            SBPL_FPRINTF(fDeb,
                         "action tind=%2d aind=%2d: dX=%3d dY=%3d endtheta=%3d (%6.2f degs -> %6.2f degs) "
                         "cost=%4d (mprimID %3d: %3d %3d %3d) numofintermcells = %d numofintercells=%d\n",
                         tind,
                         aind,
                         EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX,
                         EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
                         EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta,
                         EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[0].theta * 180 / PI_CONST,
                         EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV[EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.size() - 1].theta * 180 / PI_CONST, EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost,
                         motionprimitiveV->at(mind).motprimID, motionprimitiveV->at(mind).endcell.x,
                         motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.theta,
                         (int)EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.size(),
                         (int)EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.size());
#endif

            //add to the list of backward actions
            int targettheta = EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) targettheta = targettheta + EnvMASNAVXYTHETALATCfg.NumThetaDirs;
            EnvMASNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind]));
        }

        if (maxnumofactions < numofactions) maxnumofactions = numofactions;
    }

    //at this point we don't allow nonuniform number of actions
    if (motionprimitiveV->size() != (size_t)(EnvMASNAVXYTHETALATCfg.NumThetaDirs * maxnumofactions)) {
        SBPL_ERROR("ERROR: nonuniform number of actions is not supported "
                   "(maxnumofactions=%d while motprims=%d thetas=%d\n",
                   maxnumofactions, (unsigned int)motionprimitiveV->size(), EnvMASNAVXYTHETALATCfg.NumThetaDirs);
        throw new SBPL_Exception();
    }

    //now compute replanning data
    ComputeReplanningData();

    SBPL_INFO("done pre-computing action data based on motion primitives\n");
}

void EnvironmentMASNAVXYTHETALAT::DeprecatedPrecomputeActions()
{
    SBPL_INFO("Use of DeprecatedPrecomputeActions() is deprecated and probably doesn't work!\n");

    //construct list of actions
    SBPL_INFO("Pre-computing action data using the motion primitives for a 3D kinematic planning...\n");
    EnvMASNAVXYTHETALATCfg.ActionsV = new EnvMASNAVXYTHETALATAction_t*[EnvMASNAVXYTHETALATCfg.NumThetaDirs];
    EnvMASNAVXYTHETALATCfg.PredActionsV = new vector<EnvMASNAVXYTHETALATAction_t*> [EnvMASNAVXYTHETALATCfg.NumThetaDirs];
    vector<sbpl_2Dcell_t> footprint;
    //iterate over source angles
    for (int tind = 0; tind < EnvMASNAVXYTHETALATCfg.NumThetaDirs; tind++) {
        SBPL_INFO("processing angle %d\n", tind);
        EnvMASNAVXYTHETALATCfg.ActionsV[tind] = new EnvMASNAVXYTHETALATAction_t[EnvMASNAVXYTHETALATCfg.actionwidth];

        //compute sourcepose
        sbpl_xy_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvMASNAVXYTHETALATCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvMASNAVXYTHETALATCfg.cellsize_m);
        sourcepose.theta = DiscTheta2Cont(tind, EnvMASNAVXYTHETALATCfg.NumThetaDirs);

        //the construction assumes that the robot first turns and then goes along this new theta
        int aind = 0;
        for (; aind < 3; aind++) {
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
            //-1,0,1
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = (tind + aind - 1) % EnvMASNAVXYTHETALATCfg.NumThetaDirs; 
            double angle = DiscTheta2Cont(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta,
                                          EnvMASNAVXYTHETALATCfg.NumThetaDirs);
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX = (int)(cos(angle) + 0.5 * (cos(angle) > 0 ? 1 : -1));
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY = (int)(sin(angle) + 0.5 * (sin(angle) > 0 ? 1 : -1));
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(ceil(MASNAVXYTHETALAT_COSTMULT_MTOMM
                * EnvMASNAVXYTHETALATCfg.cellsize_m / EnvMASNAVXYTHETALATCfg.nominalvel_mpersecs
                * sqrt((double)(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX
                    * EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX + EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY
                    * EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY))));

            //compute intersecting cells
            sbpl_xy_theta_pt_t pose;
            pose.x = DISCXY2CONT(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvMASNAVXYTHETALATCfg.cellsize_m);
            pose.y = DISCXY2CONT(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvMASNAVXYTHETALATCfg.cellsize_m);
            pose.theta = angle;
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            get_2d_footprint_cells(EnvMASNAVXYTHETALATCfg.FootprintPolygon,
                                   &EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV, pose,
                                   EnvMASNAVXYTHETALATCfg.cellsize_m);
            RemoveSourceFootprint(sourcepose, &EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
            SBPL_INFO("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
                        tind, aind, EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, angle,
                        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
                        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

            //add to the list of backward actions
            int targettheta = EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) targettheta = targettheta + EnvMASNAVXYTHETALATCfg.NumThetaDirs;
            EnvMASNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind]));
        }

        //decrease and increase angle without movement
        aind = 3;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = tind - 1;
        if (EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta < 0)
            EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta += EnvMASNAVXYTHETALATCfg.NumThetaDirs;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX = 0;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY = 0;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(MASNAVXYTHETALAT_COSTMULT_MTOMM *
                                                              EnvMASNAVXYTHETALATCfg.timetoturn45degsinplace_secs);

        //compute intersecting cells
        sbpl_xy_theta_pt_t pose;
        pose.x = DISCXY2CONT(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvMASNAVXYTHETALATCfg.cellsize_m);
        pose.y = DISCXY2CONT(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvMASNAVXYTHETALATCfg.cellsize_m);
        pose.theta =
                DiscTheta2Cont(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
        get_2d_footprint_cells(EnvMASNAVXYTHETALATCfg.FootprintPolygon,
                               &EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV, pose,
                               EnvMASNAVXYTHETALATCfg.cellsize_m);
        RemoveSourceFootprint(sourcepose, &EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
        SBPL_INFO("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
                    tind, aind, EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta,
                    DiscTheta2Cont(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs),
                    EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
                    EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

        //add to the list of backward actions
        int targettheta = EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
        if (targettheta < 0) targettheta = targettheta + EnvMASNAVXYTHETALATCfg.NumThetaDirs;
        EnvMASNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind]));

        aind = 4;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].aind = aind;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta = (tind + 1) % EnvMASNAVXYTHETALATCfg.NumThetaDirs;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX = 0;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY = 0;
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(MASNAVXYTHETALAT_COSTMULT_MTOMM *
                                                              EnvMASNAVXYTHETALATCfg.timetoturn45degsinplace_secs);

        //compute intersecting cells
        pose.x = DISCXY2CONT(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvMASNAVXYTHETALATCfg.cellsize_m);
        pose.y = DISCXY2CONT(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvMASNAVXYTHETALATCfg.cellsize_m);
        pose.theta =
                DiscTheta2Cont(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
        EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
        get_2d_footprint_cells(EnvMASNAVXYTHETALATCfg.FootprintPolygon,
                               &EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV, pose,
                               EnvMASNAVXYTHETALATCfg.cellsize_m);
        RemoveSourceFootprint(sourcepose, &EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
        SBPL_INFO("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d cost=%d\n",
                    tind, aind, EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs),
                    EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
                    EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

        //add to the list of backward actions
        targettheta = EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind].endtheta;
        if (targettheta < 0) targettheta = targettheta + EnvMASNAVXYTHETALATCfg.NumThetaDirs;
        EnvMASNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvMASNAVXYTHETALATCfg.ActionsV[tind][aind]));
    }

    //now compute replanning data
    ComputeReplanningData();

    SBPL_INFO("done pre-computing action data\n");
}

void EnvironmentMASNAVXYTHETALAT::InitializeEnvConfig(vector<SBPL_masxytheta_mprimitive>* motionprimitiveV)
{
    //aditional to configuration file initialization of EnvMASNAVXYTHETALATCfg if necessary

    //dXY dirs
    EnvMASNAVXYTHETALATCfg.dXY[0][0] = -1;
    EnvMASNAVXYTHETALATCfg.dXY[0][1] = -1;
    EnvMASNAVXYTHETALATCfg.dXY[1][0] = -1;
    EnvMASNAVXYTHETALATCfg.dXY[1][1] = 0;
    EnvMASNAVXYTHETALATCfg.dXY[2][0] = -1;
    EnvMASNAVXYTHETALATCfg.dXY[2][1] = 1;
    EnvMASNAVXYTHETALATCfg.dXY[3][0] = 0;
    EnvMASNAVXYTHETALATCfg.dXY[3][1] = -1;
    EnvMASNAVXYTHETALATCfg.dXY[4][0] = 0;
    EnvMASNAVXYTHETALATCfg.dXY[4][1] = 1;
    EnvMASNAVXYTHETALATCfg.dXY[5][0] = 1;
    EnvMASNAVXYTHETALATCfg.dXY[5][1] = -1;
    EnvMASNAVXYTHETALATCfg.dXY[6][0] = 1;
    EnvMASNAVXYTHETALATCfg.dXY[6][1] = 0;
    EnvMASNAVXYTHETALATCfg.dXY[7][0] = 1;
    EnvMASNAVXYTHETALATCfg.dXY[7][1] = 1;

    sbpl_xy_theta_pt_t temppose;
    temppose.x = 0.0;
    temppose.y = 0.0;
    temppose.theta = 0.0;
    vector<sbpl_2Dcell_t> footprint;
    get_2d_footprint_cells(EnvMASNAVXYTHETALATCfg.FootprintPolygon, &footprint, temppose, EnvMASNAVXYTHETALATCfg.cellsize_m);
    SBPL_INFO("number of cells in footprint of the robot = %d\n", (unsigned int)footprint.size());

    for (vector<sbpl_2Dcell_t>::iterator it = footprint.begin(); it != footprint.end(); ++it) {
        SBPL_PRINTF("Footprint cell at (%d, %d)\n", it->x, it->y);
    }

#if DEBUG
    SBPL_FPRINTF(fDeb, "footprint cells (size=%d):\n", (int)footprint.size());
    for(int i = 0; i < (int) footprint.size(); i++)
    {
        SBPL_FPRINTF(fDeb, "%d %d (cont: %.3f %.3f)\n", footprint.at(i).x, footprint.at(i).y,
                     DISCXY2CONT(footprint.at(i).x, EnvMASNAVXYTHETALATCfg.cellsize_m),
                     DISCXY2CONT(footprint.at(i).y, EnvMASNAVXYTHETALATCfg.cellsize_m));
    }
#endif

    if (motionprimitiveV == NULL)
        DeprecatedPrecomputeActions();
    else
        PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);
}

bool EnvironmentMASNAVXYTHETALAT::IsValidCell(int X, int Y)
{
    return (X >= 0 && X < EnvMASNAVXYTHETALATCfg.EnvWidth_c && Y >= 0 && Y < EnvMASNAVXYTHETALATCfg.EnvHeight_c &&
            EnvMASNAVXYTHETALATCfg.Grid2D[X][Y] < EnvMASNAVXYTHETALATCfg.obsthresh);
}

bool EnvironmentMASNAVXYTHETALAT::IsWithinMapCell(int X, int Y)
{
    return (X >= 0 && X < EnvMASNAVXYTHETALATCfg.EnvWidth_c && Y >= 0 && Y < EnvMASNAVXYTHETALATCfg.EnvHeight_c);
}

/* TODO: Check for collision of robot footprints*/
bool EnvironmentMASNAVXYTHETALAT::IsValidConfiguration(std::vector<pose_t> pos)
{
    vector<sbpl_2Dcell_t> footprint;
    sbpl_xy_theta_pt_t pose;

    //compute continuous pose
    for(int i = 0; i < pos.size(); ++i)
      {
	pose.x = DISCXY2CONT(pos[i].x, EnvMASNAVXYTHETALATCfg.cellsize_m);
	pose.y = DISCXY2CONT(pos[i].y, EnvMASNAVXYTHETALATCfg.cellsize_m);
	pose.theta = DiscTheta2Cont(pos[i].theta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
	
	//compute footprint cells
	get_2d_footprint_cells(EnvMASNAVXYTHETALATCfg.FootprintPolygon, &footprint, pose, EnvMASNAVXYTHETALATCfg.cellsize_m);
	
	//iterate over all footprint cells
	for (int find = 0; find < (int)footprint.size(); find++) {
	  int x = footprint.at(find).x;
	  int y = footprint.at(find).y;
	  
	  if (x < 0 || x >= EnvMASNAVXYTHETALATCfg.EnvWidth_c || y < 0 || y >= EnvMASNAVXYTHETALATCfg.EnvHeight_c ||
	      EnvMASNAVXYTHETALATCfg.Grid2D[x][y] >= EnvMASNAVXYTHETALATCfg.obsthresh)
	    {
	      return false;
	    }
	}
      }
    return true;
}


int EnvironmentMASNAVXYTHETALAT::GetActionCost(int SourceX, int SourceY, int SourceTheta,
                                                EnvMASNAVXYTHETALATAction_t* action)
{
  sbpl_2Dcell_t cell;
  sbpl_xy_theta_cell_t interm3Dcell;
  int i;
  
  //TODO - go over bounding box (minpt and maxpt) to test validity and skip
    //testing boundaries below, also order intersect cells so that the four
    //farthest pts go first
  
  if (!IsValidCell(SourceX, SourceY)) return INFINITECOST;
  if (!IsValidCell(SourceX + action->dX, SourceY + action->dY)) return INFINITECOST;
  
    if (EnvMASNAVXYTHETALATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY] >=
        EnvMASNAVXYTHETALATCfg.cost_inscribed_thresh) 
    {
        return INFINITECOST;
    }
    //need to iterate over discretized center cells and compute cost based on them
    unsigned char maxcellcost = 0;
    for (i = 0; i < (int)action->interm3DcellsV.size(); i++) {
        interm3Dcell = action->interm3DcellsV.at(i);
        interm3Dcell.x = interm3Dcell.x + SourceX;
        interm3Dcell.y = interm3Dcell.y + SourceY;

        if (interm3Dcell.x < 0 || interm3Dcell.x >= EnvMASNAVXYTHETALATCfg.EnvWidth_c || interm3Dcell.y < 0
            || interm3Dcell.y >= EnvMASNAVXYTHETALATCfg.EnvHeight_c) return INFINITECOST;

        maxcellcost = __max(maxcellcost, EnvMASNAVXYTHETALATCfg.Grid2D[interm3Dcell.x][interm3Dcell.y]);

        //check that the robot is NOT in the cell at which there is no valid orientation
        if (maxcellcost >= EnvMASNAVXYTHETALATCfg.cost_inscribed_thresh) return INFINITECOST;
    }

    //check collisions that for the particular footprint orientation along the action
    if (EnvMASNAVXYTHETALATCfg.FootprintPolygon.size() > 1 && (int)maxcellcost >=
        EnvMASNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh)
    {
        checks++;

        for (i = 0; i < (int)action->intersectingcellsV.size(); i++) {
            //get the cell in the map
            cell = action->intersectingcellsV.at(i);
            cell.x = cell.x + SourceX;
            cell.y = cell.y + SourceY;

            //check validity
            if (!IsValidCell(cell.x, cell.y)) return INFINITECOST;

            //if(EnvMASNAVXYTHETALATCfg.Grid2D[cell.x][cell.y] > currentmaxcost)
            ////cost computation changed: cost = max(cost of centers of the
            //robot along action)
            //	currentmaxcost = EnvMASNAVXYTHETALATCfg.Grid2D[cell.x][cell.y];
            //	//intersecting cells are only used for collision checking
        }
    }

    //to ensure consistency of h2D:
    maxcellcost = __max(maxcellcost, EnvMASNAVXYTHETALATCfg.Grid2D[SourceX][SourceY]);
    int currentmaxcost =
            (int)__max(maxcellcost, EnvMASNAVXYTHETALATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);

    return action->cost * (currentmaxcost + 1); //use cell cost as multiplicative factor
}



double EnvironmentMASNAVXYTHETALAT::EuclideanDistance_m(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2));
    return EnvMASNAVXYTHETALATCfg.cellsize_m * sqrt((double)sqdist);
}

//calculates a set of cells that correspond to the specified footprint
//adds points to it (does not clear it beforehand)

void EnvironmentMASNAVXYTHETALAT::CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, vector<sbpl_2Dcell_t>* footprint,
                                                             const vector<sbpl_2Dpt_t>& FootprintPolygon)
{
    int pind;

#if DEBUG
    //  SBPL_INFO("---Calculating Footprint for Pose: %f %f %f---\n",
    //	 pose.x, pose.y, pose.theta);
#endif

    //handle special case where footprint is just a point
    if (FootprintPolygon.size() <= 1) {
        sbpl_2Dcell_t cell;
        cell.x = CONTXY2DISC(pose.x, EnvMASNAVXYTHETALATCfg.cellsize_m);
        cell.y = CONTXY2DISC(pose.y, EnvMASNAVXYTHETALATCfg.cellsize_m);

        for (pind = 0; pind < (int)footprint->size(); pind++) {
            if (cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y) break;
        }
        if (pind == (int)footprint->size()) footprint->push_back(cell);
        return;
    }

    vector<sbpl_2Dpt_t> bounding_polygon;
    unsigned int find;
    double max_x = -INFINITECOST, min_x = INFINITECOST, max_y = -INFINITECOST, min_y = INFINITECOST;
    sbpl_2Dpt_t pt(0, 0);
    for (find = 0; find < FootprintPolygon.size(); find++) {
        //rotate and translate the corner of the robot
        pt = FootprintPolygon[find];

        //rotate and translate the point
        sbpl_2Dpt_t corner;
        corner.x = cos(pose.theta) * pt.x - sin(pose.theta) * pt.y + pose.x;
        corner.y = sin(pose.theta) * pt.x + cos(pose.theta) * pt.y + pose.y;
        bounding_polygon.push_back(corner);
#if DEBUG
        //    SBPL_INFO("Pt: %f %f, Corner: %f %f\n", pt.x, pt.y, corner.x, corner.y);
#endif
        if (corner.x < min_x || find == 0) {
            min_x = corner.x;
        }
        if (corner.x > max_x || find == 0) {
            max_x = corner.x;
        }
        if (corner.y < min_y || find == 0) {
            min_y = corner.y;
        }
        if (corner.y > max_y || find == 0) {
            max_y = corner.y;
        }
    }

#if DEBUG
    //  SBPL_INFO("Footprint bounding box: %f %f %f %f\n", min_x, max_x, min_y, max_y);
#endif
    //initialize previous values to something that will fail the if condition during the first iteration in the for loop
    int prev_discrete_x = CONTXY2DISC(pt.x, EnvMASNAVXYTHETALATCfg.cellsize_m) + 1;
    int prev_discrete_y = CONTXY2DISC(pt.y, EnvMASNAVXYTHETALATCfg.cellsize_m) + 1;
    int prev_inside = 0;
    int discrete_x;
    int discrete_y;

    for (double x = min_x; x <= max_x; x += EnvMASNAVXYTHETALATCfg.cellsize_m / 3) {
        for (double y = min_y; y <= max_y; y += EnvMASNAVXYTHETALATCfg.cellsize_m / 3) {
            pt.x = x;
            pt.y = y;
            discrete_x = CONTXY2DISC(pt.x, EnvMASNAVXYTHETALATCfg.cellsize_m);
            discrete_y = CONTXY2DISC(pt.y, EnvMASNAVXYTHETALATCfg.cellsize_m);

            //see if we just tested this point
            if (discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside == 0) {

#if DEBUG
                //		SBPL_INFO("Testing point: %f %f Discrete: %d %d\n", pt.x, pt.y, discrete_x, discrete_y);
#endif

                if (IsInsideFootprint(pt, &bounding_polygon)) {
                    //convert to a grid point

#if DEBUG
                    //			SBPL_INFO("Pt Inside %f %f\n", pt.x, pt.y);
#endif

                    sbpl_2Dcell_t cell;
                    cell.x = discrete_x;
                    cell.y = discrete_y;

                    //insert point if not there already
                    int pind = 0;
                    for (pind = 0; pind < (int)footprint->size(); pind++) {
                        if (cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y) break;
                    }
                    if (pind == (int)footprint->size()) footprint->push_back(cell);

                    prev_inside = 1;

#if DEBUG
                    //			SBPL_INFO("Added pt to footprint: %f %f\n", pt.x, pt.y);
#endif
                }
                else {
                    prev_inside = 0;
                }

            }
            else {
#if DEBUG
                //SBPL_INFO("Skipping pt: %f %f\n", pt.x, pt.y);
#endif
            }

            prev_discrete_x = discrete_x;
            prev_discrete_y = discrete_y;
        }//over x_min...x_max
    }
}

//calculates a set of cells that correspond to the footprint of the base
//adds points to it (does not clear it beforehand) 
void EnvironmentMASNAVXYTHETALAT::CalculateFootprintForPose(sbpl_xy_theta_pt_t pose, vector<sbpl_2Dcell_t>* footprint)
{
    CalculateFootprintForPose(pose, footprint, EnvMASNAVXYTHETALATCfg.FootprintPolygon);
}

//removes a set of cells that correspond to the specified footprint at the sourcepose
//adds points to it (does not clear it beforehand) 
void EnvironmentMASNAVXYTHETALAT::RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose,
                                                         vector<sbpl_2Dcell_t>* footprint,
                                                         const vector<sbpl_2Dpt_t>& FootprintPolygon)
{
    vector<sbpl_2Dcell_t> sourcefootprint;

    //compute source footprint
    get_2d_footprint_cells(FootprintPolygon, &sourcefootprint, sourcepose, EnvMASNAVXYTHETALATCfg.cellsize_m);

    //now remove the source cells from the footprint
    for (int sind = 0; sind < (int)sourcefootprint.size(); sind++) {
        for (int find = 0; find < (int)footprint->size(); find++) {
            if (sourcefootprint.at(sind).x == footprint->at(find).x && sourcefootprint.at(sind).y
                == footprint->at(find).y) {
                footprint->erase(footprint->begin() + find);
                break;
            }
        }//over footprint
    }//over source
}

//removes a set of cells that correspond to the footprint of the base at the sourcepose
//adds points to it (does not clear it beforehand) 
void EnvironmentMASNAVXYTHETALAT::RemoveSourceFootprint(sbpl_xy_theta_pt_t sourcepose,
                                                         vector<sbpl_2Dcell_t>* footprint)
{
    RemoveSourceFootprint(sourcepose, footprint, EnvMASNAVXYTHETALATCfg.FootprintPolygon);
}

//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentMASNAVXYTHETALAT::EnsureHeuristicsUpdated(bool bGoalHeuristics)
{ /*
    if (bNeedtoRecomputeStartHeuristics && !bGoalHeuristics) {
        grid2Dsearchfromstart->search(EnvMASNAVXYTHETALATCfg.Grid2D, EnvMASNAVXYTHETALATCfg.cost_inscribed_thresh,
                                      EnvMASNAVXYTHETALATCfg.StartX_c, EnvMASNAVXYTHETALATCfg.StartY_c,
                                      EnvMASNAVXYTHETALATCfg.EndX_c, EnvMASNAVXYTHETALATCfg.EndY_c,
                                      SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
        bNeedtoRecomputeStartHeuristics = false;
        SBPL_INFO("2dsolcost_infullunits=%d\n",
                    (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(EnvMASNAVXYTHETALATCfg.EndX_c,
                                                                                   EnvMASNAVXYTHETALATCfg.EndY_c) /
                          EnvMASNAVXYTHETALATCfg.nominalvel_mpersecs));

    }

    if (bNeedtoRecomputeGoalHeuristics && bGoalHeuristics) {
        grid2Dsearchfromgoal->search(EnvMASNAVXYTHETALATCfg.Grid2D, EnvMASNAVXYTHETALATCfg.cost_inscribed_thresh,
                                     EnvMASNAVXYTHETALATCfg.EndX_c, EnvMASNAVXYTHETALATCfg.EndY_c,
                                     EnvMASNAVXYTHETALATCfg.StartX_c, EnvMASNAVXYTHETALATCfg.StartY_c,
                                     SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH);
        bNeedtoRecomputeGoalHeuristics = false;
        SBPL_INFO("2dsolcost_infullunits=%d\n",
                    (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(EnvMASNAVXYTHETALATCfg.StartX_c,
                                                                                  EnvMASNAVXYTHETALATCfg.StartY_c) /
                          EnvMASNAVXYTHETALATCfg.nominalvel_mpersecs));
    }
  */
}

void EnvironmentMASNAVXYTHETALAT::ComputeHeuristicValues()
{
  /*
    //whatever necessary pre-computation of heuristic values is done here
    SBPL_INFO("Precomputing heuristics...\n");

    //allocated 2D grid searches
    grid2Dsearchfromstart = new SBPL2DGridSearch(EnvMASNAVXYTHETALATCfg.EnvWidth_c, EnvMASNAVXYTHETALATCfg.EnvHeight_c,
                                                 (float)EnvMASNAVXYTHETALATCfg.cellsize_m);
    grid2Dsearchfromgoal = new SBPL2DGridSearch(EnvMASNAVXYTHETALATCfg.EnvWidth_c, EnvMASNAVXYTHETALATCfg.EnvHeight_c,
                                                (float)EnvMASNAVXYTHETALATCfg.cellsize_m);

    //set OPEN type to sliding buckets
    grid2Dsearchfromstart->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);
    grid2Dsearchfromgoal->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);

    SBPL_INFO("done\n");
  */
}

//------------debugging functions---------------------------------------------
bool EnvironmentMASNAVXYTHETALAT::CheckQuant(FILE* fOut)
{
    for (double theta = -10; theta < 10; theta += 2.0 * PI_CONST / EnvMASNAVXYTHETALATCfg.NumThetaDirs * 0.01) {
        int nTheta = ContTheta2Disc(theta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
        double newTheta = DiscTheta2Cont(nTheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
        int nnewTheta = ContTheta2Disc(newTheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);

        SBPL_FPRINTF(fOut, "theta=%f(%f)->%d->%f->%d\n", theta, theta * 180 / PI_CONST, nTheta, newTheta, nnewTheta);

        if (nTheta != nnewTheta) {
            SBPL_ERROR("ERROR: invalid quantization\n");
            return false;
        }
    }

    return true;
}

//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------

bool EnvironmentMASNAVXYTHETALAT::InitializeEnv(const char* cfgFile)
{
  return false;
}


bool EnvironmentMASNAVXYTHETALAT::InitializeEnv(int width, int height, const unsigned char* mapdata, int numagents, 
						    std::vector<pose_t> start, std::vector<pose_t> goal,
						    double goaltol_x, double goaltol_y, double goaltol_theta,
						    const vector<sbpl_2Dpt_t> & perimeterptsV,
						    double cellsize_m, double nominalvel_mpersecs,
						    double timetoturn45degsinplace_secs, unsigned char obsthresh,
						    const char* sMotPrimFile)
{
    SBPL_INFO("env: initialize with width=%d height=%d "
                "cellsize=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d\n",
                width, height, cellsize_m, nominalvel_mpersecs,
                timetoturn45degsinplace_secs, obsthresh);

    SBPL_INFO("NOTE: goaltol parameters currently unused\n");

    SBPL_INFO("perimeter has size=%d\n", (unsigned int)perimeterptsV.size());

    for (int i = 0; i < (int)perimeterptsV.size(); i++) {
        SBPL_INFO("perimeter(%d) = %.4f %.4f\n", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
    }

    EnvMASNAVXYTHETALATCfg.obsthresh = obsthresh;

    //TODO - need to set the tolerance as well
    for(int i = 0; i < start.size(); i++){
      start[i].x = CONTXY2DISC(start[i].x, cellsize_m);
      start[i].y = CONTXY2DISC(start[i].y, cellsize_m);
      start[i].theta = ContTheta2Disc(start[i].theta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);	
    }
    
    for(int i = 0; i < goal.size(); i++){
      goal[i].x = CONTXY2DISC(goal[i].x, cellsize_m);
      goal[i].y = CONTXY2DISC(goal[i].y, cellsize_m);
      goal[i].theta = ContTheta2Disc(goal[i].theta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
    }

    SetConfiguration(width, height, mapdata, start, goal,
                     cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, perimeterptsV);

    if (sMotPrimFile != NULL) {
      FILE* fMotPrim = fopen(sMotPrimFile, "r");
      if (fMotPrim == NULL) {
	SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
	throw new SBPL_Exception();
      }
      
      if (ReadMotionPrimitives(fMotPrim) == false) {
            SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
            throw new SBPL_Exception();
      }
        fclose(fMotPrim);
    }

    if (EnvMASNAVXYTHETALATCfg.mprimV.size() != 0) {
        InitGeneral(&EnvMASNAVXYTHETALATCfg.mprimV);
    }
    else
        InitGeneral( NULL);

    return true;
}

bool EnvironmentMASNAVXYTHETALAT::InitGeneral(vector<SBPL_masxytheta_mprimitive>* motionprimitiveV)
{
    //Initialize other parameters of the environment
    InitializeEnvConfig(motionprimitiveV);

    //initialize Environment
    InitializeEnvironment();

    //pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}


void EnvironmentMASNAVXYTHETALAT::PrintHeuristicValues()
{
#ifndef ROS
    const char* heur = "heur.txt";
#endif
    FILE* fHeur = SBPL_FOPEN(heur, "w");
    if (fHeur == NULL) {
        SBPL_ERROR("ERROR: could not open debug file to write heuristic\n");
        throw new SBPL_Exception();
    }
    SBPL2DGridSearch* grid2Dsearch = NULL;

    for (int i = 0; i < 2; i++) {
        if (i == 0 && grid2Dsearchfromstart != NULL) {
            grid2Dsearch = grid2Dsearchfromstart;
            SBPL_FPRINTF(fHeur, "start heuristics:\n");
        }
        else if (i == 1 && grid2Dsearchfromgoal != NULL) {
            grid2Dsearch = grid2Dsearchfromgoal;
            SBPL_FPRINTF(fHeur, "goal heuristics:\n");
        }
        else
            continue;

        for (int y = 0; y < EnvMASNAVXYTHETALATCfg.EnvHeight_c; y++) {
	  for (int x = 0; x < EnvMASNAVXYTHETALATCfg.EnvWidth_c; x++) {
	    if (grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y) < INFINITECOST){
	      SBPL_FPRINTF(fHeur, "%5d ", grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y));
	    }
	    else
	      {
	      SBPL_FPRINTF(fHeur, "XXXXX ");
	      }
	  }
	  SBPL_FPRINTF(fHeur, "\n");
        }
    }
    SBPL_FCLOSE(fHeur);
}

void EnvironmentMASNAVXYTHETALAT::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvMASNAVXYTHETALAT... function: SetAllPreds is undefined\n");
    throw new SBPL_Exception();
}


void EnvironmentMASNAVXYTHETALAT::GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvMASNAVXYTHETALATAction_t*>* actionindV/*=NULL*/){
  GetSuccs(SourceStateID, SuccIDV, CostV, actionindV);
}

void EnvironmentMASNAVXYTHETALAT::GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvMASNAVXYTHETALATAction_t*>* actionindV/*=NULL*/){
  GetSuccs(SourceStateID, SuccIDV, CostV, actionindV);
}

//lazysuccswithuniqueids returns true succs
void EnvironmentMASNAVXYTHETALAT::GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvMASNAVXYTHETALATAction_t*>* actionindV/*=NULL*/){
  GetSuccsWithUniqueIds(SourceStateID, SuccIDV, CostV, actionindV);
  for(int i = 0; i < SuccIDV->size(); i++)
    isTrueCost->push_back(true);
}

const EnvMASNAVXYTHETALATConfig_t* EnvironmentMASNAVXYTHETALAT::GetEnvNavConfig()
{
    return &EnvMASNAVXYTHETALATCfg;
}

bool EnvironmentMASNAVXYTHETALAT::UpdateCost(int x, int y, unsigned char newcost)
{
#if DEBUG
    //SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,EnvMASNAVXYTHETALATCfg.Grid2D[x][y], newcost);
#endif

    EnvMASNAVXYTHETALATCfg.Grid2D[x][y] = newcost;

    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;

    return true;
}

bool EnvironmentMASNAVXYTHETALAT::SetMap(const unsigned char* mapdata)
{
    int xind = -1, yind = -1;

    for (xind = 0; xind < EnvMASNAVXYTHETALATCfg.EnvWidth_c; xind++) {
        for (yind = 0; yind < EnvMASNAVXYTHETALATCfg.EnvHeight_c; yind++) {
            EnvMASNAVXYTHETALATCfg.Grid2D[xind][yind] = mapdata[xind + yind * EnvMASNAVXYTHETALATCfg.EnvWidth_c];
        }
    }

    bNeedtoRecomputeStartHeuristics = true;
    bNeedtoRecomputeGoalHeuristics = true;

    return true;
}

void EnvironmentMASNAVXYTHETALAT::PrintEnv_Config(FILE* fOut)
{
    //implement this if the planner needs to print out EnvMASNAVXYTHETALAT. configuration

    SBPL_ERROR("ERROR in EnvMASNAVXYTHETALAT... function: PrintEnv_Config is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentMASNAVXYTHETALAT::PrintTimeStat(FILE* fOut)
{
#if TIME_DEBUG
    SBPL_FPRINTF(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, "
                 "time_getsuccs = %f\n",
                 time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC,
                 time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}

bool EnvironmentMASNAVXYTHETALAT::IsObstacle(int x, int y)
{
#if DEBUG
    SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,EnvMASNAVXYTHETALATCfg.Grid2D[x][y]);
#endif

    return (EnvMASNAVXYTHETALATCfg.Grid2D[x][y] >= EnvMASNAVXYTHETALATCfg.obsthresh);
}

/*
void EnvironmentMASNAVXYTHETALAT::GetEnvParms(int *size_x, std::vector<pose_t>* start, std::vector<pose_t>* goal,
                                               double* cellsize_m, double* nominalvel_mpersecs,
                                               double* timetoturn45degsinplace_secs, unsigned char* obsthresh,
                                               vector<SBPL_masxytheta_mprimitive>* mprimitiveV)
{
    *size_x = EnvMASNAVXYTHETALATCfg.EnvWidth_c;
    *size_y = EnvMASNAVXYTHETALATCfg.EnvHeight_c;

    *startx = DISCXY2CONT(EnvMASNAVXYTHETALATCfg.StartX_c, EnvMASNAVXYTHETALATCfg.cellsize_m);
    *starty = DISCXY2CONT(EnvMASNAVXYTHETALATCfg.StartY_c, EnvMASNAVXYTHETALATCfg.cellsize_m);
    *starttheta = DiscTheta2Cont(EnvMASNAVXYTHETALATCfg.StartTheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
    *goalx = DISCXY2CONT(EnvMASNAVXYTHETALATCfg.EndX_c, EnvMASNAVXYTHETALATCfg.cellsize_m);
    *goaly = DISCXY2CONT(EnvMASNAVXYTHETALATCfg.EndY_c, EnvMASNAVXYTHETALATCfg.cellsize_m);
    *goaltheta = DiscTheta2Cont(EnvMASNAVXYTHETALATCfg.EndTheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);

    *cellsize_m = EnvMASNAVXYTHETALATCfg.cellsize_m;
    *nominalvel_mpersecs = EnvMASNAVXYTHETALATCfg.nominalvel_mpersecs;
    *timetoturn45degsinplace_secs = EnvMASNAVXYTHETALATCfg.timetoturn45degsinplace_secs;

    *obsthresh = EnvMASNAVXYTHETALATCfg.obsthresh;

    *mprimitiveV = EnvMASNAVXYTHETALATCfg.mprimV;
}
*/

bool EnvironmentMASNAVXYTHETALAT::PoseContToDisc(double px, double py, double pth, int &ix, int &iy, int &ith) const
{
    ix = CONTXY2DISC(px, EnvMASNAVXYTHETALATCfg.cellsize_m);
    iy = CONTXY2DISC(py, EnvMASNAVXYTHETALATCfg.cellsize_m);
    ith = ContTheta2Disc(pth, EnvMASNAVXYTHETALATCfg.NumThetaDirs); // ContTheta2Disc() normalizes the angle
    return (pth >= -2 * PI_CONST) && (pth <= 2 * PI_CONST) && (ix >= 0) && (ix < EnvMASNAVXYTHETALATCfg.EnvWidth_c) &&
           (iy >= 0) && (iy < EnvMASNAVXYTHETALATCfg.EnvHeight_c);
}

bool EnvironmentMASNAVXYTHETALAT::PoseDiscToCont(int ix, int iy, int ith, double &px, double &py, double &pth) const
{
    px = DISCXY2CONT(ix, EnvMASNAVXYTHETALATCfg.cellsize_m);
    py = DISCXY2CONT(iy, EnvMASNAVXYTHETALATCfg.cellsize_m);
    pth = normalizeAngle(DiscTheta2Cont(ith, EnvMASNAVXYTHETALATCfg.NumThetaDirs));
    return (ith >= 0) && (ith < EnvMASNAVXYTHETALATCfg.NumThetaDirs) && (ix >= 0) &&
           (ix < EnvMASNAVXYTHETALATCfg.EnvWidth_c) && (iy >= 0) && (iy < EnvMASNAVXYTHETALATCfg.EnvHeight_c);
}

unsigned char EnvironmentMASNAVXYTHETALAT::GetMapCost(int x, int y)
{
    return EnvMASNAVXYTHETALATCfg.Grid2D[x][y];
}

bool EnvironmentMASNAVXYTHETALAT::SetEnvParameter(const char* parameter, int value)
{
    if (EnvMASNAVXYTHETALAT.bInitialized == true) {
        SBPL_ERROR("ERROR: all parameters must be set before initialization of the environment\n");
        return false;
    }

    SBPL_INFO("setting parameter %s to %d\n", parameter, value);

    if (strcmp(parameter, "cost_inscribed_thresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvMASNAVXYTHETALATCfg.cost_inscribed_thresh = (unsigned char)value;
    }
    else if (strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvMASNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh = value;
    }
    else if (strcmp(parameter, "cost_obsthresh") == 0) {
        if (value < 0 || value > 255) {
            SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
            return false;
        }
        EnvMASNAVXYTHETALATCfg.obsthresh = (unsigned char)value;
    }
    else if (strcmp(parameter, "NumAgents") == 0){
      if (value < 0){
	SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
	return false;
      }
      EnvMASNAVXYTHETALATCfg.NumAgents = value;
    }
    else {
        SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
        return false;
    }

    return true;
}

int EnvironmentMASNAVXYTHETALAT::GetEnvParameter(const char* parameter)
{
    if (strcmp(parameter, "cost_inscribed_thresh") == 0) {
        return (int)EnvMASNAVXYTHETALATCfg.cost_inscribed_thresh;
    }
    else if (strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0) {
        return (int)EnvMASNAVXYTHETALATCfg.cost_possibly_circumscribed_thresh;
    }
    else if (strcmp(parameter, "cost_obsthresh") == 0) {
        return (int)EnvMASNAVXYTHETALATCfg.obsthresh;
    }
    else {
        SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
        throw new SBPL_Exception();
    }
}

void EnvironmentMASNAVXYTHETALAT::GetCoordFromState(int stateID, vector<pose_t>& poses, vector<bool>& goalsVisited) const
{
    EnvMASNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    poses = HashEntry->poses;
    goalsVisited = HashEntry->goalsVisited;
}

int EnvironmentMASNAVXYTHETALAT::GetStateFromCoord(vector<pose_t> poses, vector<bool> goalsVisited)
{
    EnvMASNAVXYTHETALATHashEntry_t* OutHashEntry;
    if ((OutHashEntry = (this->*GetHashEntry)(poses, goalsVisited)) == NULL) {
        //have to create a new entry
      OutHashEntry = (this->*CreateNewHashEntry)(poses, goalsVisited);
    }
    return OutHashEntry->stateID;
}

/*
void EnvironmentMASNAVXYTHETALAT::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath,
                                                                 vector<sbpl_xy_theta_pt_t>* xythetaPath)
{
    vector<EnvMASNAVXYTHETALATAction_t*> actionV;
    vector<int> CostV;
    vector<int> SuccIDV;
    int targetx_c, targety_c, targettheta_c;
    int sourcex_c, sourcey_c, sourcetheta_c;

    SBPL_INFO("checks=%ld\n", checks);

    xythetaPath->clear();

#if DEBUG
    SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

    for (int pind = 0; pind < (int)(stateIDPath->size()) - 1; pind++) {
        int sourceID = stateIDPath->at(pind);
        int targetID = stateIDPath->at(pind + 1);

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
#endif

        //get successors and pick the target via the cheapest action
        SuccIDV.clear();
        CostV.clear();
        actionV.clear();
        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

        int bestcost = INFINITECOST;
        int bestsind = -1;

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
        GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
        SBPL_FPRINTF(fDeb, "looking for %d %d %d -> %d %d %d (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcetheta_c,
                     targetx_c, targety_c, targettheta_c, (int)SuccIDV.size());
#endif

        for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
#if DEBUG
            int x_c, y_c, theta_c;
            GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c);
            SBPL_FPRINTF(fDeb, "succ: %d %d %d\n", x_c, y_c, theta_c);
#endif

            if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost) {
                bestcost = CostV[sind];
                bestsind = sind;
            }
        }
        if (bestsind == -1) {
            SBPL_ERROR("ERROR: successor not found for transition:\n");
            GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
            GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
            SBPL_INFO("%d %d %d -> %d %d %d\n", sourcex_c, sourcey_c, sourcetheta_c, targetx_c, targety_c,
                        targettheta_c);
            throw new SBPL_Exception();
        }

        //now push in the actual path
        int sourcex_c, sourcey_c, sourcetheta_c;
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
        double sourcex, sourcey;
        sourcex = DISCXY2CONT(sourcex_c, EnvMASNAVXYTHETALATCfg.cellsize_m);
        sourcey = DISCXY2CONT(sourcey_c, EnvMASNAVXYTHETALATCfg.cellsize_m);
        //TODO - when there are no motion primitives we should still print source state
        for (int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size()) - 1; ipind++) {
            //translate appropriately
            sbpl_xy_theta_pt_t intermpt = actionV[bestsind]->intermptV[ipind];
            intermpt.x += sourcex;
            intermpt.y += sourcey;

#if DEBUG
            int nx = CONTXY2DISC(intermpt.x, EnvMASNAVXYTHETALATCfg.cellsize_m);
            int ny = CONTXY2DISC(intermpt.y, EnvMASNAVXYTHETALATCfg.cellsize_m);
            SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ",
                intermpt.x, intermpt.y, intermpt.theta,
                nx, ny,
                ContTheta2Disc(intermpt.theta, EnvMASNAVXYTHETALATCfg.NumThetaDirs), EnvMASNAVXYTHETALATCfg.Grid2D[nx][ny]);
            if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
            else SBPL_FPRINTF(fDeb, "\n");
#endif

            //store
            xythetaPath->push_back(intermpt);
        }
    }
}
*/



bool EnvironmentMASNAVXYTHETALAT::SetNumGoals(int numgoals)
{
  if (numgoals < 0)
    {
      SBPL_INFO("ERROR: Number of goals cannot be negative");
      return false;
    }
  EnvMASNAVXYTHETALATCfg.NumGoals = numgoals;
  return true;
}


bool EnvironmentMASNAVXYTHETALAT::SetNumAgents(int numagents)
{
  if (numagents < 0)
    {
      SBPL_INFO("ERROR: Number of agents cannot be negative");
      return false;
    }
  EnvMASNAVXYTHETALATCfg.NumAgents = numagents;
  return true;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentMASNAVXYTHETALAT::SetGoal(std::vector<sbpl_xy_theta_pt_t> goal_m)
{
  std::vector<pose_t> goal( EnvMASNAVXYTHETALATCfg.NumGoals);
  for (int goal_i = 0; goal_i < EnvMASNAVXYTHETALATCfg.NumGoals; goal_i ++)
    {
      int x = CONTXY2DISC(goal_m[goal_i].x, EnvMASNAVXYTHETALATCfg.cellsize_m);
      int y = CONTXY2DISC(goal_m[goal_i].y, EnvMASNAVXYTHETALATCfg.cellsize_m);
      int theta = ContTheta2Disc(goal_m[goal_i].theta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);

      SBPL_INFO("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", goal_m[goal_i].x, goal_m[goal_i].y, goal_m[goal_i].theta, x, y, theta);

      if (!IsWithinMapCell(x, y)) {
        SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x, y);
        return -1;
      }
      goal[goal_i].x = x;
      goal[goal_i].y = y;
      goal[goal_i].theta = theta;
    }
      
    if (!IsValidConfiguration(goal)) {
        SBPL_INFO("WARNING: goal configuration is invalid\n");
    }
    
    EnvMASNAVXYTHETALATHashEntry_t* OutHashEntry;
    std::vector<bool> goalsVisited(EnvMASNAVXYTHETALATCfg.NumGoals,true);
    if ((OutHashEntry = (this->*GetHashEntry)(goal, goalsVisited)) == NULL) {
      //have to create a new entry
      OutHashEntry = (this->*CreateNewHashEntry)(goal, goalsVisited);
    }

    //need to recompute start heuristics?
    if (EnvMASNAVXYTHETALAT.goalstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
        bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
    }

    EnvMASNAVXYTHETALAT.goalstateid = OutHashEntry->stateID;

    EnvMASNAVXYTHETALATCfg.end = goal;

    return EnvMASNAVXYTHETALAT.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentMASNAVXYTHETALAT::SetStart(std::vector<sbpl_xy_theta_pt_t> start_m)
{
  std::vector<pose_t> start(EnvMASNAVXYTHETALATCfg.NumAgents);
  std::vector<int> x(EnvMASNAVXYTHETALATCfg.NumAgents), y(EnvMASNAVXYTHETALATCfg.NumAgents), theta(EnvMASNAVXYTHETALATCfg.NumAgents);
  for (int agent_i = 0; agent_i < EnvMASNAVXYTHETALATCfg.NumAgents; ++agent_i)
	   {
	     int x_agent = CONTXY2DISC(start_m[agent_i].x, EnvMASNAVXYTHETALATCfg.cellsize_m);
	     int y_agent = CONTXY2DISC(start_m[agent_i].y, EnvMASNAVXYTHETALATCfg.cellsize_m);
	     int theta_agent = ContTheta2Disc(start_m[agent_i].theta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);
	     
	     if (!IsWithinMapCell(x_agent, y_agent)) {
	       SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x_agent, y_agent);
	       return -1;
	     }
	     
	     SBPL_INFO("env: setting start of Agent %d to %.3f %.3f %.3f (%d %d %d)\n", (agent_i+1), start_m[agent_i].x, start_m[agent_i].y, start_m[agent_i].theta, x_agent, y_agent, theta_agent);
	     start[agent_i].x = x_agent;
	     start[agent_i].y = y_agent;
	     start[agent_i].theta = theta_agent;
	   }  

  if (!IsValidConfiguration(start)) {
    SBPL_INFO("WARNING: start configuration is invalid\n");
  }
	   
    EnvMASNAVXYTHETALATHashEntry_t* OutHashEntry;
    std::vector<bool> goalsvisited(EnvMASNAVXYTHETALATCfg.NumGoals, false);
    if ((OutHashEntry = (this->*GetHashEntry)(start, goalsvisited)) == NULL) {
        //have to create a new entry
      OutHashEntry = (this->*CreateNewHashEntry)(start, goalsvisited);
    }

    //need to recompute start heuristics?
    if (EnvMASNAVXYTHETALAT.startstateid != OutHashEntry->stateID) {
        bNeedtoRecomputeStartHeuristics = true;
        //because termination condition can be not all states TODO - make it dependent on term. condition
        bNeedtoRecomputeGoalHeuristics = true; 
    }

    //set start
    EnvMASNAVXYTHETALAT.startstateid = OutHashEntry->stateID;
    EnvMASNAVXYTHETALATCfg.start = start;
    
    return EnvMASNAVXYTHETALAT.startstateid;
}

void EnvironmentMASNAVXYTHETALAT::GetFakePath(int stateID, std::vector<double>& coord)
{
  std::vector<int> d_coord(3);
  std::vector<double> c_coord(3);
  std::vector<pose_t> start = EnvMASNAVXYTHETALATCfg.start;
  for(int agent_i = 0, i = 0; agent_i < EnvMASNAVXYTHETALATCfg.NumAgents; agent_i++, i+=3)
    {
      d_coord[0] = start[agent_i].x;
      d_coord[1] = start[agent_i].y + stateID;
      d_coord[2] = start[agent_i].theta;
      
      bool ret = PoseDiscToCont(d_coord[0], d_coord[1], d_coord[2], c_coord[0], c_coord[1], c_coord[2]);
      if(ret)
	{
	  coord[i] = c_coord[0];
	  coord[i+1] = c_coord[1];
	  coord[i+2] = c_coord[2];
	}
      else
	SBPL_ERROR("ERROR in EnvMASNAVXYTHETALAT... function:Get Fake Path"); 
    }
}

void EnvironmentMASNAVXYTHETALAT::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
    if(stateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvMASNAVXYTHETALAT... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL) fOut = stdout;

    EnvMASNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];

    if (stateID == EnvMASNAVXYTHETALAT.goalstateid && bVerbose) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    if (bVerbose)
      {
	for(int i = 0; i < EnvMASNAVXYTHETALATCfg.NumAgents; i++)
	  SBPL_FPRINTF(fOut, "X=%d Y=%d Theta=%d\n", HashEntry->poses[i].x, HashEntry->poses[i].y, HashEntry->poses[i].theta);
      }
    else
      {
	for(int i = 0; i < EnvMASNAVXYTHETALATCfg.NumAgents; i++)
	  SBPL_FPRINTF(fOut, "%.3f %.3f %.3f\n", DISCXY2CONT(HashEntry->poses[i].x, EnvMASNAVXYTHETALATCfg.cellsize_m),
		       DISCXY2CONT(HashEntry->poses[i].y, EnvMASNAVXYTHETALATCfg.cellsize_m),
		       DiscTheta2Cont(HashEntry->poses[i].theta, EnvMASNAVXYTHETALATCfg.NumThetaDirs));
      }   
}



EnvMASNAVXYTHETALATHashEntry_t* EnvironmentMASNAVXYTHETALAT::GetHashEntry_hash(std::vector<pose_t> poses, std::vector<bool> GoalsVisited)
{
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    int binid = GETHASHBIN(poses, GoalsVisited);

#if DEBUG
    if ((int)Coord2StateIDHashTable[binid].size() > 5)
    {
        SBPL_FPRINTF(fDeb, "WARNING: Hash table has a bin %d of size %d\n",
            binid, (int)Coord2StateIDHashTable[binid].size());

        PrintHashTableHist(fDeb);
    }
#endif

    //iterate over the states in the bin and select the perfect match
    vector<EnvMASNAVXYTHETALATHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
    for (int ind = 0; ind < (int)binV->size(); ind++) {
#if TIME_DEBUG	
      time_gethash += clock()-currenttime;
#endif
      EnvMASNAVXYTHETALATHashEntry_t* hashentry = binV->at(ind);
      bool matchfound = IsEqualHashEntry(hashentry, poses, GoalsVisited);
      if (matchfound)
	return hashentry;
    }
    return NULL;
}

bool EnvironmentMASNAVXYTHETALAT::IsEqualHashEntry(EnvMASNAVXYTHETALATHashEntry_t* hashentry, std::vector<pose_t> poses, std::vector<bool> GoalsVisited)
{
  bool matchfound = true;
  for (int agent_i =0; agent_i <  EnvMASNAVXYTHETALATCfg.NumAgents; agent_i++){
    if ( (hashentry->poses[agent_i].x == poses[agent_i].x) && (hashentry->poses[agent_i].y == poses[agent_i].y) && (hashentry->poses[agent_i].theta == poses[agent_i].theta) && (hashentry->goalsVisited == GoalsVisited)) {
      }
    else {
      matchfound = false;
      break;
    }
  }
  return matchfound;
}

EnvMASNAVXYTHETALATHashEntry_t* EnvironmentMASNAVXYTHETALAT::CreateNewHashEntry_hash(std::vector<pose_t> poses, std::vector<bool> GoalsVisited)
{
    int i;

#if TIME_DEBUG	
    clock_t currenttime = clock();
#endif

    EnvMASNAVXYTHETALATHashEntry_t* HashEntry = new EnvMASNAVXYTHETALATHashEntry_t;

    HashEntry->poses = poses;;
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

void EnvironmentMASNAVXYTHETALAT::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  SBPL_INFO("Undefined");
}

void EnvironmentMASNAVXYTHETALAT::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<
					     EnvMASNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
{
  SBPL_INFO("Undefined");
  /*  
#if TIME_DEBUG
  clock_t currenttime = clock();
#endif
  
  //clear the successor array
  SuccIDV->clear();
  CostV->clear();
  SuccIDV->reserve(EnvMASNAVXYTHETALATCfg.actionwidth);
  CostV->reserve(EnvMASNAVXYTHETALATCfg.actionwidth);
  if (actionV != NULL) {
    actionV->clear();
    actionV->reserve(EnvMASNAVXYTHETALATCfg.actionwidth);
  }

    //goal state should be absorbing
    if (SourceStateID == EnvMASNAVXYTHETALAT.goalstateid) return;

    //get X, Y for the state
    EnvMASNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    //iterate through actions
    for (aind = 0; aind < EnvMASNAVXYTHETALATCfg.actionwidth; aind++) {
        EnvMASNAVXYTHETALATAction_t* nav3daction = &EnvMASNAVXYTHETALATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
        int newY = HashEntry->Y + nav3daction->dY;
        int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, EnvMASNAVXYTHETALATCfg.NumThetaDirs);

        //skip the invalid cells
        if (!IsValidCell(newX, newY)) continue;

        //get cost
        int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if (cost >= INFINITECOST) continue;

        EnvMASNAVXYTHETALATHashEntry_t* OutHashEntry;
        if ((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta)) == NULL) {
            //have to create a new entry
            OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta);
        }

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
        if (actionV != NULL) actionV->push_back(nav3daction);
    }

#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif
  */
}



void EnvironmentMASNAVXYTHETALAT::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
}



void EnvironmentMASNAVXYTHETALAT::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
}

void EnvironmentMASNAVXYTHETALAT::InitializeEnvironment()
{
    EnvMASNAVXYTHETALATHashEntry_t* HashEntry;

    int maxsize = EnvMASNAVXYTHETALATCfg.EnvWidth_c * EnvMASNAVXYTHETALATCfg.EnvHeight_c * EnvMASNAVXYTHETALATCfg.NumThetaDirs;

    
    SBPL_INFO("environment stores states in hashtable\n");
    
    //initialize the map from Coord to StateID
    HashTableSize = 4 * 1024 * 1024; //should be power of two
    Coord2StateIDHashTable = new vector<EnvMASNAVXYTHETALATHashEntry_t*> [HashTableSize];
    GetHashEntry = &EnvironmentMASNAVXYTHETALAT::GetHashEntry_hash;
    CreateNewHashEntry = &EnvironmentMASNAVXYTHETALAT::CreateNewHashEntry_hash;
    
    //not using hash
    Coord2StateIDHashTable_lookup = NULL;

    //initialize the map from StateID to Coord
    StateID2CoordTable.clear();

    //create start state
    std::vector<bool> goalsVisited(EnvMASNAVXYTHETALATCfg.NumGoals,false);
    if ((HashEntry = (this->*GetHashEntry)(EnvMASNAVXYTHETALATCfg.start, goalsVisited)) == NULL) {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvMASNAVXYTHETALATCfg.start, goalsVisited);
    }
    EnvMASNAVXYTHETALAT.startstateid = HashEntry->stateID;

    //create goal state
    if ((HashEntry = (this->*GetHashEntry)(EnvMASNAVXYTHETALATCfg.start, goalsVisited))==NULL) {
        //have to create a new entry
        HashEntry = (this->*CreateNewHashEntry)(EnvMASNAVXYTHETALATCfg.start,
						goalsVisited);
    }
    EnvMASNAVXYTHETALAT.goalstateid = HashEntry->stateID;

    //initialized
    EnvMASNAVXYTHETALAT.bInitialized = true;
}

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>

//TODO: does not use goalsVisited in hash computation
unsigned int EnvironmentMASNAVXYTHETALAT::GETHASHBIN(std::vector<pose_t> poses, std::vector<bool> goalsVisited)
{
  unsigned int hashbin = 0;
  for(int i = 0; i < poses.size(); ++i)
    {
      hashbin += inthash(inthash(poses[i].x) + (inthash(poses[i].y) << 1) + (inthash(poses[i].theta) << 2));
    }
  return inthash(hashbin) & (HashTableSize - 1);
}

void EnvironmentMASNAVXYTHETALAT::PrintHashTableHist(FILE* fOut)
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int j = 0; j < HashTableSize; j++) {
        if ((int)Coord2StateIDHashTable[j].size() == 0)
            s0++;
        else if ((int)Coord2StateIDHashTable[j].size() < 5)
            s1++;
        else if ((int)Coord2StateIDHashTable[j].size() < 25)
            s50++;
        else if ((int)Coord2StateIDHashTable[j].size() < 50)
            s100++;
        else if ((int)Coord2StateIDHashTable[j].size() < 100)
            s200++;
        else if ((int)Coord2StateIDHashTable[j].size() < 400)
            s300++;
        else
            slarge++;
    }
    SBPL_FPRINTF(fOut, "hash table histogram: 0:%d, <5:%d, <25:%d, <50:%d, <100:%d, <400:%d, >400:%d\n", s0, s1, s50,
                 s100, s200, s300, slarge);
}

int EnvironmentMASNAVXYTHETALAT::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  SBPL_INFO("TODO: GetStartHeuristic not define");
  return 0;
  /*
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if(FromStateID >= (int)StateID2CoordTable.size()
        || ToStateID >= (int)StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvMASNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //get X, Y for the state
    EnvMASNAVXYTHETALATHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
    EnvMASNAVXYTHETALATHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];

    //TODO - check if one of the gridsearches already computed and then use it.

    return (int)(MASNAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X,
                                                                    ToHashEntry->Y) /
                 EnvMASNAVXYTHETALATCfg.nominalvel_mpersecs);
    */
}

int EnvironmentMASNAVXYTHETALAT::GetGoalHeuristic(int stateID)
{
  SBPL_INFO("TODO: GetStartHeuristic not define");
  return 0;
  /*
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvMASNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    EnvMASNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    //computes distances from start state that is grid2D, so it is EndX_c EndY_c
    int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); 
    int hEuclid = (int)(MASNAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(HashEntry->X, HashEntry->Y,
                                                                           EnvMASNAVXYTHETALATCfg.EndX_c,
                                                                           EnvMASNAVXYTHETALATCfg.EndY_c));

    //define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D, hEuclid)) / EnvMASNAVXYTHETALATCfg.nominalvel_mpersecs);
      */
}


int EnvironmentMASNAVXYTHETALAT::GetStartHeuristic(int stateID)
{
  /*
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvMASNAVXYTHETALAT... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    EnvMASNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
    int hEuclid = (int)(MASNAVXYTHETALAT_COSTMULT_MTOMM * EuclideanDistance_m(EnvMASNAVXYTHETALATCfg.StartX_c,
                                                                           EnvMASNAVXYTHETALATCfg.StartY_c, HashEntry->X,
                                                                           HashEntry->Y));

    //define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D, hEuclid)) / EnvMASNAVXYTHETALATCfg.nominalvel_mpersecs);
  */
  SBPL_INFO("TODO: GetStartHeuristic not define");
  return 0;
}


int EnvironmentMASNAVXYTHETALAT::SizeofCreatedEnv()
{
    return (int)StateID2CoordTable.size();
}

//---------------------------------------------------------------------------


bool EnvironmentMASNAVXYTHETALAT::isGoal(int id){
  // any state with all goals visited is a goal state
  EnvMASNAVXYTHETALATHashEntry_t* HashEntry = StateID2CoordTable[id];
  for(int i = 0; i < EnvMASNAVXYTHETALATCfg.NumGoals; i++)
    {
      if (!HashEntry->goalsVisited[i])
	return false;
    }
  return true;
}

