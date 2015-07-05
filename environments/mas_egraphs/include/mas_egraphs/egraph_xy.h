#ifndef EGRAPH_XY_H
#define EGRAPH_XY_H

#ifndef ROS
#define ROS
#endif

#include<mas_egraphs/key.h>
#include<egraphs/egraph.h>
#include<egraphs/egraphable.h>
#include<mas_egraphs/egraph_mas_2d_grid_heuristic.h>
#include<egraphs/egraph_discretize.h>
#include<sbpl/headers.h>
#include<egraph_vis/egraph_visualizer.h>
#include<mas_egraphs/environment_xy.h>

class EGraphXY:public Environment_xy, public EGraphable<std::vector<int> >, public EGraphMarkerMaker, public EGraphDiscretize{
  public:
    EGraphXY();
    bool collisionCheckPose(int x, int y, int z, int theta, int& cost);
    bool snap(const std::vector<double>& from, const std::vector<double>& to, int& id, int& cost);
    virtual bool getCoord(int id, std::vector<double>& coord);
    virtual int getStateID(const std::vector<double>& coord);
    virtual bool isGoal(int id);
    //virtual int GetNumGoals() const;
    //virtual int GetNumAgents() const;
    void projectToHeuristicSpace(const std::vector<double>& coord, std::vector<int>& dp) const; 
    void projectGoalToHeuristicSpace(std::vector<int>& dp) const; 
    void contToDisc (const std::vector<double>& c, std::vector<int>& d);
    void discToCont(const std::vector<int>& d, std::vector<double>& c);
    void getAssignments(int solution_stateID, std::vector<int>& assignments) const;
    virtual bool isValidEdge(const std::vector<double>& coord, const std::vector<double>& coord2, bool& change_cost, int& cost);
    virtual bool isValidVertex(const std::vector<double>& coord);

    bool InitializeEnv(int width, int height,
		       const unsigned char* mapdata,
		       int numagents, //int numgoals,
		       //std::vector<pose_cont_t> start,
		       //std::vector<pose_cont_t> goal,
		       double goaltol_x, double goaltol_y, double goaltol_theta,
		       const std::vector<std::vector<sbpl_2Dpt_t> > & perimeterptsV,
		       double cellsize_m, double time_per_action,
		       const std::vector<std::string> sMotPrimFiles);

    visualization_msgs::MarkerArray stateToVisualizationMarker(std::vector<double> coord);
    visualization_msgs::MarkerArray stateToDetailedVisualizationMarker(std::vector<double> coord);
    visualization_msgs::MarkerArray edgeToVisualizationMarker(std::vector<double> coord, std::vector<double> coord2);
};

#endif
