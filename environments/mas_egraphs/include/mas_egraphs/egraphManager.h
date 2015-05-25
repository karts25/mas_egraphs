#ifndef EGRAPH_MANAGER_H
#define EGRAPH_MANAGER_H

#ifndef ROS
#define ROS
#endif 

#include <ros/ros.h>
#include <egraphs/egraph.h>
#include <egraphs/egraphable.h>
#include <egraphs/egraph_heuristic.h>
#include <boost/thread/condition_variable.hpp>
#include <vector>
#include <map>
#include <memory>
#include <string>
#include <limits>
#include <egraphs/planner_state.h>

typedef EGraph* EGraphPtr;
typedef std::vector<std::vector<double> > EGraphPath;
typedef std::vector<int> EGraphCosts;
typedef std::vector<double> ContState;
typedef std::vector<int> DiscState;
typedef std::vector<std::pair<int, int> > SuccessorList;
typedef std::pair<int, int> Edge;

struct EGraphParams {
    bool feedback_path;
    bool update_stats;
};

struct EGraphStats {
    double egraph_validity_check_time;
    double heuristic_computation_time;
    double feedback_time;
    double error_check_time;
    double shortest_path_time;
    double get_direct_shortcut_time;
    double shortcut_time;
    double snap_time;
    double precomp_time;
    int num_snaps;
    double combo_time;
    EGraphStats():
        egraph_validity_check_time(0),
        heuristic_computation_time(0),
        feedback_time(0),
        error_check_time(0),
        shortest_path_time(0),
        shortcut_time(0),
        snap_time(0),
        precomp_time(0),
        num_snaps(0),
        combo_time(0){}
};

struct UpdateEGThreadData {
    EGraphPath path_to_feedback;
    EGraphCosts costs;
    boost::thread* feedback_thread_;
    boost::condition_variable egraph_cond_;
    boost::mutex egraph_mutex_;
    bool planner_ok_;
};

template <typename HeuristicType>
class EGraphManager {
 public:
        typedef EGraphable<HeuristicType>* EGraphablePtr;
        typedef EGraphHeuristic<HeuristicType>* EGraphHeuristicPtr;
	typedef std::vector<std::vector<int>> Matrix;
	EGraphManager(EGraphPtr egraph, EGraphablePtr egraph_env, EGraphHeuristicPtr egraph_heur, int numgoals, int numagents);
	void setEpsE(double epsE);
	bool setGoal();
	void updateManager();
	void allocateHeuristics();
	int getHeuristic(int state_id);
	void updateHeuristicGrids(const std::vector<std::vector<bool> >& grid);
	void printVector(std::vector<int>& v); 
	void getSnapSuccessors(int stateID, std::vector<int>* SuccIDV, 
                               std::vector<int>* CostV, std::vector<bool>* isTrueCost, 
                               std::vector<EdgeType>* edgeTypes);
        int getSnapTrueCost(int parentID, int childID);
        void getDirectShortcutSuccessors(int stateID, std::vector<int>* SuccIDV, 
                                         std::vector<int>* CostV, std::vector<bool>* isTrueCost,
                                         std::vector<EdgeType>* edgeTypes);
        void getComboSnapShortcutSuccessors(int stateID, std::vector<int>* SuccIDV, 
                                            std::vector<int>* CostV, std::vector<bool>* isTrueCost);

        void getSnapShortcuts(int stateID, 
                              std::vector<int>* SuccIDV, 
                              std::vector<int>* CostV, 
                              std::vector<bool>* isTrueCost,
                              std::vector<EdgeType>* edgeTypes,
                              std::vector<int>* snap_midpoints);
        int getSnapShortcutTrueCost(int parentID, int snap_midpoint, int childID);
        bool reconstructSnapShortcut(LazyAEGState* state, LazyAEGState*& next_state,
                                     std::vector<int>* wholePathIds, std::vector<int>* costs,
                                     int& totalCost);

        bool reconstructDirectShortcuts(LazyAEGState* state, 
                                           LazyAEGState*& next_state, 
                                           std::vector<int>* wholePathIds, 
                                           std::vector<int>* costs, int& shortcut_count,
                                           int& totalCost);

        bool reconstructSnap(LazyAEGState* state, LazyAEGState*& next_state, 
                             std::vector<int>* wholePathIds, std::vector<int>* costs);

        bool reconstructComboSnapShortcut(LazyAEGState* state, LazyAEGState*& next_state, 
                                          std::vector<int>* wholePathIds, std::vector<int>* costs, 
                                          int goal_id);

        void storeLastPath(const std::vector<int>& path, const std::vector<int>& costs);
        void feedbackLastPath();
        EGraphStats getStats(){ return stats_; };
        void save(std::string filename){ egraph_->save(filename); };
	void validateEGraph(bool update_egraph=true);
        void initEGraph(bool set_goal=true);	
	EGraphablePtr egraph_env_;
       
 private:
	int numgoals_;
	int numagents_;
	std::vector<int> allgoals_coord_;
	//vector of distance matrices between "cities" for TSP
	std::vector<Matrix> TSPEdgecosts_; // vector of 2d distance matrices, one per agent
	std::vector<EGraphPtr> egraphperagent_;
	// this is specifically for combosnaps. because a snap combo edge is
        //      source->[snap]->snap_id->[shortcut]->successor, 
        // this cache holds:
        //      (source, successor)->(snap_id, cost)
        //std::map<Edge, std::pair<int, int> > snap_combo_cache_;

        // snap successors is called twice. this is the cache for the results
        std::vector<int> snap_successors_cache_;
        std::vector<int> snap_costs_cache_;
        EGraphParams params_;
	EGraphPtr egraph_;
	EGraphHeuristicPtr egraph_heur_;
	std::vector<std::vector<EGraphHeuristicPtr> > egraph_heurs_;
	std::vector<EGraphHeuristicPtr> egraph_heurs_vec_;
        UpdateEGThreadData update_eg_thread_data_;
        EGraphStats stats_;

	// helper function that returns the coords of goal # i
	void getGoalCoord(int i, std::vector<int>& coord);
	// call TSP solver
	int solveTSP(int agent_i, std::vector<int>& goalindices);
	void computeGoalDistances(int agent_i);
	void segmentEGraph();
	int getHeuristicPerAgent(int state_id, int agent_i, std::vector<int>& heur_coord_agent,
				 std::vector<int>& assignment);

	void errorCheckEGraphVertex(EGraph::EGraphVertex* vertex);
        std::vector<int> getDirectShortcutStateIDs(int start_id, int end_id,
						   std::vector<int>* costs);

        void fillInDirectShortcut(int start_id, int end_id,
                                  std::vector<int>* wholePathIds, 
                                  std::vector<int>* costs, int& shortcut_count);

        void printVector(std::vector<double>& v); 

	DiscState getDiscStateFromID(int state_id);
};
#include<mas_egraphs/../../src/egraphManager.hpp>

#endif
