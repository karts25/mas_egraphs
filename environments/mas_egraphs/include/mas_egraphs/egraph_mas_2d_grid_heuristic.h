#ifndef EGRAPH_MAS_2D_GRID_HEURISTIC_H
#define EGRAPH_MAS_2D_GRID_HEURISTIC_H

#ifndef ROS
#define ROS
#endif

#include <mas_egraphs/egraph_heuristic.h>
#include <mas_egraphs/egraph_xy.h>
#include <mas_egraphs/mas_egraph.h>
#include <sbpl/headers.h>
#include <map>

class EGraphMAS2dGridHeuristic : public EGraphHeuristic<std::vector<int> >{
  public:
  EGraphMAS2dGridHeuristic(const EGraphXY& env, 
			   int size_x, int size_y, int move_cost);
  void setAgentId(int agentid);
  void setGrid(const std::vector<std::vector<bool> >& grid);
  void setGoal(const std::vector<int>& goal);
  int getHeuristic(const std::vector<int>& coord);
  void getEGraphVerticesWithSameHeuristic(const std::vector<int>& coord, 
					  std::vector<EGraph::EGraphVertex*>& vertices);
  void runPrecomputations();
  void getDirectShortcut(int component, std::vector<EGraph::EGraphVertex*>& shortcuts);
  virtual void resetShortcuts();
  virtual EGraphHeuristic* clone() const
  {
    return new EGraphMAS2dGridHeuristic(*this);
  }
  protected:
    class EGraphMAS2dGridHeuristicCell: public AbstractSearchState{
      public:
        EGraphMAS2dGridHeuristicCell(){
          open_iteration = 0;
          closed_iteration = 0;
        };
        ~EGraphMAS2dGridHeuristicCell(){};
          
        int open_iteration;
        int closed_iteration;
        int id;
        int cost;
        std::vector<EGraph::EGraphVertex*> egraph_vertices;
    };

    int iteration_;
    int agentid_;
    int sizex_;
    int sizey_;
    int width_;
    int height_;
    int planeSize_;
    int cost_1_move_;
    int inflated_cost_1_move_;
    CHeap heap;
    CHeap sc_heap;
    std::vector<int> goal_dp_;

    std::vector<bool> empty_components_;
    std::vector<EGraph::EGraphVertex*> shortcut_cache_;
    std::vector<EGraphMAS2dGridHeuristicCell> heur;
    std::vector<EGraphMAS2dGridHeuristicCell> sc;
    const EGraphXY& env_;
};

#endif
