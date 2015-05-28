#include<ros/ros.h>
#include<mas_egraphs/GetXYThetaPlan.h>
#include <mas_egraphs/egraph_stat_writer.h>

int main(int argc, char** argv){
  if(argc < 2){
    printf("provide a path to a test file!\n");
    return 1;
  }
  ros::init(argc,argv,"run_tests");
  ros::NodeHandle nh;

  mas_egraphs::GetXYThetaPlan::Request req;
  mas_egraphs::GetXYThetaPlan::Response res;

  //egraph and planner parameters
  req.egraph_eps = 1.0;
  req.final_egraph_eps = 1.0;
  req.dec_egraph_eps = 1.0;
  req.initial_eps = 1.0;
  req.final_eps = 1.0;
  req.dec_eps = 0;
  req.feedback_path = true;
  req.save_egraph = false;
  req.use_egraph = true;
  
  ros::service::waitForService("/sbpl_planning/plan_path",10);
  ros::ServiceClient planner = ros::NodeHandle().serviceClient<mas_egraphs::GetXYThetaPlan>("/sbpl_planning/plan_path", true);
  sleep(1);

  FILE* fin = fopen(argv[1],"r");
  if(!fin){
    printf("file %s does not exist\n", argv[1]);
    return 1;
  }
  fscanf(fin,"experiments:\n\n");

  bool first = true;
  double start_x, start_y, start_theta, goal_x, goal_y, goal_theta;
  while(1){
    req.start_x.clear();
    req.start_y.clear();
    req.start_theta.clear();
    req.goal_x.clear();
    req.goal_y.clear();
    req.goal_theta.clear();
    int test_num = 0;
    if(fscanf(fin,"  - test: test_%d\n", &test_num) <= 0)
      break;
    if(fscanf(fin,"    num_agents: %d\n", &req.num_agents) <=0)
      break;
    if(fscanf(fin,"    num_goals: %d\n", &req.num_goals) <=0)
      break;
    
    printf("num_agents %d num_goals %d\n", req.num_agents, req.num_goals);
    for(int agent_i = 0; agent_i < req.num_agents; ++agent_i)
      {
	if(fscanf(fin,"    start: %lf %lf %lf\n", &start_x, &start_y, &start_theta) <= 0)
	  break;
	req.start_x.push_back(start_x);
	req.start_y.push_back(start_y);
	req.start_theta.push_back(start_theta);
      }
    for(int i = 0; i < req.num_goals; i++){
      if(fscanf(fin,"    goal: %lf %lf %lf\n", &goal_x, &goal_y, &goal_theta) <= 0)
	  break;
      req.goal_x.push_back(goal_x);
      req.goal_y.push_back(goal_y);
      req.goal_theta.push_back(goal_theta);
    }
    planner.call(req,res);
    EGraphStatWriter::writeStatsToFile("mas_egraphs_stats.csv", first, res.stat_names, res.stat_values);
    first = false;
  }
  return 0;
}

