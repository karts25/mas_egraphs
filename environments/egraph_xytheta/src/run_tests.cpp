#include<ros/ros.h>
#include<egraph_xytheta/GetXYThetaPlan.h>
#include <egraphs/egraph_stat_writer.h>

int main(int argc, char** argv){
  if(argc < 2){
    printf("provide a path to a test file!\n");
    return 1;
  }
  ros::init(argc,argv,"run_tests");
  ros::NodeHandle nh;

  egraph_xytheta::GetXYThetaPlan::Request req;
  egraph_xytheta::GetXYThetaPlan::Response res;

  //egraph and planner parameters
  req.initial_eps = 1.2;
  req.final_eps = 1.2;
  req.dec_eps = 0.2;
  req.feedback_path = true;
  req.save_egraph = true;
  req.use_egraph = true;

  ros::service::waitForService("/sbpl_planning/plan_path",10);
  ros::ServiceClient planner = ros::NodeHandle().serviceClient<egraph_xytheta::GetXYThetaPlan>("/sbpl_planning/plan_path", true);
  sleep(1);

  FILE* fin = fopen(argv[1],"r");
  if(!fin){
    printf("file %s does not exist\n", argv[1]);
    return 1;
  }
  fscanf(fin,"experiments:\n\n");

  bool first = true;
  while(1){
    int test_num = 0;
    if(fscanf(fin,"  - test: test_%d\n", &test_num) <= 0)
      break;
    if(fscanf(fin,"    start: %lf %lf %lf\n", &req.start_x, &req.start_y, &req.start_theta) <= 0)
      break;
    if(fscanf(fin,"    goal: %lf %lf %lf\n\n", &req.goal_x, &req.goal_y, &req.goal_theta) <= 0)
      break;

    planner.call(req,res);
    EGraphStatWriter::writeStatsToFile("egraph_xytheta_stats.csv", first, res.stat_names, res.stat_values);
    first = false;
  }

  return 0;
}

