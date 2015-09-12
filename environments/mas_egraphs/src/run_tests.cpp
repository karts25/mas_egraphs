#include<ros/ros.h>
#include<mas_egraphs/GetXYThetaPlan.h>
#include <mas_egraphs/egraph_stat_writer.h>

int main(int argc, char** argv){
  if(argc < 2){
    printf("provide a path to a test file!\n");
    return 1;
  }
  ros::init(argc, argv, "run_tests");
  ros::NodeHandle nh;

  //mas_egraphs::GetXYThetaPlan::Request req;
  //mas_egraphs::GetXYThetaPlan::Response res;
  mas_egraphs::GetXYThetaPlan plan_req_msg;
  //egraph and planner parameters
  ros::Publisher plan_req_pub = nh.advertise<mas_egraphs::GetXYThetaPlan>("mas_egraphs/mas_plan_req", 1);
  ros::Subscriber plan_stats_sub = nh.subscribe("/mas_egraphs/mas_stats", 1,
                                                &writeStatsCallback, this);
  sleep(1);
  
  FILE* fin = fopen(argv[1],"r");
  if(!fin){
    printf("file %s does not exist\n", argv[1]);
    return 1;
  }
  fscanf(fin,"experiments:\n\n");

  double start_x, start_y, start_z, start_theta, goal_x, goal_y, goal_z, goal_theta;
  while(1){  
    plan_req_msg.start_x.clear();
    plan_req_msg.start_y.clear();
    plan_req_msg.start_z.clear();
    plan_req_msg.start_theta.clear();
    plan_req_msg.goal_x.clear();
    plan_req_msg.goal_y.clear();
    plan_req_msg.goal_z.clear();
    plan_req_msg.goal_theta.clear();
    int test_num = 0;
    if(fscanf(fin,"  - test: %d\n", &test_num) <= 0)
      break;
    if(fscanf(fin,"    eps_comm: %lf\n", &plan_req_msg.eps_comm) <= 0)
      break;
    if(fscanf(fin,"    num_agents: %d\n", &plan_req_msg.num_agents) <=0)
      break;
    if(fscanf(fin,"    num_goals: %d\n", &plan_req_msg.num_goals) <=0)
      break;
    printf("num_agents %d num_goals %d eps_comm %f \n", plan_req_msg.num_agents, plan_req_msg.num_goals, plan_req_msg.eps_comm);
    for(int agent_i = 0; agent_i < plan_req_msg.num_agents; agent_i++){
      if(fscanf(fin,"    start: %lf %lf %lf %lf\n", 
		&start_x, &start_y, &start_z, &start_theta) <= 0)
	break;
	plan_req_msg.start_x.push_back(start_x);
	plan_req_msg.start_y.push_back(start_y);
	plan_req_msg.start_z.push_back(start_z);
	plan_req_msg.start_theta.push_back(start_theta);
      }
    for(int i = 0; i < plan_req_msg.num_goals; i++){
      if(fscanf(fin,"    goal: %lf %lf %lf %lf\n",
		&goal_x, &goal_y, &goal_z, &goal_theta) <= 0)
	break;
      plan_req_msg.goal_x.push_back(goal_x);
      plan_req_msg.goal_y.push_back(goal_y);
      plan_req_msg.goal_z.push_back(goal_z);
      plan_req_msg.goal_theta.push_back(goal_theta);
    }
    plan_req_pub.publish(plan_req_msg);
    //EGraphStatWriter::writeStatsToFile("mas_egraphs_stats.csv", first, res.stat_names, res.stat_values);
  }
  
  ros::spin();
  return 0;
}


void writeStatsToFile(const mas_egraphs::MasStats::ConstPtr &msg){
    
}
