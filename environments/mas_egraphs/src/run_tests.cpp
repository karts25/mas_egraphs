#include<ros/ros.h>
#include<mas_egraphs/GetXYThetaPlan.h>
#include<mas_egraphs/MasStats.h>
#include <mas_egraphs/egraph_stat_writer.h>

// terrible global variables
namespace run_tests{
    int test_id; 
    int num_agents;
    int num_goals;
    std::string out_filename;
    std::vector<bool> plan_done;
    double eps_comm;
} // end namespace run_tests

void writeStatsToFile(const mas_egraphs::MasStats::ConstPtr &msg){
    std::string out_filename_agent = run_tests::out_filename + "_agent" +  std::to_string(msg->agentID) + ".csv";  
    FILE* fout = fopen(out_filename_agent.c_str(), "a");    
    fprintf(fout, "%d,%d,%d,%.2f,%.2f,%d", msg->agentID, run_tests::test_id, msg->success, run_tests::eps_comm, 
            msg->total_time_s, msg->num_packets_peragent);
    for(unsigned int i = 0; i < msg->plan_times_s.size(); i++)
        {
            fprintf(fout, ",%.2f",msg->plan_times_s[i]);
        }
    fprintf(fout,"\n");
    fclose(fout);
    std::cout<<"Agent "<< msg->agentID <<": writing stats for test "<<run_tests::test_id<<std::endl;
    run_tests::plan_done[msg->agentID] = true;
}


int main(int argc, char** argv){
    if(argc < 3){
        printf("usage: run_tests <filename> <num_agents>\n");
        return 1;
    }
    ros::init(argc, argv, "run_tests");
    ros::NodeHandle nh;

    mas_egraphs::GetXYThetaPlan plan_req_msg;
    //egraph and planner parameters
    ros::Publisher plan_req_pub = nh.advertise<mas_egraphs::GetXYThetaPlan>("mas_egraphs/mas_plan_req", 1);
    ros::Subscriber plan_stats_sub = nh.subscribe("/mas_egraphs/mas_stats", 10,
                                                  &writeStatsToFile);
    sleep(1);
  
    std::string in_filename(argv[1]);
    FILE* fin = fopen(in_filename.c_str(),"r");
    if(!fin){
        printf("file %s does not exist\n", argv[1]);
        return 1;
    }

    std::size_t out_filename_startindex = in_filename.find_last_of("/\\");
    std::size_t out_filename_endindex = in_filename.find_last_of(".");
    run_tests::out_filename = in_filename.substr(out_filename_startindex+1, out_filename_endindex);
    run_tests::out_filename = "results/" + run_tests::out_filename;

    // Set values to sweep eps
    std::vector<double> eps_comm_values;
    eps_comm_values.push_back(1);
    eps_comm_values.push_back(5);
    eps_comm_values.push_back(10);    

    fscanf(fin,"experiments:\n\n");
    double start_x, start_y, start_z, start_theta, goal_x, goal_y, goal_z, goal_theta;

    run_tests::num_agents = atoi(argv[2]);
    for(int agent_i = 0; agent_i < run_tests::num_agents; agent_i++){
        std::string out_filename_agent = run_tests::out_filename + "_agent"
            +  std::to_string(agent_i) + ".csv";          
        FILE* fout = fopen(out_filename_agent.c_str(), "w");    
        fprintf(fout, "agent_id, test_id, success, eps_comm, total_execution_time, #packets_sent, plan_times\n");
        fclose(fout);
    }


    while(fin)
        { 
        plan_req_msg.start_x.clear();
        plan_req_msg.start_y.clear();
        plan_req_msg.start_z.clear();
        plan_req_msg.start_theta.clear();
        plan_req_msg.goal_x.clear();
        plan_req_msg.goal_y.clear();
        plan_req_msg.goal_z.clear();
        plan_req_msg.goal_theta.clear();                      
        
        if(fscanf(fin,"  - test: test_%d\n", &run_tests::test_id) <= 0)
                break;                    
        //if(fscanf(fin,"    eps_comm: %lf\n", &plan_req_msg.eps_comm) <= 0)
        //    break;
        //if(fscanf(fin,"    num_agents: %d\n", &plan_req_msg.num_agents) <=0)
        //    break;
        if(fscanf(fin,"    num_goals: %d\n", &plan_req_msg.num_goals) <=0)
            break;
        for(int agent_i = 0; agent_i < run_tests::num_agents; agent_i++){
            if(fscanf(fin,"    start: %lf %lf %lf %lf\n", 
                      &start_x, &start_y, &start_z, &start_theta) <= 0)
                break;
            plan_req_msg.start_x.push_back(start_x);
            plan_req_msg.start_y.push_back(start_y);
            plan_req_msg.start_z.push_back(start_z);
            plan_req_msg.start_theta.push_back(start_theta);
        }
        for(int i = 0; i < plan_req_msg.num_goals; i++){
            if(fscanf(fin,"    goal: %lf %lf %lf\n",
                      &goal_x, &goal_y, &goal_z) <= 0)
                break;
            plan_req_msg.goal_x.push_back(goal_x);
            plan_req_msg.goal_y.push_back(goal_y);
            plan_req_msg.goal_z.push_back(goal_z);
            plan_req_msg.goal_theta.push_back(goal_theta);
        }      

        for(int eps_comm_i = 0; eps_comm_i < eps_comm_values.size(); eps_comm_i ++){                        
            run_tests::eps_comm = eps_comm_values[eps_comm_i];
            plan_req_msg.eps_comm = run_tests::eps_comm;
            run_tests::plan_done.clear();
            run_tests::plan_done.resize(run_tests::num_agents, false);        

            std::cout<<"test "<<run_tests::test_id<<", num_agents "<< run_tests::num_agents
                     <<" num_goals "<<plan_req_msg.num_goals << " eps_comm "<< plan_req_msg.eps_comm << std::endl;
            plan_req_pub.publish(plan_req_msg);
            while((!std::all_of(run_tests::plan_done.begin(), run_tests::plan_done.end(), [](bool i){return i;} ))
                  && (ros::ok()))        
                {                                
                    ros::spinOnce();
                    ros::Duration(0.5).sleep();        
                }
            std::cout<<"done" << std::endl;        
        }
        if(!ros::ok())
            break;       
    }
    if(!fin)
        printf("All tests done\n");
    else if (!ros::ok())
        printf("Terminated by user\n");

    fclose(fin);    
    return 0;
}


