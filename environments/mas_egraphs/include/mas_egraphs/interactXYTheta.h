#ifndef INTERACT_XY_H
#define INTERACT_XY_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <mas_egraphs/GetXYThetaPlan.h>

class ControlPlanner{
  public:
    void callPlanner();
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    ControlPlanner();

  private:
    interactive_markers::InteractiveMarkerServer* server;
    interactive_markers::MenuHandler menu_handler;
    costmap_2d::Costmap2DROS* planner_costmap_ros_;
    FILE* fout;
    tf::TransformListener tf_;
    boost::thread* planner_thread;
    boost::mutex mutex;
    boost::condition_variable call_planner_cond;
    ros::ServiceClient planner;
    ros::Publisher interrupt_pub;
    ros::Publisher vis_pub_;
    mas_egraphs::GetXYThetaPlan::Request req;
    mas_egraphs::GetXYThetaPlan::Response res;
    int test_num;
    double radius_;
};

#endif
