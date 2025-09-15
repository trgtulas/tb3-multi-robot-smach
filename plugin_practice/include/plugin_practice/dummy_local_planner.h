
#ifndef DUMMY_LOCAL_PLANNER_H
#define DUMMY_LOCAL_PLANNER_H

#include <memory.h>
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace plugin_practice
{
class DummyLocalPlanner : public nav_core::BaseLocalPlanner {
public:
    DummyLocalPlanner(); 
    ~DummyLocalPlanner();

    virtual void initialize(std::string name, tf2_ros::Buffer* tf, 
                    costmap_2d::Costmap2DROS* costmap_ros) override;
    
    virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    
    virtual bool isGoalReached() override;

    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    bool initialized_;
    std::vector<geometry_msgs::PoseStamped> global_plan_; // to access the global plan
    tf2_ros::Buffer* tf_; // to access tf 
    costmap_2d::Costmap2DROS* costmap_ros_; // to access costmap data

}; //end of DummyLocalPlanner class

} // end of my_namespace

#endif