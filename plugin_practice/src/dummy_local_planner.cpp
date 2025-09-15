#include "plugin_practice/dummy_local_planner.h"
#include <pluginlib/class_list_macros.h>

namespace plugin_practice{

DummyLocalPlanner::DummyLocalPlanner()
: initialized_(false)
{
}

DummyLocalPlanner::~DummyLocalPlanner()
{
}

void DummyLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)

{
    ROS_WARN("DummyLocalPlanner initialize called");
    if(!initialized_){
        ROS_INFO("DummyLocalPlanner is initialized");
        initialized_ = true;
        tf = tf_;
        costmap_ros_ = costmap_ros;
    }
    else{
        ROS_WARN("DummyLocalPlanner was already initalized"); 
    }
}

bool DummyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) 
{
    if(!initialized_){
        ROS_ERROR("DummyLocalPlanner not initialized");
        return false;
    }
    if (global_plan_.empty()) {
        ROS_WARN("Global Path is emtpy");
        return false;
    }

    geometry_msgs::PoseStamped robot_pose;
    try {
        costmap_ros_->getRobotPose(robot_pose);
    } catch (...) {
        ROS_ERROR("failed to get robot_pose");
        return false;
    }

    geometry_msgs::PoseStamped goal = global_plan_.back();
    double dx = goal.pose.position.x - robot_pose.pose.position.x;
    double dy = goal.pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    double angle_to_goal = std::atan2(dy, dx);

    cmd_vel.linear.x = std::min(0.2, distance);
    cmd_vel.angular.z = angle_to_goal;

    ROS_INFO_THROTTLE(1.0, "DummyLocalPlanner cmd_vel: [%.2f, %.2f]", cmd_vel.linear.x, cmd_vel.angular.z);
    return true;
}

bool DummyLocalPlanner::isGoalReached() 
{
    if(!initialized_  || global_plan_.empty()) return false;

    geometry_msgs::PoseStamped robot_pose;
    try {
        costmap_ros_-> getRobotPose(robot_pose);
    } catch (...) {
        ROS_ERROR("Failed to get robot pose");
        return false;
    }

    geometry_msgs::PoseStamped goal = global_plan_.back();
    double dx = goal.pose.position.x = robot_pose.pose.position.x;
    double dy = goal.pose.position.y = robot_pose.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    return (distance < 0.2);
}



bool plugin_practice::DummyLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    global_plan_ = plan;
    ROS_INFO("DummyLocalPlanner: Received plan with %lu points", plan.size());
    return true;
}

} // end of pluging_practice class

PLUGINLIB_EXPORT_CLASS(plugin_practice::DummyLocalPlanner, nav_core::BaseLocalPlanner)