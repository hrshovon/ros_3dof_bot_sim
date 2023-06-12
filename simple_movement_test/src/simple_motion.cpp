#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_common.h>
#include <math.h>


#define LOGGER_TAG "movement test"
#define PLANNING_GROUP_DEF "bot_arm"

void getJointValues(moveit::planning_interface::MoveGroupInterface &move_group_interface,const moveit::core::JointModelGroup* joint_model_group,std::vector<double> &joint_group_positions)
{
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
}



bool planJointMove(moveit::planning_interface::MoveGroupInterface &move_group_interface,const moveit::core::JointModelGroup* joint_model_group,std::vector<double> joint_group_positions)
{
    /*
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    std::vector<double> joint_group_positions_comp;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_comp);
    */
    std::vector<double> joint_group_positions_comp;
    getJointValues(move_group_interface,joint_model_group,joint_group_positions_comp);
    if (joint_group_positions.size()!=joint_group_positions_comp.size())
    {
        ROS_INFO_NAMED(LOGGER_TAG,"ERROR! Provided joint values %d, but there are only %d valid values",joint_group_positions.size(),joint_group_positions_comp.size());
        return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan demo_plan;
    move_group_interface.setJointValueTarget(joint_group_positions);


    bool success = (move_group_interface.plan(demo_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED(LOGGER_TAG, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

    if(success)
    {
        move_group_interface.move();
        return true;
    }   
    return false;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "simplemotion");
    ros::NodeHandle node_handle;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    static const std::string PLANNING_GROUP = "bot_arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    ROS_INFO_NAMED(LOGGER_TAG, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    ROS_INFO_NAMED(LOGGER_TAG, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    ROS_INFO_NAMED(LOGGER_TAG, "Available Planning Groups:");

    std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    std::vector<double> joint_state_goal;
    joint_state_goal.push_back(-12*M_PI/180);
    joint_state_goal.push_back(57*M_PI/180);
    joint_state_goal.push_back(90*M_PI/180);

    planJointMove(move_group_interface, joint_model_group,joint_state_goal);
    
    joint_state_goal[0]=29*M_PI/180;

    planJointMove(move_group_interface, joint_model_group,joint_state_goal);
    

    ros::shutdown();
    return 0;
}