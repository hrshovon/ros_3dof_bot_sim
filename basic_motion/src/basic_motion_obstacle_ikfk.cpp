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


#define LOGGER_TAG "ros_move_basic"
#define PLANNING_GROUP_DEF "bot_arm"

void getJointValues(moveit::planning_interface::MoveGroupInterface &move_group_interface,const moveit::core::JointModelGroup* joint_model_group,std::vector<double> &joint_group_positions)
{
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
}


bool detecSelfCollision(planning_scene::PlanningScene &planning_scene)
{
    //self collision check
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    return collision_result.collision;
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


bool planDemoJointMove(moveit::planning_interface::MoveGroupInterface &move_group_interface,const moveit::core::JointModelGroup* joint_model_group)
{   
    std::vector<double> joint_moves;
    getJointValues(move_group_interface,joint_model_group,joint_moves);
    for(int i=0;i<joint_moves.size();i++)
    {
        joint_moves[i]=0.78;
    
    }
    return planJointMove(move_group_interface,joint_model_group,joint_moves);
}

void addColObject(moveit::planning_interface::MoveGroupInterface &move_group_interface,moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,std::string obj_name, double size[3], double pos[4])
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = obj_name;

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = size[0];
    primitive.dimensions[primitive.BOX_Y] = size[1];
    primitive.dimensions[primitive.BOX_Z] = size[2];

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = pos[0];
    box_pose.position.x = pos[1];
    box_pose.position.y = pos[2];
    box_pose.position.z = pos[3];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO_NAMED(LOGGER_TAG, "Add an object into the world with name %s",obj_name.c_str());
    planning_scene_interface.addCollisionObjects(collision_objects);
}

void get_robot_ik_and_fk(const moveit::core::RobotModelPtr& kinematic_model)
{
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP_DEF);

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // Joint Limits
    // ^^^^^^^^^^^^
    // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
    /* Set one joint in the Panda arm outside its joint limit */
    joint_values[0] = 5.57;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    /* Check whether any joint is outside its joint limits */
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    /* Enforce the joint limits for this state and check again*/
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    // Forward Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // Now, we can compute forward kinematics for a set of random joint
    // values. Note that we would like to find the pose of the
    // "panda_link8" which is the most distal link in the
    // "panda_arm" group of the robot.
    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("arm");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    // Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // We can now solve inverse kinematics (IK) for the Panda robot.
    // To solve IK, we will need the following:
    //
    //  * The desired pose of the end-effector (by default, this is the last link in the "panda_arm" chain):
    //    end_effector_state that we computed in the step above.
    //  * The timeout: 0.1 s
    double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bot_mover_basic_node");
    ros::NodeHandle node_handle;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    static const std::string PLANNING_GROUP = PLANNING_GROUP_DEF;

    //initiate movegroupinterface 
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    //initial planningsceneinterface, planning scene and robot model loader
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    ROS_INFO_NAMED(LOGGER_TAG, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    ROS_INFO_NAMED(LOGGER_TAG, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    ROS_INFO_NAMED(LOGGER_TAG, "Available Planning Groups:");

    std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


    
    //demo, detect self collision
    bool selfCollision = detecSelfCollision(planning_scene);
    ROS_INFO_STREAM("Current state is " << (selfCollision ? "in" : "not in") << " self collision");
    

    ROS_INFO_NAMED(LOGGER_TAG, "Planning and executing a demo joint move");

    bool success = planDemoJointMove(move_group_interface, joint_model_group);
    if (success)
    {
        ROS_INFO_NAMED(LOGGER_TAG, "Demo joint move successful");
    }

    std::vector<double> joint_states;
    getJointValues(move_group_interface,joint_model_group,joint_states);

    for(int i=0;i<joint_states.size();i++)
    {
        ROS_INFO_NAMED(LOGGER_TAG,"joint_%d=%lf",i,joint_states[i]);
    }

    ROS_INFO_NAMED(LOGGER_TAG, "Calculating ik and fk");

    //robot_ik_and_fk
    get_robot_ik_and_fk(kinematic_model);


    ROS_INFO_NAMED(LOGGER_TAG, "Adding obstacles");



    
    double box_dims[] = {0.1,1.5,0.3};
    double box_pos_orientation[] = {1.0,0.5,0.0,0.25};

    addColObject(move_group_interface,planning_scene_interface,"box1",box_dims,box_pos_orientation);
    
    /*
    part that does not work
    Eigen::Isometry3d box_pose{ Eigen::Isometry3d::Identity() };
    box_pose.translation().x() = 1.0;
    box_pose.translation().y() = 0.5;
    box_pose.translation().z() = 0.7;

    auto box = std::make_shared<shapes::Box>(0.1,1.5,0.3);
    planning_scene.getWorldNonConst()->addToObject("box", box, box_pose);

    robot_state::RobotState& state = planning_scene.getCurrentStateNonConst();
    state.setToDefaultValues();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = move_group_interface.getPlanningFrame();
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = true;
    planning_scene.checkCollision(c_req, c_res, *kinematic_state);
    if (c_res.collision)
    {
        ROS_INFO_NAMED(LOGGER_TAG,"collision detected!");
    }
    
    */
    ros::shutdown();
    return 0;
}