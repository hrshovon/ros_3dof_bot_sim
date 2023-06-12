search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=first_arm.srdf
robot_name_in_srdf=first_arm
moveit_config_pkg=first_arm_moveit_config
robot_name=first_arm
planning_group_name=bot_arm
ikfast_plugin_pkg=first_arm_bot_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=finger_low
ikfast_output_path=/home/rashid/ros_ws/src/first_arm_bot_arm_ikfast_plugin/src/first_arm_bot_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
