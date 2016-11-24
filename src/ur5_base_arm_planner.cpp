/**
 * @file ur5_base_arm_planner.cpp
 * @brief Replacement (lighter) of move_group
 */
#include <moveit/move_group/capability_names.h>
#include <moveit/macros/console_colors.h>
#include <boost/tokenizer.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <achq/ur5_base_arm_planner.h>


/**
 * @function ur5_base_arm_planner
 * @brief Constructor
 */
ur5_base_arm_planner::ur5_base_arm_planner( const planning_scene_monitor::PlanningSceneMonitorPtr &_psm ) :

  mPlanning_scene_monitor(_psm),
  mNh("~") {

  // Allow trajectory execution, false for debug
  mContext.reset( new move_group::MoveGroupContext( _psm, true, false ) ); 
  // Start capabilities
  configureCapabilities();
}

/**
 * @function configureCapabilities
 */
void ur5_base_arm_planner::configureCapabilities() {
  
  try {
    mCapability_plugin_loader.reset( new pluginlib::ClassLoader<move_group::MoveGroupCapability>("moveit_ros_move_group", "move_group::MoveGroupCapability" ) );
  } catch( pluginlib::PluginlibException& _ex ) {
    ROS_FATAL_STREAM("Exception while creating plugin loader for move_group capabilities:" << _ex.what() );
    return;
  }

  // Add scene capability
  std::string capability_plugins;
  if( mNh.getParam("capabilities", capability_plugins ) ) {
    boost::char_separator<char>sep(" ");
    boost::tokenizer<boost::char_separator<char> > tok( capability_plugins, sep );
    for( boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin(); beg!=tok.end(); ++beg ) {
      std::string plugin = *beg;
      try {
	printf( MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'...\n" MOVEIT_CONSOLE_COLOR_RESET,
		plugin.c_str() );
	move_group::MoveGroupCapability* cap = mCapability_plugin_loader->createUnmanagedInstance(plugin);
	cap->setContext(mContext);
	cap->initialize();
	mCapabilities.push_back( boost::shared_ptr<move_group::MoveGroupCapability>(cap) );
      } catch( pluginlib::PluginlibException& _ex ) {
	ROS_ERROR_STREAM("Exception while loading move_group capability" << plugin<<":"<< _ex.what() << std::endl );
      }
    } // end for
  }
  printf("** Finished loading capabilities!\n");
}

/**
 * @function getCurrentState
 */
moveit::core::RobotState ur5_base_arm_planner::getCurrentState( ) {
  
  planning_scene_monitor::LockedPlanningSceneRO psro = planning_scene_monitor::LockedPlanningSceneRO( mPlanning_scene_monitor );
  
  return psro->getCurrentState();
}


/**
 * @function isStateColliding
 */
bool ur5_base_arm_planner::isStateColliding( const moveit::core::RobotState& _state, 
					     const std::string& _group ) {
  
  planning_scene_monitor::LockedPlanningSceneRO psro = planning_scene_monitor::LockedPlanningSceneRO( mPlanning_scene_monitor );
  return psro->isStateColliding( _state, _group, false ); // false: debug
}

/**
 * @function checkPathNonCollision
 * @brief Check if the path entered collides with the present environment. True if no collision, false if it collides
 * at any point
 */
bool ur5_base_arm_planner::checkPathNonCollision( moveit::core::RobotState _start_state, 
						  const trajectory_msgs::JointTrajectory &_traj,
						  std::string _group ) {
  
  moveit::core::RobotState checked_state( _start_state );
  for( int i = 0; i < _traj.points.size(); ++i ) {
    // Get a state with the point
    checked_state.setJointGroupPositions( _start_state.getJointModelGroup(_group), 
					  _traj.points[i].positions );
    checked_state.update();
    if( isStateColliding( checked_state, _group ) ) {
      ROS_WARN("** Trajectory collides in point %d/%d! \n", i, _traj.points.size() ); return false;
    }
  }

  return true;
}


/**
 * @function executeTraj
 */
bool ur5_base_arm_planner::executeTraj( const moveit_msgs::RobotTrajectory &_traj ) {

  ros::NodeHandle n;
  moveit_msgs::ExecuteKnownTrajectory::Request req;
  moveit_msgs::ExecuteKnownTrajectory::Response res;
  req.trajectory = _traj;
  req.wait_for_execution = true;
  ros::ServiceClient execute_service_;
  execute_service_ = n.serviceClient<moveit_msgs::ExecuteKnownTrajectory>( move_group::EXECUTE_SERVICE_NAME );
  usleep(0.5*1e6);
  
  return ( execute_service_.call( req, res ) == moveit_msgs::MoveItErrorCodes::SUCCESS );

}

/**
 * @function planTargetPose
 */
bool ur5_base_arm_planner::planTargetPose( const moveit::core::RobotState &_start_state,
					   const std::string &_group_name,
					   const std::vector<geometry_msgs::Pose> &_target_poses,
					   const std::string &_frame_id,
					   const std::string &_end_effector,
					   moveit_msgs::RobotTrajectory &_traj,
					   double &_planning_time,
					   const double &_speed_rate ) {

  boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > mac;
  ros::NodeHandle n;
  mac.reset(new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>( n,
									     move_group::MOVE_ACTION,	
									     false));
  
  ros::Time final_time = ros::Time::now() + ros::Duration(5.0);
  while (mNh.ok() && !mac->isServerConnected() && final_time > ros::Time::now())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  if( !mac->isServerConnected() ) { ROS_ERROR("Planner server NOT connected"); return false; }
  moveit_msgs::MoveGroupGoal goal;
  goal.request.group_name = _group_name;
  goal.request.num_planning_attempts = 5;
  goal.request.max_velocity_scaling_factor = _speed_rate;
  goal.request.allowed_planning_time = 5.0;
  goal.request.planner_id = "";
  //goal.request.workspace_parameters = ;
  moveit::core::robotStateToRobotStateMsg( _start_state, goal.request.start_state );
  // Target pose
  double goal_position_tolerance = 0.03;//1e-4;
  double goal_orientation_tolerance = M_PI*1.0; //1e-3;
  goal.request.goal_constraints.resize( 1 );
  for( int i = 0; i < _target_poses.size(); ++i ) {
    geometry_msgs::PoseStamped tg;
    tg.pose = _target_poses[i];
    tg.header.stamp = ros::Time::now();
    tg.header.frame_id = _frame_id;
    moveit_msgs::Constraints c = kinematic_constraints::constructGoalConstraints( _end_effector,
										  tg,
										  goal_position_tolerance,
										  goal_orientation_tolerance );
    goal.request.goal_constraints[0] = kinematic_constraints::mergeConstraints( goal.request.goal_constraints[i], c );
  }

  goal.planning_options.plan_only = true;
  goal.planning_options.look_around = false; // what is this?
  goal.planning_options.replan = false; // what is this?
  goal.planning_options.planning_scene_diff.is_diff = true; // what is ths?
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true; // I can guess what is this

  moveit::planning_interface::MoveGroup::Plan plan;
  mac->sendGoal( goal );
  if( !mac->waitForResult() ) { ROS_INFO_STREAM("[DEBUG] MoveGroup action returned early!"); }
  if( mac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
    _traj = ( mac->getResult()->planned_trajectory );
    _planning_time = mac->getResult()->planning_time;
    return true;
  } else {
    return false;
  }
  
  
}


/**
 * @function planJointSpace
 */                        
bool ur5_base_arm_planner::planJointSpace( const moveit::core::RobotState &_start_state,
					   const moveit::core::RobotState &_goal_state,
					   const std::string &_group_name,
					   moveit_msgs::RobotTrajectory &_traj,
					   double &_planning_time,
					   const double &_speed_rate ) {

  boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > mac;
  ros::NodeHandle n;
  mac.reset(new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>( n,
									     move_group::MOVE_ACTION,	
									     false));
  
  ros::Time final_time = ros::Time::now() + ros::Duration(5.0);
  while (mNh.ok() && !mac->isServerConnected() && final_time > ros::Time::now())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  if( !mac->isServerConnected() ) { ROS_ERROR("Planner server NOT connected"); return false; }
  moveit_msgs::MoveGroupGoal goal;
  goal.request.group_name = _group_name;
  goal.request.num_planning_attempts = 5;
  goal.request.max_velocity_scaling_factor = _speed_rate;
  goal.request.allowed_planning_time = 5.0;
  goal.request.planner_id = "";
  double goalJointTolerance = 1e-3; // 0.1deg 
  //goal.request.workspace_parameters = ;
  moveit::core::robotStateToRobotStateMsg( _start_state, goal.request.start_state );
  // Joint
  goal.request.goal_constraints.resize(1);
  goal.request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints( _goal_state, 
										      _start_state.getRobotModel()->getJointModelGroup(_group_name), 
										      goalJointTolerance );

  goal.planning_options.plan_only = true;
  goal.planning_options.look_around = false; // what is this?
  goal.planning_options.replan = false; // what is this?
  goal.planning_options.planning_scene_diff.is_diff = true; // what is ths?
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true; // I can guess what is this

  moveit::planning_interface::MoveGroup::Plan plan;
  mac->sendGoal( goal );
  if( !mac->waitForResult() ) { ROS_INFO_STREAM("[DEBUG] MoveGroup action returned early!"); }
  if( mac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
    _traj = ( mac->getResult()->planned_trajectory );
    _planning_time = mac->getResult()->planning_time;
    return true;
  } else {
    return false;
  }
  
}

/**
 * @function getTrajectoryFromPath
 */
bool ur5_base_arm_planner::getTrajectoryFromPath( const trajectory_msgs::JointTrajectory &_path,
						 trajectory_msgs::JointTrajectory &_traj ) {

  
  moveit_msgs::RobotTrajectory bt_msg;
  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_state::RobotState state_i = this->getCurrentState();
  robot_trajectory::RobotTrajectory rt( state_i.getRobotModel(), "manipulator");
  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg( state_i, _path );
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt);
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(bt_msg);
 
  _traj = bt_msg.joint_trajectory;
   return success;
}
