
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group/node_name.h> // NODE_NAME
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/move_group/capability_names.h>
#include <moveit/move_group/move_group_capability.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/tokenizer.hpp>
#include <moveit/macros/console_colors.h>
#include <moveit/move_group/node_name.h>
#include <memory>
#include <set>

/**
 * @class ur5_base_arm_planner
 */
class ur5_base_arm_planner {

  public:
    ur5_base_arm_planner( const planning_scene_monitor::PlanningSceneMonitorPtr &_psm );
    void configureCapabilities();
    moveit::core::RobotState getCurrentState( );
    bool isStateColliding( const moveit::core::RobotState& _state, 
	                    const std::string& _group );
    bool checkPathNonCollision( moveit::core::RobotState _start_state, 
	             	        const trajectory_msgs::JointTrajectory &_traj,
			        std::string _group );
    bool executeTraj( const moveit_msgs::RobotTrajectory &_traj );
    bool planTargetPose( const moveit::core::RobotState &_start_state,
					   const std::string &_group_name,
					   const std::vector<geometry_msgs::Pose> &_target_poses,
					   const std::string &_frame_id,
					   const std::string &_end_effector,
					   moveit_msgs::RobotTrajectory &_traj,
					   double &_planning_time,
					   const double &_speed_rate );

     bool planJointSpace( const moveit::core::RobotState &_start_state,
	   	   const moveit::core::RobotState &_goal_state,
		   const std::string &_group_name,
		   moveit_msgs::RobotTrajectory &_traj,
		   double &_planning_time,
		   const double &_speed_rate );
     bool getTrajectoryFromPath( const trajectory_msgs::JointTrajectory &_path,
     			        trajectory_msgs::JointTrajectory &_traj );

  protected:
    planning_scene_monitor::PlanningSceneMonitorPtr mPlanning_scene_monitor;
    ros::NodeHandle mNh;
    move_group::MoveGroupContextPtr mContext;
    std::shared_ptr<pluginlib::ClassLoader<move_group::MoveGroupCapability> > mCapability_plugin_loader;
    //std::vector<move_group::MoveGroupCapabilityPtr> mCapabilities;
    std::vector< boost::shared_ptr<move_group::MoveGroupCapability> > mCapabilities;
};


