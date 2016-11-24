#include <ros/ros.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <moveit/move_group/node_name.h> // NODE_NAME
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <achq/HandoverPlanningSrv.h>
#include <achq/ur5_base_arm_planner.h>
#include <achq/GoalSender.h>

#include <ros/ros.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

/*****************************/
/** class PutOnTable         */
/*****************************/
class ExtendObject : public GoalSender {
  
public:
  ExtendObject( ros::NodeHandle* _n, ur5_base_arm_planner* _hbap );
  void resetOctomap();
/*
  bool plan_arm_motion_onTable( achq::HandoverPlanningSrv::Request &_req,
				achq::HandoverPlanningSrv::Response &_response );

  bool plan_arm_motion_inPlace( achq::HandoverPlanningSrv::Request &_req,
				achq::HandoverPlanningSrv::Response &_response );
  
  bool getTrajectoryFromPath( const trajectory_msgs::JointTrajectory &_path,
			      trajectory_msgs::JointTrajectory &_traj );

  void add_points( control_msgs::FollowJointTrajectoryGoal &_goal,
		   double t0, double t1, double t2, double t3, double t4 );
  bool getAngles( double _xg, double _yg, double _zg,
		  double &_t0, double &_t1, double &_t2, double &_t3, double &_t4 );
*/
  void get_arm_state( const sensor_msgs::JointStateConstPtr &_msg );

protected:
  ros::NodeHandle* mN;
  Client* mClient;
  ur5_base_arm_planner* mHbap;
  double mObject_height; 
  double mOffset; 

  std::vector<double> mArm_joints;
};

/**
 * @function resetOctomap
 */
void ExtendObject::resetOctomap() {

  ros::ServiceClient client = mN->serviceClient<std_srvs::Empty>("/clear_octomap");
  std_srvs::Empty srv;
  if (client.call(srv))
    { ROS_INFO("[planning_arm_v4] Reset octomap all right for new query");  }
  else
    { ROS_ERROR("Failed to clear octomap"); }
  
}


/******************************************************/

/**
 * Constructor
 */
ExtendObject::ExtendObject( ros::NodeHandle* _n,
			    ur5_base_arm_planner* _hbap ) :
  mN(0),
  GoalSender(_n),
  mHbap( _hbap ) {
  mN = _n;
  mObject_height = 0.21; 
  mOffset = 0.06; 


  // initialize action client
  printf("--Getting arm client ready \n");
  mClient  = new Client("/follow_joint_trajectory", true);

  // wait for the action server to establish connection
  mClient->waitForServer();
  printf("--Arm client ready! \n");  

  // make sure the controller is running
/*
  ros::ServiceClient client = mN->serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;
  bool running = false;
  while (running == false) {
    ros::Duration(0.1).sleep();
    if (client.call(list_controllers)) {
      for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
        controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
        if (c.name == "arm_controller" && c.state == "running") {
          running = true;
        }
      }
    }
  }     
*/
  printf("Server is on...\n");

}

/**
 * @function get_arm_state
 */
void ExtendObject::get_arm_state( const sensor_msgs::JointStateConstPtr &_msg ) {
  
  const int n = 6;
  std::string names[n] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  mArm_joints.resize(n);
  for( int i = 0; i < n; ++i ) {
    for( int j = 0; j < _msg->name.size(); ++j ) {
      if( names[i].compare(_msg->name[j]) == 0 ) {
	this->mArm_joints[i] = _msg->position[j];
	break;
      } // if
    } // for j
  } // for i
} 




int main( int argc, char* argv[] ) {

  // Start
  ros::init( argc, argv, move_group::NODE_NAME );
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  boost::shared_ptr<tf::TransformListener> tf( new tf::TransformListener( ros::Duration(10.0) ) );
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor( new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf) );
  if( planning_scene_monitor->getPlanningScene() ) {
    ROS_INFO("Good, planning scene is ON");
  } else {
    ROS_ERROR("Planning scene NOT configured!");
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor();
  planning_scene_monitor->startStateMonitor();

  ur5_base_arm_planner ubap( planning_scene_monitor );
  usleep(0.5*1e6); ros::spinOnce();

  ExtendObject eo( &nh, &ubap );
 
  // Subscribe to state 
  printf("Subscribed ! \n");
  ros::Subscriber sub = nh.subscribe("/joint_states", 1,
				     &ExtendObject::get_arm_state,
				     &eo);
  
  ROS_INFO( "[Server Planning Arm v4] Ready to plan an arm motion...waiting for calls \n" );

  // Ask for a joint space trajectory
  ros::spinOnce();

  moveit_msgs::RobotTrajectory traj; 
  moveit::core::RobotState start_state = ubap.getCurrentState();
  double plan_time;
   printf("Get start state...\n");
  moveit::core::RobotState goal_state = start_state;
  printf("Got start state \n");
/*
  printf("Shoulder pan : %f \n",   goal_state.getVariablePosition( "shoulder_pan_joint" ) );
  printf("Shoulder lift: %f \n",   goal_state.getVariablePosition(  "shoulder_lift_joint" ) );
 printf("Elbow: %f \n",   goal_state.getVariablePosition(  "elbow_joint" ) );
 printf("Wrist 1: %f \n",   goal_state.getVariablePosition(  "wrist_1_joint" ) );
 printf("Wrist 2: %f \n",   goal_state.getVariablePosition(  "wrist_2_joint" ) );
 printf(" Wrist 3: %f \n",   goal_state.getVariablePosition(  "wrist_3_joint" ) );

  goal_state.setVariablePosition( "elbow_joint", 0.0 );
  goal_state.setVariablePosition( "wrist_1_joint", -1.5 );
  printf("Plan!\n");  
  if( ubap.planJointSpace( start_state,
		       goal_state,
		       "manipulator",		   
                       traj,
		       plan_time, 0.1 ) ) { printf("Found a plan in joint space! Duration: %f \n", plan_time ); 
   if( ubap.executeTraj( traj ) ) { printf("Executed traj! \n"); }
  }
  else { printf("Did not found a path! \n"); }

  tf::StampedTransform tfx = eo.get_Tf_base_wrist();
  printf("Pose w.r.t. base: %f %f %f - wxyz: %f %f %f %f \n", 
         tfx.getOrigin().getX(), tfx.getOrigin().getY(), tfx.getOrigin().getZ(),
         tfx.getRotation().getW(), tfx.getRotation().getAxis().getX(), tfx.getRotation().getAxis().getY(), tfx.getRotation().getAxis().getZ() );
*/

  // Go pose
  std::vector<geometry_msgs::Pose> target_poses;
  geometry_msgs::Pose pose;
  pose.position.x = 0.173; //0.198; //-0.004;
  pose.position.y = 0.116; //0.108; //0.109;
  pose.position.z = 0.93; //0.89; //1.00;
  double qx, qy, qz, qw;
  qx = -0.093; qy = 0.938; qz = -0.33; qw = 0.63;
  double qa = qx*qx  + qy*qy + qz*qz + qw*qw;
  qa = sqrt(qa); 
  pose.orientation.x = qx/qa; 
  pose.orientation.y =  qy/qa;
  pose.orientation.z = qz/qa;
  pose.orientation.w = qw/qa; 

  //pose.orientation.normalize();

  target_poses.push_back( pose );
  if( ubap.planTargetPose( start_state,
			"manipulator",
                        target_poses,
			"base_link",
			"wrist_3_link",
                        traj,
			plan_time,
			0.1  ) ) { printf("Found a path in cartesian space \n");
     if( ubap.executeTraj( traj ) ) { printf("Executed traj! \n"); }
  } else { printf("Did not find a solution! traj size: %d \n", traj.joint_trajectory.points.size() ); }

  ros::spin();
  
  return 0;

}
