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


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;



/*****************************/
/** class PutOnTable         */
/*****************************/
class ExtendObject : public GoalSender {
  
public:
  ExtendObject( ros::NodeHandle* _n, ur5_base_arm_planner* _hbap );
  void resetOctomap();
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
 *
 */
int main(int argc, char **argv)
{
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
  ros::Subscriber sub = nh.subscribe("/joint_states", 1,
				     &ExtendObject::get_arm_state,
				     &eo);

  // Offer service
  ros::ServiceServer service = nh.advertiseService( "server_planning_arm_motion_v2", 
						    &ExtendObject::plan_arm_motion_onTable, &eo ); 
  ros::ServiceServer service_inPlace = nh.advertiseService( "server_planning_arm_motion_inPlace", 
							    &ExtendObject::plan_arm_motion_inPlace, &eo ); 
  
  ROS_INFO( "[Server Planning Arm v4] Ready to plan an arm motion...waiting for calls \n" );
  ros::spin();
  
  return 0;
}


/**
 * @function plan_arm_motion_onTable
 */
bool ExtendObject::plan_arm_motion_onTable( achq::HandoverPlanningSrv::Request &_req,
					    achq::HandoverPlanningSrv::Response &_response ) {
  
  printf("On table request!\n");
  // Standard response
  _response.success = false;
  // 1. Find the point where the robot's hand must point at (depending on the bounding box)
  geometry_msgs::Point msg_map_centroid = _req.user_centroid_3D.point;
  geometry_msgs::PoseStamped align_goal;
  /*
  tf::Point P_base_centroid, P_map_centroid;
  tf::pointMsgToTF( msg_map_centroid, P_map_centroid );
  tf::Quaternion qm = (this->Tf_base_map.getRotation()); 
  tf::Matrix3x3 rm(qm);
  P_base_centroid = rm * (P_map_centroid - this->Tf_map_base.getOrigin() );
*/
  // 2. Move arm to reach the bounding box centroid
  // or if that is out of range, keep it clamped
  double xg = 0; // P_base_centroid.getX(); // > 0.28
  double yg = 0; //0.078;
  double zg = 0; //P_base_centroid.getZ() + 0.5*mObject_height;
  double t0, t1, t2, t3, t4;
  if( !getAngles( xg, yg, zg,
		  t0, t1, t2, t3, t4 ) ) { printf("Get angles error \n"); return false; }

  double t0_plus = t0 + mOffset; 

  // fill ROS message
  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("elbow_joint");
  goal.trajectory.joint_names.push_back("wrist_1_joint");
  goal.trajectory.joint_names.push_back("wrist_2_joint");
  goal.trajectory.joint_names.push_back("wrist_3_joint");

  double dt1 = -5*M_PI/180.0;

  // Use the starting height of 
  
  add_points( goal, mArm_joints[0],            dt1,  -M_PI/2.0, -M_PI/2.0, 0 ); // Down
  add_points( goal, t0,           dt1,  -M_PI/2.0, -M_PI/2.0, 0 ); // Go up
  add_points( goal, t0_plus,      dt1,          0, -M_PI/2.0, 0 ); // Rotate front
  add_points( goal, t0_plus,       t1,          0,        t3, 0 ); // Incline forwards
  add_points( goal, t0,            t1,          0,        t3, 0 ); // Place (go down)

  // Get a trajectory
  trajectory_msgs::JointTrajectory traj;
  bool success = mHbap->getTrajectoryFromPath( goal.trajectory,
					       traj );
  // Check that this trajectory does not have a collision on any of its points
  if( !mHbap->checkPathNonCollision( mHbap->getCurrentState(),
				     traj, "arm" ) ) { printf("Error in check coll\n"); return false; }
  if( success ) { 

    // Move the arm & wait for the action server to complete the order
    goal.trajectory = traj;
    mClient->sendGoal(goal);        
    mClient->waitForResult( goal.trajectory.points.back().time_from_start );
    // Store result in response
    _response.trajectory = goal.trajectory;
    //Get backwards trajectory
    trajectory_msgs::JointTrajectory bt;
    bt.joint_names = _response.trajectory.joint_names;
    for( int i = _response.trajectory.points.size() - 1; i >= 0; i-- ) {
      bt.points.push_back( _response.trajectory.points[i] );
    }
    mHbap->getTrajectoryFromPath( bt, traj );
    _response.backwards_trajectory = traj; 
    _response.success = true;

    return true;

  } else {
    printf("Didn't get traj from path \n");
    return false;
  }

}


/**
 * @function move_arm
 */
bool ExtendObject::plan_arm_motion_inPlace( achq::HandoverPlanningSrv::Request &_req,
					    achq::HandoverPlanningSrv::Response &_response ) {
  // Standard response
  _response.success = false;
     
  // 1. Find the point where the robot's hand must point at (depending on the bounding box)
  geometry_msgs::Point msg_map_centroid = _req.user_centroid_3D.point;
  geometry_msgs::PoseStamped align_goal;
  
  tf::Point P_base_centroid, P_map_centroid;
  tf::pointMsgToTF( msg_map_centroid, P_map_centroid );
  tf::Quaternion qm;// = (this->Tf_base_map.getRotation()); 
  tf::Matrix3x3 rm(qm);
  //P_base_centroid = rm * (P_map_centroid - this->Tf_map_base.getOrigin() );

  // 2. Move arm to reach the bounding box centroid
  // or if that is out of range, keep it clamped
  double xg = 0.40;
  double yg = 0.078;
  // Seemingly limits for safe George's movement
  double height = P_base_centroid.getZ();
  if( height < 0.68 ) { height = 0.68; }
  if( height > 0.92 ) { height = 0.92; }
  double zg = height;

  double t0, t1, t2, t3, t4;
  if( !getAngles( xg, yg, zg,
		  t0, t1, t2, t3, t4 ) ) { printf("Got angles error \n"); return false; }

  // fill ROS message
  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("elbow_joint");
  goal.trajectory.joint_names.push_back("wrist_1_joint");
  goal.trajectory.joint_names.push_back("wrist_2_joint");
  goal.trajectory.joint_names.push_back("wrist_3_joint");

  double dt1 = -5*M_PI/180.0;
  
  add_points( goal, mArm_joints[0],       dt1,  -M_PI/2.0, -M_PI/2.0, 0 ); // Down
  add_points( goal, t0,      dt1,  -M_PI/2.0, -M_PI/2.0, 0 ); // Go up
  add_points( goal, t0,      dt1,          0, -M_PI/2.0, 0 ); // Rotate front
  add_points( goal, t0,       t1,          0,        t3, 0 ); // Incline forwards

  // Get a trajectory
  trajectory_msgs::JointTrajectory traj;
  bool success = mHbap->getTrajectoryFromPath( goal.trajectory,
					       traj );
  // Check that this trajectory does not have a collision on any of its points
  if( !mHbap->checkPathNonCollision( mHbap->getCurrentState(),
				     traj, "arm" ) ) { printf("Error in check coll\n"); return false; }
  if( success ) { 


    // Now do it
    goal.trajectory = traj;
    mClient->sendGoal(goal);    
    // wait for the action server to complete the order
    mClient->waitForResult( goal.trajectory.points.back().time_from_start );
    // Copy in response
    _response.trajectory = goal.trajectory;

    //Get backwards trajectory
    trajectory_msgs::JointTrajectory bt;
    bt.joint_names = _response.trajectory.joint_names;
    for( int i = _response.trajectory.points.size() - 1; i >= 0; i-- ) {
      bt.points.push_back( _response.trajectory.points[i] );
    }
    mHbap->getTrajectoryFromPath( bt, traj );
    _response.backwards_trajectory = traj; 
    _response.success = true;

    return true;

  } else {
    printf("Didn't get traj from path \n");
    return false;
  }

}


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


bool ExtendObject::getAngles( double _xg, double _yg, double _zg,
			      double &_t0, double &_t1, double &_t2, double &_t3, double &_t4 ) {

  double alpha, d; double beta, k, ko;
  beta = atan2(0.005, 0.345);
  k = sqrt(0.345*0.345 + 0.005*0.005 );
  ko = (_xg - 0.141 - 0.141);
  if( fabs(ko/k) > 1 ) { printf("Robot was too far! - dist > 1"); return false; } // Alpha is nan

  alpha = asin( ko/k ) - beta;
  d = _zg  + 0.012 + 0.005*sin(alpha) - 0.345*cos(alpha) - 0.340;

  printf("X: %f y: %f z: %f \n", _xg, _yg, _zg );
  printf("Alpha: %f d: %f beta: %f \n", alpha*180.0/3.1416, d, beta*180.0/3.1416 );

  _t0 = d;
  _t1 = -alpha;
  _t2 = 0;
  _t3 = -(M_PI/2.0-alpha);
  _t4 = 0;

  printf("D: %f Angles: %f %f %f %f \n",  _t0, _t1*180.0/M_PI, _t2, _t3*180.0/M_PI, _t4 );
  
  if( _t0 < 0.01 || _t0 > 0.68 ) { printf("Exceed limit on lift joint \n"); return false; }
  if( _t1 > 0 ||  _t1 < (-150.0 + 5.0)*M_PI/180.0 ) { printf("Exceed limit on flex link \n"); return false; }
  if( _t3 < (-110 + 5)*M_PI/180.0 || _t3 > (70-5)*M_PI/180.0 ) { printf("Exceed limit on flex joint \n"); return false; }

  double t0_plus = _t0 + mOffset; 
  if( t0_plus > 0.68 ) { printf("Placing above is too high \n"); return false; }

  // Double checking
  double xc, zc;
  xc = 0.141 + 0.345*sin(alpha) - 0.005*cos(alpha) + 0.141;
  zc = d + 0.340 +0.345*cos(alpha) - 0.005*sin(alpha) - 0.012 - mObject_height*0.5;
  printf("Xc: %f zc: %f \n", xc, zc);

  return true;
}


/**
 * @function add_points
 * @brief Simple utilitarian function
 */
void ExtendObject::add_points( control_msgs::FollowJointTrajectoryGoal &_goal,
			       double t0, double t1, double t2, double t3, double t4 ) {
  
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.resize(5);
  point.positions[0] = t0;
  point.positions[1] = t1;
  point.positions[2] = t2;
  point.positions[3] = t3;
  point.positions[4] = t4;
  point.velocities.resize(5);
  for (size_t i = 0; i < 5; ++i) {
    point.velocities[i] = 0.0;
  }
  // This does not matter as we will pass this path through a traj generator
  //point.time_from_start = ros::Duration(_dt);

  _goal.trajectory.points.push_back( point );

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
  mClient  = new Client("/arm_controller/follow_joint_trajectory", true);

  // wait for the action server to establish connection
  mClient->waitForServer();
  printf("--Arm client ready! \n");  

  // make sure the controller is running
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
