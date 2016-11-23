

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <sstream>

/**
 * @class ur5_interact
 */
class ur5_interact {
  public:
  ur5_interact( ros::NodeHandle* _nh );  
  void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg);
  void test_move_wrist();


  protected:

  ros::NodeHandle* mNh;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* mTrajClient;
  sensor_msgs::JointState mJointState;
};

/**
 * @function ur5_interact
 * @brief Constructor
 */
ur5_interact::ur5_interact( ros::NodeHandle* _nh ) {
   
  mNh = _nh;
  mTrajClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("follow_joint_trajectory", true);     

  ROS_INFO("Waiting for action server to start.");
  mTrajClient->waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started!");

}

/**
 * @funhction joint_states_cb
 */
void ur5_interact::joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg) {
   /* for( int i = 0; i < msg->position.size(); ++i ) {
      ROS_INFO("Joint [%s]: %f", msg->name[i].c_str(), msg->position[i] );
    }*/
    mJointState = *msg;
}

/**
 * @function test_move_wrist
 */
void ur5_interact::test_move_wrist() {

  control_msgs::FollowJointTrajectoryGoal msg;
  msg.trajectory.joint_names = this->mJointState.name;
  msg.trajectory.points.resize(3);
  trajectory_msgs::JointTrajectoryPoint p0, p1, p2;
  
  std::vector<double> zero_vel( this->mJointState.position.size() );
  for( int i = 0; i < zero_vel.size(); ++i ) { zero_vel[i] = 0; }

  // First position: Let it be the joint state
  p0.positions = this->mJointState.position;
  p0.velocities = zero_vel;
  p0.time_from_start = ros::Duration(0.0);

  p1 = p0;
  p1.positions[5] += 0.15;
  p1.time_from_start = ros::Duration(2.0);

  p2 = p1;
  p2.positions[3] += 0.1;
  p2.positions[2] += 0.1;
  p2.time_from_start = ros::Duration(4.0);

  msg.trajectory.points[0] = p0;
  msg.trajectory.points[1] = p1;
  msg.trajectory.points[2] = p2;

  for( int i = 0; i < msg.trajectory.points.size(); ++i ) {
   for( int j = 0; j < msg.trajectory.points[i].positions.size(); ++j ) {
      printf(" %f ", msg.trajectory.points[i].positions[j] );
    }
    printf("\n");
  }

  printf("Sending goal! \n");  
  mTrajClient->sendGoal(msg);
    bool finished_before_timeout = mTrajClient->waitForResult(ros::Duration(10.0));
    if (finished_before_timeout) {
      actionlib::SimpleClientGoalState state = mTrajClient->getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    } else {
    ROS_INFO("Action did not finish before the time out.");
   }

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur5_test_1");

  ros::NodeHandle nh;
  printf("Subscribe \n");
  ur5_interact mU( &nh );
  ros::Subscriber sb = nh.subscribe("/joint_states", 1, &ur5_interact::joint_states_cb, &mU  );

  ros::spinOnce();
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  mU.test_move_wrist();

  ros::spin();

  return 0;
}
