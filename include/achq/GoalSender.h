#include <ros/ros.h>
#include <tf/transform_datatypes.h>

/**
 * @class GoalSender
 */
class GoalSender {

 public:
 GoalSender( ros::NodeHandle *_nh );
 void tfListener();
 tf::StampedTransform get_Tf_base_wrist() { return Tf_base_wrist; }
 
  protected:
 
  ros::NodeHandle* mN;
  tf::StampedTransform Tf_base_wrist;
};
