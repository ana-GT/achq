/**
 * @brief Implementation of simple class that sends goals to map 
 * @brief and tells you if robot gets there or not
 */
#include <achq/GoalSender.h>
#include <boost/thread.hpp>
#include <tf/transform_listener.h>



/**
 * @function GoalSender
 * @brief Constructor
 */
GoalSender::GoalSender( ros::NodeHandle *_nh ) :
  mN(0) {

  this->mN = _nh;
  // Start thread to listen to transforms constantly
  boost::thread tf_listener_t( &GoalSender::tfListener,  this );
}


/**
 * @function tfListener
 */
void GoalSender::tfListener() {

  // Listen for transform once (assume head does not move)
  tf::TransformListener listener;
  double frequency = 20.0; // 20 Hz
  ros::Rate tf_rate( frequency );

  listener.waitForTransform("base_link", 
			    "wrist_3_link",
			    ros::Time(), ros::Duration(1.0) );
  
  while ( ros::ok() ) {
    
    try {

      listener.lookupTransform( "base_link",
				"wrist_3_link",
				ros::Time(),
				this->Tf_base_wrist );
     
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }
    
    tf_rate.sleep();

  } // end while
  
}
