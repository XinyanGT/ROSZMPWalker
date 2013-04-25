/**
 * @function simpleWalk
 */
#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>


#include "zmpUtilities.h"
#include <stdio.h>

// Global Variables
ros::Publisher gPubJointCommands;
osrf_msgs::JointCommands gJointCommands;
zmpUtilities gZU;

/**
 * @function setJointStates
 */
void setJointStates( const sensor_msgs::JointState::ConstPtr &_js ) {
  
  static ros::Time startTime = ros::Time::now();
  static int count = 0;

   gJointCommands.header.stamp = _js->header.stamp;

    // assigned sinusoidal joint angle targets
    for( unsigned int i = 0; i < gJointCommands.name.size(); ++i ) {
      gJointCommands.position[i] = 1.5*sin( (ros::Time::now() - startTime).toSec() );
   
    }

  gPubJointCommands.publish( gJointCommands );
}

/**
 * @function main
 */
int main( int argc, char** argv ) {
  printf("Start main of simpleWalk Test \n");
  ros::init( argc, argv, "walk" );
  ros::NodeHandle* rosNode = new ros::NodeHandle();

  // Variables
  double stepLength = 0.15;
  double footSeparation = 0.282;
  double stepDuration = 1.0;
  double slopeTime = 0.15;
  double levelTime = 0.85;
  double dt = 0.01;
  double zg = 0.84;
  int numPreviewSteps = 2;
  double Qe = 1;
  double R = 0.000001;

  gZU.setParameters( dt, 9.81 );
  gZU.generateZmpPositions( 10, true, 
			    stepLength, footSeparation,
			    stepDuration,
			    slopeTime,
			    levelTime );

  gZU.getControllerGains( Qe, R, zg, numPreviewSteps );
  gZU.generateCOMPositions();
  gZU.getJointTrajectories();
  gZU.print( std::string("LeftFoot.txt"), gZU.mLeftFoot );
  gZU.print( std::string("RightFoot.txt"), gZU.mRightFoot );

  printf("Gotten all joint trajectories \n");

  // Wait until sim is active (play)
  ros::Time last_ros_time;
  bool wait = true;
  
  while( wait ) {
    last_ros_time = ros::Time::now();
    if( last_ros_time.toSec() > 0 ) {
      wait = false;
    }       
  }

  // Set names
  gJointCommands.name.push_back("atlas::l_arm_usy");

  unsigned int n = gJointCommands.name.size();
  gJointCommands.position.resize(n);
  gJointCommands.velocity.resize(n);
  gJointCommands.effort.resize(n);
  gJointCommands.kp_position.resize(n);
  gJointCommands.ki_position.resize(n);
  gJointCommands.kd_position.resize(n);
  gJointCommands.kp_velocity.resize(n);
  gJointCommands.i_effort_min.resize(n);
  gJointCommands.i_effort_max.resize(n);

  for( unsigned int i = 0; i < n; ++i ) {
    std::vector<std::string> pieces;
    boost::split( pieces, gJointCommands.name[i], boost::is_any_of(":") );
    
    rosNode->getParam("atlas_controller/gains/" + pieces[2] + "/p", gJointCommands.kp_position[i] );
    rosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i", gJointCommands.ki_position[i] );
    rosNode->getParam("atlas_controller/gains/" + pieces[2] + "/d", gJointCommands.kd_position[i] );

    rosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", gJointCommands.i_effort_min[i] );
    gJointCommands.i_effort_min[i] = -gJointCommands.i_effort_min[i];

    rosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", gJointCommands.i_effort_max[i] );


    gJointCommands.velocity[i] = 0;
    gJointCommands.effort[i] = 0;
    gJointCommands.kp_velocity[i] = 0;
  }

  // ros topic subscribtions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, setJointStates,
    ros::VoidPtr(), rosNode->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();

  ros::Subscriber subJointStates = rosNode->subscribe(jointStatesSo);
  // ros::Subscriber subJointStates =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetJointStates);

  gPubJointCommands =
    rosNode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);

  ros::spin();

  return 0;

}
