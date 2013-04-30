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

#include "simpleWalk.h"

// Global Variables
ros::Publisher gPubJointCommands;
osrf_msgs::JointCommands gJointCommands;
zmpUtilities gZU;


/**
 * @function main
 */
int main( int argc, char** argv ) {

  ros::init( argc, argv, "walk" );
  ros::NodeHandle* rosNode = new ros::NodeHandle();

  // Set ZMP configuration values
  setZMPConfig( gZU );

  // Wait until sim is active (play)
  ros::Time last_ros_time;
  bool wait = true;
  
  while( wait ) {
    last_ros_time = ros::Time::now();
    if( last_ros_time.toSec() > 0 ) {
      wait = false;
    }       
  }

  // Set joint command configuration
  setJointConfig( gJointCommands, rosNode );

  // Set into initial position
  setInitPosition( gZU );
  
  // ros topic subscribtions
  ros::SubscribeOptions jointStatesSo = ros::SubscribeOptions::create<sensor_msgs::JointState>( "/atlas/joint_states", 1, setJointStates, ros::VoidPtr(), rosNode->getCallbackQueue() );
  
  // Declare preference of UDP over TCP
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();
  ros::Subscriber subJointStates = rosNode->subscribe(jointStatesSo);

  // Advertise joint commands
  gPubJointCommands = rosNode->advertise<osrf_msgs::JointCommands>( "/atlas/joint_commands", 1, true);

  ros::spin();
  
  return 0;
}

/**
 * @function setInitPosition
 */
void setInitPosition( zmpUtilities &_zu ) {

  // Get init position

}

/**
 * @function setJointStates
 */
void setJointStates( const sensor_msgs::JointState::ConstPtr &_js ) {
  
  static ros::Time startTime = ros::Time::now();
  static int count = 0;
  
  for( unsigned int i = 0; i < _js->name.size(); ++i ) {
    gJointCommands.header.stamp = _js->header.stamp;
    gJointCommands.position[i] = gZU.mWholeBody[count](i);
  }
  
  if( ros::Time::now().toSec() - startTime.toSec() > 0.01 ) {
    count++;
    startTime = ros::Time::now();
    //printf("Count: %d \n", count);
  }
  
  gPubJointCommands.publish( gJointCommands );
}

/**
 * @function setZMPConfig
 */
void setZMPConfig( zmpUtilities &_zu ) {
  
  // Variables
  int numSteps = 12;
  double stepLength = 0.15;
  double footSeparation = 0.282;
  double stepDuration = 1.0;
  double slopeTime = 0.15;
  double levelTime = 0.85;
  double dt = 0.01;
  double zg = 0.8438;
  int numPreviewSteps = 2;
  double Qe = 1;
  double R = 0.000001;

  _zu.setParameters( dt, 9.81 );
  _zu.generateZmpPositions( numSteps, true, 
			    stepLength, footSeparation,
			    stepDuration,
			    slopeTime,
			    levelTime );

  _zu.getControllerGains( Qe, R, zg, numPreviewSteps );
  _zu.generateCOMPositions();
  _zu.getJointTrajectories();

}

/**
 * @function setJointConfig
 */
void setJointConfig( osrf_msgs::JointCommands &_jointCommands,
		     ros::NodeHandle* _rosNode ) {

  // Set names (follow this order and use ALL these joints, even if you don't set them)
  _jointCommands.name.push_back("atlas::back_lbz");
  _jointCommands.name.push_back("atlas::back_mby");
  _jointCommands.name.push_back("atlas::back_ubx");

  _jointCommands.name.push_back("atlas::neck_ay");

  _jointCommands.name.push_back("atlas::l_leg_uhz");
  _jointCommands.name.push_back("atlas::l_leg_mhx");
  _jointCommands.name.push_back("atlas::l_leg_lhy");
  _jointCommands.name.push_back("atlas::l_leg_kny");
  _jointCommands.name.push_back("atlas::l_leg_uay");
  _jointCommands.name.push_back("atlas::l_leg_lax");

  _jointCommands.name.push_back("atlas::r_leg_uhz");
  _jointCommands.name.push_back("atlas::r_leg_mhx");
  _jointCommands.name.push_back("atlas::r_leg_lhy");
  _jointCommands.name.push_back("atlas::r_leg_kny");
  _jointCommands.name.push_back("atlas::r_leg_uay");
  _jointCommands.name.push_back("atlas::r_leg_lax");

  _jointCommands.name.push_back("atlas::l_arm_usy"); 
  _jointCommands.name.push_back("atlas::l_arm_shx");
  _jointCommands.name.push_back("atlas::l_arm_ely");
  _jointCommands.name.push_back("atlas::l_arm_elx");
  _jointCommands.name.push_back("atlas::l_arm_uwy");
  _jointCommands.name.push_back("atlas::l_arm_mwx");

  _jointCommands.name.push_back("atlas::r_arm_usy");
  _jointCommands.name.push_back("atlas::r_arm_shx");
  _jointCommands.name.push_back("atlas::r_arm_ely");
  _jointCommands.name.push_back("atlas::r_arm_elx");
  _jointCommands.name.push_back("atlas::r_arm_uwy");
  _jointCommands.name.push_back("atlas::r_arm_mwx"); 


  unsigned int n = _jointCommands.name.size();

  // Set other parameters
  _jointCommands.position.resize(n);
  _jointCommands.velocity.resize(n);
  _jointCommands.effort.resize(n);
  _jointCommands.kp_position.resize(n);
  _jointCommands.ki_position.resize(n);
  _jointCommands.kd_position.resize(n);
  _jointCommands.kp_velocity.resize(n);
  _jointCommands.i_effort_min.resize(n);
  _jointCommands.i_effort_max.resize(n);

  // Set PID values from node parameters
  for( unsigned int i = 0; i < n; ++i ) {
    std::vector<std::string> pieces;
    boost::split( pieces, _jointCommands.name[i], boost::is_any_of(":") );
    
    _rosNode->getParam("atlas_controller/gains/" + pieces[2] + "/p", _jointCommands.kp_position[i] );
    _rosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i", _jointCommands.ki_position[i] );
    _rosNode->getParam("atlas_controller/gains/" + pieces[2] + "/d", _jointCommands.kd_position[i] );

    _rosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", _jointCommands.i_effort_min[i] );
    _jointCommands.i_effort_min[i] = -_jointCommands.i_effort_min[i];

    _rosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", _jointCommands.i_effort_max[i] );
    
    
    _jointCommands.velocity[i] = 0;
    _jointCommands.effort[i] = 0;
    _jointCommands.kp_velocity[i] = 0;
  }

}
