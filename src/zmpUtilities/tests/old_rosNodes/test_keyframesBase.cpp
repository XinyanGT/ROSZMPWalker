/**
 * @function test_keyframes
 */
#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "zmpUtilities.h"
#include <stdio.h>

// Global Variables
ros::Publisher gPub_trajKeyframes;

/**
 * @function main
 */
int main( int argc, char** argv ) {

  ros::init( argc, argv, "test_keyframes" );
  ros::NodeHandle* rosNode = new ros::NodeHandle();

  // Wait until sim is active (play)
  ros::Time last_ros_time;
  bool wait = true;
  
  while( wait ) {
    last_ros_time = ros::Time::now();
    if( last_ros_time.toSec() > 0 ) {
      wait = false;
    }       
  }



  // 	Variables
  zmpUtilities zp;
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

  zp.setParameters( dt, 9.81 );
  zp.generateZmpPositions( 10, true, 
			   stepLength, footSeparation,
			   stepDuration,
			   slopeTime,
			   levelTime );

  zp.getControllerGains( Qe, R, zg, numPreviewSteps );
  zp.generateCOMPositions();
  zp.getJointTrajectories();

  // Load the trajectories
  trajectory_msgs::JointTrajectory jt;
  jt.header.stamp = ros::Time::now();
  jt.header.frame_id = "atlas::pelvis";

  jt.joint_names.push_back("atlas::l_leg_uhz");
  jt.joint_names.push_back("atlas::l_leg_mhx");
  jt.joint_names.push_back("atlas::l_leg_lhy");
  jt.joint_names.push_back("atlas::l_leg_kny");
  jt.joint_names.push_back("atlas::l_leg_uay");
  jt.joint_names.push_back("atlas::l_leg_lax");

  jt.joint_names.push_back("atlas::r_leg_uhz");
  jt.joint_names.push_back("atlas::r_leg_mhx");
  jt.joint_names.push_back("atlas::r_leg_lhy");
  jt.joint_names.push_back("atlas::r_leg_kny");
  jt.joint_names.push_back("atlas::r_leg_uay");
  jt.joint_names.push_back("atlas::r_leg_lax");


  // Read the file
  std::vector< std::vector <float> > jointValues;
  
    for( int i = 0; i < zp.mLeftLeg.size(); ++i ) {
      std::vector<float> val(12);

	 	for( int j = 0; j < 6; j++ ) {
			val[j] = zp.mLeftLeg[i](j);
        	val[j+6] = zp.mRightLeg[i](j);
		}
      
      jointValues.push_back( val );
    }

  // Set time 
  ros::Time t = jt.header.stamp;
  ros::Time dt2( 0.1 );

  // Fill the trajectory msg
  int counter = 0;
  for( int repeat  = 0; repeat < 10; ++repeat ) {
    for( int i = 0; i < jointValues.size(); ++i ) {
      trajectory_msgs::JointTrajectoryPoint p;
      for( int j = 0; j < 12; j++ ) {
	p.positions.push_back( jointValues[i][j] );
      }
      
      jt.points.push_back( p );
      jt.points[i].time_from_start = ros::Duration().fromSec( t.toSec() + counter*(dt2.toSec()) );
	  counter++;
    }

  }

 
  // Publish the trajectories
  gPub_trajKeyframes = rosNode->advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1, true );

  gPub_trajKeyframes.publish( jt );

  // Spin
  ros::spin();

  return 0;

}
