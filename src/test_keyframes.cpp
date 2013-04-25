/**
 * @function test_keyframes
 */
#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

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

  // Load the trajectories
  trajectory_msgs::JointTrajectory jt;
  jt.header.stamp = ros::Time::now();
//  jt.header.frame_id = "atlas::pelvis";
  jt.header.frame_id = "atlas::l_leg_lax";

  jt.joint_names.push_back("atlas::l_arm_usy");
  jt.joint_names.push_back("atlas::l_arm_shx");
  jt.joint_names.push_back("atlas::l_arm_ely");
  jt.joint_names.push_back("atlas::l_arm_elx");
  jt.joint_names.push_back("atlas::l_arm_uwy");
  jt.joint_names.push_back("atlas::l_arm_mwx");

  jt.joint_names.push_back("atlas::r_arm_usy");
  jt.joint_names.push_back("atlas::r_arm_shx");
  jt.joint_names.push_back("atlas::r_arm_ely");
  jt.joint_names.push_back("atlas::r_arm_elx");
  jt.joint_names.push_back("atlas::r_arm_uwy");
  jt.joint_names.push_back("atlas::r_arm_mwx");

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
  // Load
  FILE* f;
  int counter;
  
  f = fopen("mz_traj.txt", "r");
  if( f!= NULL ) {
    for( int i = 0; i < 1000; ++i ) {
      std::vector<float> val(24);
      fscanf( f, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n", &counter, 
	      &val[0], &val[1], &val[2], &val[3], &val[4], &val[5],
	      &val[6], &val[7], &val[8], &val[9], &val[10], &val[11],
	      &val[12], &val[13], &val[14], &val[15], &val[16], &val[17],
	      &val[18], &val[19], &val[20], &val[21], &val[22], &val[23] );
      
      jointValues.push_back( val );
    }
    fclose(f);
    printf("VALUES READ! \n");
  }
  else {
    printf("FILE NO OPENED!!! \n");
    return 1;
  }

  // Set time 
  double dt = 0.5;

  // Fill the trajectory msg
  for( int repeat  = 0; repeat < 10; ++repeat ) {
    for( int i = 0; i < jointValues.size(); ++i ) {
      trajectory_msgs::JointTrajectoryPoint p;
      for( int j = 0; j < 24; j++ ) {
	p.positions.push_back( jointValues[i][j] );
      }
      
      jt.points.push_back( p );
      jt.points[i].time_from_start = ros::Duration().fromSec(dt);
    }

  }

 
  // Publish the trajectories
  gPub_trajKeyframes = rosNode->advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1, true );

  gPub_trajKeyframes.publish( jt );

  // Spin
  ros::spin();

  return 0;

}
