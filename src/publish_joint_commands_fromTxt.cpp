/**
 * @function publish_joint_commands
 */

#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>


// To read txt file
#include <Eigen/Core>
#include <stdio.h>
#include <stdlib.h>

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;
std::vector< std::vector<float> > jointValues;

double dt = 0.05;

/**
 * @function SetJointStates
 */
void SetJointStates( const sensor_msgs::JointState::ConstPtr &_js ) {
  
  static ros::Time startTime = ros::Time::now();
  static int counter = 0;

  {
    // For testing round trip time
    jointcommands.header.stamp = _js->header.stamp;

    if( (ros::Time::now() - startTime ).toSec() > dt ) {
      counter++;
      startTime = ros::Time::now();
      printf("Counter: %d \n", counter);

      for( int i = 0; i < 24; ++i ) {
	printf(" %f ", jointcommands.position[i] );
      }
      printf("\n");

    }

    // assigned joint values
    if( counter < 1000 ) {
      for( unsigned int i = 0; i < jointcommands.name.size(); ++i ) {
	jointcommands.position[i] = jointValues[counter][i];
      }
    }

    else {
      printf("SEND 1000 ALREADY! \n");
    }
    pub_joint_commands_.publish( jointcommands );
  }

}

/**
 * @function main
 */
int main( int argc, char** argv ) {
 
  ros::init( argc, argv, "pub_joint_command_test" );
  ros::NodeHandle* rosnode = new ros::NodeHandle();

  ros::Time last_ros_time_;
  printf("Wait until it is active \n");
  // Wait until sim is active (play)
  bool wait = true;

  while( wait ) {
    last_ros_time_ = ros::Time::now();
    if( last_ros_time_.toSec() > 0 ) {
      wait = false;
    }
  }
  printf("It is active NOW!!! \n");
  // Read txt values
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

  // Set name of joints to be commanded 
  // (must match these inside AtlasPlugin)
  /*
  jointcommands.name.push_back("atlas::back_lbz");
  jointcommands.name.push_back("atlas::back_mby");
  jointcommands.name.push_back("atlas::back_ubx");

  jointcommands.name.push_back("atlas::neck_ay");
  */

  jointcommands.name.push_back("atlas::l_arm_usy");
  jointcommands.name.push_back("atlas::l_arm_shx");
  jointcommands.name.push_back("atlas::l_arm_ely");
  jointcommands.name.push_back("atlas::l_arm_elx");
  jointcommands.name.push_back("atlas::l_arm_uwy");
  jointcommands.name.push_back("atlas::l_arm_mwx");

  jointcommands.name.push_back("atlas::r_arm_usy");
  jointcommands.name.push_back("atlas::r_arm_shx");
  jointcommands.name.push_back("atlas::r_arm_ely");
  jointcommands.name.push_back("atlas::r_arm_elx");
  jointcommands.name.push_back("atlas::r_arm_uwy");
  jointcommands.name.push_back("atlas::r_arm_mwx");

  jointcommands.name.push_back("atlas::l_leg_uhz");
  jointcommands.name.push_back("atlas::l_leg_mhx");
  jointcommands.name.push_back("atlas::l_leg_lhy");
  jointcommands.name.push_back("atlas::l_leg_kny");
  jointcommands.name.push_back("atlas::l_leg_uay");
  jointcommands.name.push_back("atlas::l_leg_lax");

  jointcommands.name.push_back("atlas::r_leg_uhz");
  jointcommands.name.push_back("atlas::r_leg_mhx");
  jointcommands.name.push_back("atlas::r_leg_lhy");
  jointcommands.name.push_back("atlas::r_leg_kny");
  jointcommands.name.push_back("atlas::r_leg_uay");
  jointcommands.name.push_back("atlas::r_leg_lax");



  unsigned int n = jointcommands.name.size();
  jointcommands.position.resize(n);
  jointcommands.velocity.resize(n);
  jointcommands.effort.resize(n);
  jointcommands.kp_position.resize(n);
  jointcommands.ki_position.resize(n);
  jointcommands.kd_position.resize(n);
  jointcommands.kp_velocity.resize(n);
  jointcommands.i_effort_min.resize(n);
  jointcommands.i_effort_max.resize(n);

  for( unsigned int i = 0; i < n; ++i ) {
    std::vector<std::string> pieces;
    boost::split( pieces, jointcommands.name[i], boost::is_any_of(":") );

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p", jointcommands.kp_position[i] );
    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i", jointcommands.ki_position[i] );
    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d", jointcommands.kd_position[i] );

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", jointcommands.i_effort_min[i] );
    jointcommands.i_effort_min[i] = -jointcommands.i_effort_min[i];

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", jointcommands.i_effort_max[i] );


    jointcommands.velocity[i] = 0;
    jointcommands.effort[i] = 0;
    jointcommands.kp_velocity[i] = 0;
  }

  // ros topic subscribtions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, SetJointStates,
    ros::VoidPtr(), rosnode->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();

  ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);
  // ros::Subscriber subJointStates =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetJointStates);

  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);

  ros::spin();

  return 0;
}
