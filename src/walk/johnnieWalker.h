/**
 * @file johnnieWalker.h
 * @author Everybody
 * @date Mon, April 29th, 2013 
 */

#ifndef _JOHNNIE_WALKER_H_
#define _JOHNNIE_WALKER_H_

// ROS stuff
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>

// Messages
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/AtlasCommand.h>

// Dart
#include <robotics/parser/dart_parser/DartLoader.h>
#include <robotics/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/FileInfoSkel.hpp>

// vrc-golem/dart-atlas
#include <atlas/AtlasKinematics.h>
#include <utils/AtlasPaths.h>


// Additional
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>
#include <vector>

// Our ZMP library
#include "zmp/zmpUtilities.h"


/** FUNCTIONS */
void initVariables();
void RosJointInit(); 
void setSubscriptionPublishing();

/** Callback to read joint states as published */
void GetJointStates( const sensor_msgs::JointState::ConstPtr &_js );
/** Load Atlas urdf */
bool setKinematics();
/** Put Atlas in a better walking position (WTH?) */
void RelaxAtlas( atlas::AtlasKinematics *_AK, 
		 kinematics::Skeleton *_atlas, 
		 Eigen::VectorXd &_dofs, 
		 double _kp, 
		 double _kd, 
		 int _N );


void MoveDesireDofs( atlas::AtlasKinematics* _AK, kinematics::Skeleton *_atlas, 
		     const Eigen::VectorXd &_dofs, 
		     const double &_leftKp, const double &_rightKp, 
		     const double &_otherKp, const double &_leftKd, 
		     const double &_rightKd, const double &_otherKd, 
		     const int &_N );
void PublishCommand( atlas::AtlasKinematics *_AK, const Eigen::VectorXd &_dofs, 
		     const double &_leftKp, const double &_rightKp, const double &_otherKp, 
		     const double &_leftKd, const double &_rightKd, const double &_otherKd );

void mapJointState2DartPose( Eigen::VectorXd &_dartPose,
			     const Eigen::VectorXd  &_jointAngles );

void  MoveCOMIK( atlas::AtlasKinematics *_AK, 
		 kinematics::Skeleton *_atlasSkel, 
		 Eigen::Matrix4d &_Twb,
		 Eigen::Matrix4d _Twm[atlas::NUM_MANIPULATORS],
		 Eigen::VectorXd &_dofs, 
		 Eigen::Vector3d _comDelta, 
		 Eigen::Vector3d _leftDelta, 
		 Eigen::Vector3d _rightDelta, 
		 const double &_leftKp, 
		 const double &_rightKp, 
		 const double &_otherKp,
		 const double &_leftKd, 
		 const double &_rightKd, 
		 const double &_otherKd,
		 const int _N );

void UpdateDofs( Eigen::VectorXd &_dofs );

void MoveJointTractoryAdv( atlas::AtlasKinematics *AK, 
			   kinematics::Skeleton *_atlasSkel, 
			   Eigen::VectorXd &dofs,
			   const std::vector<Eigen::VectorXd> &_zmp,
			   const std::vector<int> supportInfo,
			   const Eigen::VectorXd &double_support_kp, const Eigen::VectorXd &double_support_ki,
			   const Eigen::VectorXd &left_support_kp, const Eigen::VectorXd &left_support_ki,
			   const Eigen::VectorXd &right_support_kp, const Eigen::VectorXd &right_support_ki);

#endif /** _JOHNNIE_WALKER_H_ */
