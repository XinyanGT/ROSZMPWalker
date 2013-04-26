#pragma once

#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>

#include <dynamics/SkeletonDynamics.h>
#include <kinematics/FileInfoSkel.hpp>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <robotics/World.h>

#include <utils/AtlasPaths.h>


#include <dynamics/SkeletonDynamics.h>
#include <kinematics/FileInfoSkel.hpp>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <robotics/World.h>

#include <utils/AtlasPaths.h>
#include <atlas/AtlasKinematics.h>

#include <iostream>

#include "walk/zmp/zmpUtilities.h"

using namespace std;
using namespace kinematics;
using namespace dynamics;
using namespace robotics;
using namespace Eigen;
using namespace atlas;

/*
 * Global variables
 */

extern atlas::AtlasKinematics *_ak;
extern dynamics::SkeletonDynamics *_atlas;
extern zmpUtilities gZU;

extern ros::Publisher pub_joint_commands_;
extern osrf_msgs::JointCommands jointcommands;
extern osrf_msgs::JointCommands jointcommands_saved;
extern VectorXd jointangles;
extern ros::Rate *loop_rate;
extern ros::NodeHandle* rosnode;





void GetJointStates( const sensor_msgs::JointState::ConstPtr &_js );


/* @function: runTrajectory()
 * @brief: Publish joint angle trajectory to ROS
 */
void runTrajectory(AtlasKinematics *AK,
				   const std::vector<Eigen::VectorXd> &_traj,
				   const double leftKp,
				   const double rightKp,
				   const double otherKp,
				   const double leftKd,
				   const double rightKd,
				   const double otherKd);

/**********************************************
 * Init joint related ros stuff
 *********************************************/
void RosJointInit();

/**********************************************************
 * Read current actual joint angles,
 * update a global vector
 *********************************************************/
void UpdateDofs(const AtlasKinematics *AK, VectorXd &dofs);

/************************************************
 * Publish a joint commands based on a dart pos
 ***********************************************/
void PublishCommand(AtlasKinematics *AK,
					const VectorXd &dofs,
					const double leftKp,
					const double rightKp,
					const double otherKp,
					const double leftKd,
					const double rightKd,
					const double otherKd);

 /*************************************************
 * Move Atlas from start pose to end pose
 *************************************************/
// was MoveDesiredDofs()
void genDofTraj(AtlasKinematics *AK,
				const VectorXd &start_dofs,
				const VectorXd &end_dofs,
				std::vector<Eigen::VectorXd> &_traj,
				int N);

/*****************************************************
 * Move Atlas based on joint trajectory
 ********************************************************/
void MoveJointTractory(AtlasKinematics *AK,
					   Skeleton *_atlas,
					   const std::vector<Eigen::VectorXd> &_zmp,
					   VectorXd &dofs,
					   const double leftKp,
					   const double rightKp,
					   const double otherKp,
					   const double leftKd,
					   const double rightKd,
					   const double otherKd);

/*************************************************
 * Move Atlas based on comIK
 *************************************************/
// was  MoveCOMIK
/* @funtion: genCOMIKTraj
 * @brief: move atlas around by com IK
 * @parameters:
 * 		AK: for computation
 *    	_atlas: for computation
 *    	Twb: world to body
 *    	Twm: world to manip
 *    	dofs: start dofs, filled with end dofs
 *    	comDelta: move com
 *    	leftDelta: move left leg
 *    	rightDelta: move right leg
 *    	_traj: joint traj to fill
 *    	N: num steps
 */
void genCOMIKTraj(AtlasKinematics *AK,
				  Skeleton *_atlas,
				  Matrix4d &Twb,
				  Matrix4d Twm[NUM_MANIPULATORS],
				  VectorXd &dofs,
				  Vector3d comDelta,
				  Vector3d leftDelta,
				  Vector3d rightDelta,
				  std::vector<Eigen::VectorXd> &_traj,
				  const int N);

/*******************************************
 * Relax Atlas to a better pos for walking
 ********************************************/
void Relax(AtlasKinematics *AK,
		   Skeleton *_atlas,
		   VectorXd &dofs,
		   vector<VectorXd> &_traj,
		   int N);


