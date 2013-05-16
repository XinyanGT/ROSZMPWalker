/**
 * @function Set position and read final angles
 */

#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <std_msgs/String.h>

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
#include <getopt.h>

#include <zmpUtilities/zmpUtilities.h>
#include "zmp_walk.h"

using namespace std;
using namespace kinematics;
using namespace dynamics;
using namespace robotics;
using namespace Eigen;
using namespace atlas;

atlas::AtlasKinematics *_ak;
dynamics::SkeletonDynamics *_atlas;
zmpUtilities gZU;

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;
osrf_msgs::JointCommands jointcommands_saved;
VectorXd jointangles;
ros::Rate *loop_rate;
ros::NodeHandle* rosnode;


void runTrajectory(AtlasKinematics *AK,
		   	   	   const std::vector<Eigen::VectorXd> &_traj,
		   	   	   const double leftKp,
		   	   	   const double rightKp,
		   	   	   const double otherKp,
		   	   	   const double leftKd,
		   	   	   const double rightKd,
		   	   	   const double otherKd)
{
	for (int i = 0; i < _traj.size(); i++) {
		PublishCommand(AK, _traj[i], leftKp, rightKp, otherKp, leftKd, rightKd, otherKd);
		loop_rate->sleep();
	}
}

/**********************************************
 * Init joint related ros stuff
 *********************************************/
void RosJointInit() {

	// Set name of joints to be commanded
	// (must match these inside AtlasPlugin)
	jointcommands.name.push_back("atlas::back_lbz");
	jointcommands.name.push_back("atlas::back_mby");
	jointcommands.name.push_back("atlas::back_ubx");

	jointcommands.name.push_back("atlas::neck_ay");

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

		printf(" [%d] Kp: %f, Ki: %f Kd: %f, I_clamp min: %f  clamp max: %f\n", i, jointcommands.kp_position[i], jointcommands.ki_position[i], jointcommands.kd_position[i], jointcommands.i_effort_min[i], jointcommands.i_effort_max[i]);
		jointcommands.velocity[i] = 0;
		jointcommands.effort[i] = 0;
		jointcommands.kp_velocity[i] = 0;
	}

	jointcommands_saved = jointcommands;
	jointangles.resize(jointcommands.name.size());
}


/**********************************************************
 * Read current actual joint angles,
 * update a global vector
 *********************************************************/
void UpdateDofs(const AtlasKinematics *AK, VectorXd &dofs) {

	/*********************************
	 * Read actual jointangles
	 *********************************/
	sleep(1);
	cout << "\nSpin Once NOW!" << endl;
	ros::spinOnce();


	/******************************
	 * Set dofs
	 ******************************/
	for (int i = 0; i < 4; i++) {
		dofs(AK->dof_misc[i]) = jointangles[i];
	}

	for (int j = 0; j < NUM_MANIPULATORS; j++) {
		for (int k = 0; k < 6; k++) {
			dofs(AK->dof_ind[j][k]) = jointangles[j*6+k+4];
		}
	}
}


/**
 * @function Read joint angles
 */
void GetJointStates( const sensor_msgs::JointState::ConstPtr &_js ) {

	for (int i = 0; i < _js->name.size(); i++)
		jointangles[i] = _js->position[i];

	//  cout << "Current joint angles:" << endl;
	//    cout << "\t" << _js->name[i] << "\t" << _js->position[i] << endl;


}


/************************************************
 * Publish a joint commands based on a dart pos
 ***********************************************/
void PublishCommand(AtlasKinematics *AK, const VectorXd &dofs, const double leftKp,
		const double rightKp, const double otherKp, const double leftKd,
		const double rightKd, const double otherKd) {

	for (int j = 0; j < 4; j++) {
		jointcommands.position[j] = dofs(AK->dof_misc[j]);
		jointcommands.kp_position[j] = jointcommands_saved.kp_position[j] * otherKp;
		jointcommands.kd_position[j] = jointcommands_saved.kd_position[j] * otherKd;
	}

	for (int j = 0; j < NUM_MANIPULATORS; j++) {
		for (int k = 0; k < 6; k++) {
			jointcommands.position[j*6+k+4] = dofs(AK->dof_ind[j][k]);
			jointcommands.kp_position[j*6+k+4] = jointcommands_saved.kp_position[j*6+k+4] * otherKp;
			jointcommands.kd_position[j*6+k+4] = jointcommands_saved.kd_position[j*6+k+4] * otherKd;
		}
	}

	// left foot kp, ki adjustment
	for (int k = 0; k < 6; k++) {
		jointcommands.kp_position[MANIP_L_FOOT*6+k+4] = jointcommands_saved.kp_position[MANIP_L_FOOT*6+k+4] * leftKp;
		jointcommands.kd_position[MANIP_L_FOOT*6+k+4] = jointcommands_saved.kd_position[MANIP_L_FOOT*6+k+4] * leftKd;
	}

	// right foot kp, ki adjustment
	for (int k = 0; k < 6; k++) {
		jointcommands.kp_position[MANIP_R_FOOT*6+k+4] = jointcommands_saved.kp_position[MANIP_R_FOOT*6+k+4] * rightKp;
		jointcommands.kd_position[MANIP_R_FOOT*6+k+4] = jointcommands_saved.kd_position[MANIP_R_FOOT*6+k+4] * rightKd;
	}

	pub_joint_commands_.publish( jointcommands );

}

/*************************************************
 * Move Atlas based on desired joint angles
 *************************************************/
void genDofTraj(AtlasKinematics *AK,
				const VectorXd &start_dofs,
				const VectorXd &end_dofs,
				std::vector<Eigen::VectorXd> &_traj,
				int N)
{
	// reserve memory for ticks
	_traj.reserve(_traj.size() + N);

	VectorXd tmpPos = start_dofs;
	VectorXd deltaPos = (end_dofs- tmpPos) / (N-1);

	for(int i=0; i < N; i++) {
		_traj.push_back(tmpPos);
		tmpPos += deltaPos;
	}
}

/*****************************************************
 * Move Atlas based on joint trajectory
 ********************************************************/
void MoveJointTractory(AtlasKinematics *AK,
					   Skeleton *_atlas,
					   VectorXd &dofs,
					   const std::vector<Eigen::VectorXd> &_zmp,
					   const double leftKp,
					   const double rightKp,
					   const double otherKp,
					   const double leftKd,
					   const double rightKd,
					   const double otherKd)
{

	/*************************************
	 * Update pos
	 *************************************/
	VectorXd dofs_save;
	dofs_save = _atlas->getPose();
	UpdateDofs(AK, dofs);

	cout << "*************************************" << endl;
	cout << "Before publishing trajectory" << endl;
	cout << "*************************************" << endl;
	cout << "POS ERROR: " << (dofs_save - dofs).norm() << endl;
	cout << "*************************************" << endl;

	for (int i = 0; i < _zmp.size(); i++) {

		for (int j = 0; j < 4; j++) {
			jointcommands.position[j] = _zmp[i][j];
			jointcommands.kp_position[j] = jointcommands_saved.kp_position[j] * otherKp;
			jointcommands.kd_position[j] = jointcommands_saved.kd_position[j] * otherKd;
		}

		for (int j = 0; j < NUM_MANIPULATORS; j++) {
			for (int k = 0; k < 6; k++) {
				jointcommands.position[j*6+k+4] = _zmp[i][j*6+k+4];
				jointcommands.kp_position[j*6+k+4] = jointcommands_saved.kp_position[j*6+k+4] * otherKp;
				jointcommands.kd_position[j*6+k+4] = jointcommands_saved.kd_position[j*6+k+4] * otherKd;
			}
		}

		// left foot kp, ki adjustment
		for (int k = 0; k < 6; k++) {
			jointcommands.kp_position[MANIP_L_FOOT*6+k+4] = jointcommands_saved.kp_position[MANIP_L_FOOT*6+k+4] * leftKp;
			jointcommands.kd_position[MANIP_L_FOOT*6+k+4] = jointcommands_saved.kd_position[MANIP_L_FOOT*6+k+4] * leftKd;
		}

		// right foot kp, ki adjustment
		for (int k = 0; k < 6; k++) {
			jointcommands.kp_position[MANIP_R_FOOT*6+k+4] = jointcommands_saved.kp_position[MANIP_R_FOOT*6+k+4] * rightKp;
			jointcommands.kd_position[MANIP_R_FOOT*6+k+4] = jointcommands_saved.kd_position[MANIP_R_FOOT*6+k+4] * rightKd;
		}

		pub_joint_commands_.publish( jointcommands );
		loop_rate->sleep();
	}


	/************************************************
	 * Update dofs in DART
	 ************************************************/
	for (int i = 0; i < 4; i++)
		dofs(AK->dof_misc[i]) = _zmp[_zmp.size()-1](i);

	for (int i = 0; i < NUM_MANIPULATORS; i++) {
		for (int j = 0; j < 6; j++) {
			dofs(AK->dof_ind[i][j]) = _zmp[_zmp.size()-1][i*6+j+4];
		}
	}

	dofs_save = dofs;
	UpdateDofs(AK, dofs);

	cout << "*************************************" << endl;
	cout << "After publishing trajectory" << endl;
	cout << "*************************************" << endl;
	cout << "POS ERROR: " << (dofs_save - dofs).norm() << endl;
	cout << "*************************************" << endl;

}




/*************************************************
 * Move Atlas based on comIK
 *************************************************/
void  genCOMIKTraj(AtlasKinematics *AK,
				   Skeleton *_atlas,
				   Matrix4d &Twb,
				   Matrix4d Twm[NUM_MANIPULATORS],
				   VectorXd &dofs,
				   Vector3d comDelta,
				   Vector3d leftDelta,
				   Vector3d rightDelta,
				   vector<VectorXd> &_traj,
				   const int N)
{
	/***************************************
	 * Intial
	 **************************************/
	double theta = M_PI / (N-1);

	_atlas->setPose(dofs, true);

	Vector3d comStart = AK->getCOMW(_atlas, Twb);
	Vector3d dcom = comStart;

	Twm[MANIP_L_FOOT] = AK->getLimbTransW(_atlas, Twb, MANIP_L_FOOT);
	Twm[MANIP_R_FOOT] = AK->getLimbTransW(_atlas, Twb, MANIP_R_FOOT);

	Vector3d leftStart = Twm[MANIP_L_FOOT].block<3,1>(0,3);
	Vector3d rightStart = Twm[MANIP_R_FOOT].block<3,1>(0,3);

	IK_Mode mode[NUM_MANIPULATORS];
	mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
	mode[MANIP_R_FOOT] = IK_MODE_WORLD;
	mode[MANIP_L_HAND] = IK_MODE_FIXED;
	mode[MANIP_R_HAND] = IK_MODE_FIXED;

	/**************************************
	 * Print out info
	 **************************************/
	comDelta /= 2;
	leftDelta /= 2;
	rightDelta /= 2;
	cout << "com delta: " << comDelta.transpose() << endl;
	cout << "left delta: " << leftDelta.transpose() << endl;
	cout << "right delta: " << rightDelta.transpose() << endl;

	cout << "com start: " << dcom.transpose() << endl;
	cout << "left start:\n" << Twm[MANIP_L_FOOT] << endl;
	cout << "right start:\n" << Twm[MANIP_R_FOOT] << endl;

	/****************************************
	 * COMIK
	 ***************************************/
	for (int i = 0; i < N; i++) {

		dcom = comStart + comDelta * (1 - cos(theta * i));
		Twm[MANIP_L_FOOT].block<3,1>(0,3) = leftStart + leftDelta * (1 - cos(theta * i));
		Twm[MANIP_R_FOOT].block<3,1>(0,3) = rightStart + rightDelta * (1 - cos(theta * i));

		assert(AK->comIK(_atlas, dcom, Twb, mode, Twm, dofs) == true);

		// fill trajectory
		_traj.push_back(dofs);
	}

	/********************************************
	 * Print end info
	 ********************************************/
	cout << "dcom end: " << dcom.transpose() << endl;
	cout << "left foot end:\n" << Twm[MANIP_L_FOOT] << endl;
	cout << "right foot end:\n" << Twm[MANIP_R_FOOT] << endl;
}


/*******************************************
 * Relax Atlas to a better pos for walking
 ********************************************/
void Relax(AtlasKinematics *AK,
		   Skeleton *_atlas,
		   VectorXd &dofs,
		   vector<VectorXd> &_traj,
		   int N)
{
	// Set joint angles of arms and body (no legs)
	dofs(6) = -3.852963655681663e-06;
	dofs(9) = 0.0009848090520510056;
	dofs(12) = 0.00010776096065789886;
	dofs(16) = 0.7806726847358707;

	//  dofs(7) = -0.0012852229728474995;
	//  dofs(10) = 0.061783845913243596;
	//  dofs(13) = -0.2856717835152658;
	//  dofs(18) = 0.5266262672930972;
	//  dofs(23) = -0.21864856475431704;
	//  dofs(27) = -0.062394234133471116;

	//  dofs(8) = 0.0013642411907399676;
	//  dofs(11) = -0.06195623610921519;
	//  dofs(14) = -0.2865128374461472;
	//  dofs(19) = 0.5268958272322948;
	//  dofs(24) = -0.21621680953724365;
	//  dofs(28) = 0.06138342176132294;

	dofs(15) = 0.2988754829726865;
	dofs(20) = -1.3138418263023999;
	dofs(25) = 2.00219166466513;
	dofs(29) = 0.4950932275951452;
	dofs(31) = -8.449255747589035e-05;
	dofs(33) = -0.010542899622185686;

	dofs(17) = 0.298867201336498138;
	dofs(22) = 1.313736629564075;
	dofs(26) = 2.0021752327042464;
	dofs(30) = -0.49508063541354375;
	dofs(32) = -8.39712346438759e-05;
	dofs(34) = 0.01056157776909128;

	// use legIK, move legs
	Matrix4d twb = Matrix4d::Identity();
	Matrix4d leftTarget = AK->getLimbTransB(_atlas, MANIP_L_FOOT);
	leftTarget(1,3) += 0.05;
	leftTarget(2,3) += 0.02;
	Matrix4d rightTarget = AK->getLimbTransB(_atlas, MANIP_R_FOOT);
	rightTarget(1,3) -= 0.05;
	rightTarget(2,3) += 0.02;
	VectorXd nearest(12);
	nearest.setZero();
	VectorXd legAngles(12);
	AK->stanceIK(twb, leftTarget, rightTarget, nearest, legAngles);

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 6; j++) {
			dofs(AK->dof_ind[i][j]) = legAngles(i*6+j);
		}
	}

	genDofTraj(AK, _atlas->getPose(), dofs, _traj, N);

	_atlas->setPose(dofs, true);
}
