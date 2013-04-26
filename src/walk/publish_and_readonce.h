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

using namespace std;
using namespace kinematics;
using namespace dynamics;
using namespace robotics;
using namespace Eigen;
using namespace atlas;


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
void PublishCommand(AtlasKinematics *AK, const VectorXd &dofs, const double leftKp,
                      const double rightKp, const double otherKp, const double leftKd,
                      const double rightKd, const double otherKd );


 /*************************************************
 * Move Atlas based on desired joint angles
 *************************************************/
void MoveDesireDofs(AtlasKinematics *AK, Skeleton *_atlas, const VectorXd &dofs, 
                      const double leftKp, const double rightKp, 
                      const double otherKp, const double leftKd, 
                      const double rightKd, const double otherKd, const int N);


/*****************************************************
 * Move Atlas based on joint trajectory
 ********************************************************/
void MoveJointTractory(AtlasKinematics *AK, Skeleton *_atlas, 
                        const std::vector<Eigen::VectorXd> &_zmp,
                        VectorXd &dofs,
                        const double leftKp, const double rightKp, 
                        const double otherKp, const double leftKd, 
                       const double rightKd, const double otherKd);


/**********************************************************
 * Move Atlas based on joint trajectory and support info
 **********************************************************/
void MoveJointTractoryAdv(AtlasKinematics *AK, Skeleton *_atlas, 
                        VectorXd &dofs,
                        const std::vector<Eigen::VectorXd> &_zmp,
                        const std::vector<int> supportInfo,
                        const VectorXd &double_support_kp, const VectorXd &double_support_ki,
                        const VectorXd &left_support_kp, const VectorXd &left_support_ki,
                        const VectorXd &right_support_kp, const VectorXd &right_support_ki);

/*************************************************
 * Move Atlas based on comIK
 *************************************************/
void  MoveCOMIK(AtlasKinematics *AK, 
            Skeleton *_atlas, 
            Matrix4d &Twb,
            Matrix4d Twm[NUM_MANIPULATORS],
            VectorXd &dofs, 
            Vector3d comDelta, 
            Vector3d leftDelta, 
            Vector3d rightDelta, 
            const double rightKp, 
            const double leftKp, 
            const double otherKp,
            const double leftKd,
            const double rightKd,
            const double otherKd,
            const int N); 


/*******************************************
 * Relax Atlas to a better pos for walking
 ********************************************/
void Relax(AtlasKinematics *AK, Skeleton *_atlas, VectorXd &dofs, double kp, double kd, int N);