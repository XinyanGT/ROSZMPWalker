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

#include "zmpUtilities/zmpUtilities.h"
#include "publish_and_readonce.h"

using namespace std;
using namespace kinematics;
using namespace dynamics;
using namespace robotics;
using namespace Eigen;
using namespace atlas;

atlas::AtlasKinematics *_ak;
kinematics::Skeleton *_atlas;
zmpUtilities gZU;

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;
osrf_msgs::JointCommands jointcommands_saved;
VectorXd jointangles;
ros::Rate *loop_rate;
ros::NodeHandle* rosnode;

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
void MoveDesireDofs(AtlasKinematics *AK, Skeleton *_atlas, const VectorXd &dofs, 
                      const double leftKp, const double rightKp, 
                      const double otherKp, const double leftKd, 
                      const double rightKd, const double otherKd, const int N) {

  VectorXd tmpPos = _atlas->getPose();
  VectorXd deltaPos = (dofs- tmpPos) / N;

  for ( int i = 0; i < N; i++) {

    PublishCommand(AK, tmpPos, leftKp, rightKp, otherKp, leftKd, rightKd, otherKd);
    loop_rate->sleep();
    tmpPos += deltaPos;
  }

}

/*****************************************************
 * Move Atlas based on joint trajectory
 ********************************************************/
void MoveJointTractory(AtlasKinematics *AK, Skeleton *_atlas, 
                        VectorXd &dofs,
                        const std::vector<Eigen::VectorXd> &_zmp,
                        const double leftKp, const double rightKp, 
                        const double otherKp, const double leftKd, 
                       const double rightKd, const double otherKd) {

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


/**********************************************************
 * Move Atlas based on joint trajectory and support info
 **********************************************************/
void MoveJointTractoryAdv(AtlasKinematics *AK, Skeleton *_atlas, 
                        VectorXd &dofs,
                        const std::vector<Eigen::VectorXd> &_zmp,
                        const std::vector<int> supportInfo,
                        const VectorXd &double_support_kp, const VectorXd &double_support_ki,
                        const VectorXd &left_support_kp, const VectorXd &left_support_ki,
                        const VectorXd &right_support_kp, const VectorXd &right_support_ki) 
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
    switch (supportInfo[i]) {

      case DOUBLE_SUPPORT:

        for (int j = 0; j < 4; j++) {
          jointcommands.position[j] = _zmp[i](j);
          jointcommands.kp_position[j] = jointcommands_saved.kp_position[j] * double_support_kp(j);
          jointcommands.kd_position[j] = jointcommands_saved.kd_position[j] * double_support_ki(j);
        }

        for (int j = 0; j < NUM_MANIPULATORS; j++) {
          for (int k = 0; k < 6; k++) {
            jointcommands.position[j*6+k+4] = _zmp[i](j*6+k+4);
            jointcommands.kp_position[j*6+k+4] = jointcommands_saved.kp_position[j*6+k+4] * 
                                                  double_support_kp(j*6+k+4);
            jointcommands.kd_position[j*6+k+4] = jointcommands_saved.kd_position[j*6+k+4] * 
                                                  double_support_ki(j*6+k+4);
          }
        }
        cout << "DOUBLE_SUPPORT: " << double_support_kp.transpose() << endl;
        break;

      case LEFT_SUPPORT:
        for (int j = 0; j < 4; j++) {
          jointcommands.position[j] = _zmp[i](j);
          jointcommands.kp_position[j] = jointcommands_saved.kp_position[j] * left_support_kp(j);
          jointcommands.kd_position[j] = jointcommands_saved.kd_position[j] * left_support_ki(j);
        }
        for (int j = 0; j < NUM_MANIPULATORS; j++) {
          for (int k = 0; k < 6; k++) {
            jointcommands.position[j*6+k+4] = _zmp[i](j*6+k+4);
            jointcommands.kp_position[j*6+k+4] = jointcommands_saved.kp_position[j*6+k+4] * 
                                                  left_support_kp(j*6+k+4);
            jointcommands.kd_position[j*6+k+4] = jointcommands_saved.kd_position[j*6+k+4] * 
                                                  left_support_ki(j*6+k+4);
          }
        }
      cout << "LEFT_SUPPORT: " << left_support_kp.transpose() << endl;
      break;

      case RIGHT_SUPPORT:
        for (int j = 0; j < 4; j++) {
          jointcommands.position[j] = _zmp[i](j);
          jointcommands.kp_position[j] = jointcommands_saved.kp_position[j] * right_support_kp(j);
          jointcommands.kd_position[j] = jointcommands_saved.kd_position[j] * right_support_ki(j);
        }
        for (int j = 0; j < NUM_MANIPULATORS; j++) {
          for (int k = 0; k < 6; k++) {
            jointcommands.position[j*6+k+4] = _zmp[i](j*6+k+4);
            jointcommands.kp_position[j*6+k+4] = jointcommands_saved.kp_position[j*6+k+4] * 
                                                  right_support_kp(j*6+k+4);
            jointcommands.kd_position[j*6+k+4] = jointcommands_saved.kd_position[j*6+k+4] * 
                                                  right_support_ki(j*6+k+4);
          }
        }
      cout << "RIGHT_SUPPORT: " << right_support_kp.transpose() << endl;
      break;

    }
    cout << "JOINT_KP:";
    for (int i = 0; i < 28; i++)
      cout << "\t" << jointcommands.kp_position[i];

    cout << endl;
    cout << "JOINT_KD:";
    for (int i = 0; i < 28; i++)
      cout << "\t" << jointcommands.kd_position[i];

    cout << endl;
    pub_joint_commands_.publish( jointcommands );
    loop_rate->sleep();
  }


  /************************************************
   * Update dofs in DART
   ************************************************/
  for (int i = 0; i < 4; i++) {
    dofs(AK->dof_misc[i]) = _zmp[_zmp.size()-1](i);
  }

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
void  MoveCOMIK(AtlasKinematics *AK, 
            Skeleton *_atlas, 
            Matrix4d &Twb,
            Matrix4d Twm[NUM_MANIPULATORS],
            VectorXd &dofs, 
            Vector3d comDelta, 
            Vector3d leftDelta, 
            Vector3d rightDelta, 
            const double leftKp, 
            const double rightKp, 
            const double otherKp,
            const double leftKd, 
            const double rightKd, 
            const double otherKd,
            const int N) {

  /*************************************
   * Update pos
   *************************************/
  VectorXd dofs_save; 
  dofs_save = _atlas->getPose();
  UpdateDofs(AK, dofs);

  cout << "*************************************" << endl;
  cout << "POS ERROR: " << (dofs_save - dofs).norm() << endl;
  cout << "*************************************" << endl;

////  assert( (dofs_save - dofs).norm() < 1e-4);
////  _atlas->setPose(dofs, true);

//  /***************************************
//   * Clean up error
//   **************************************/
//  MoveDesireDofs(AK, _atlas, dofs_save, 20, 20, 20, 1.2, 1.2, 1.2, 600);
//  UpdateDofs(AK, dofs);
//  cout << "*************************************" << endl;
//  cout << "^^^^AFTER CLEAN POS ERROR: " << (dofs_save - dofs).norm() << endl;
//  cout << "*************************************" << endl;


  dofs = _atlas->getPose();
  /***************************************
   * Intial 
   **************************************/
  double theta = M_PI / (N-1);   

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

  dofs = _atlas->getPose();


  /****************************************
   * COMIK and publish
   ***************************************/
  for (int i = 0; i < N; i++) {

    dcom = comStart + comDelta * (1 - cos(theta * i));
    Twm[MANIP_L_FOOT].block<3,1>(0,3) = leftStart + leftDelta * (1 - cos(theta * i));
    Twm[MANIP_R_FOOT].block<3,1>(0,3) = rightStart + rightDelta * (1 - cos(theta * i));

    assert(AK->comIK(_atlas, dcom, Twb, mode, Twm, dofs) == true);

    dofs = _atlas->getPose();

    PublishCommand(AK, dofs, leftKp, rightKp, otherKp, leftKd, rightKd, otherKd);
    loop_rate->sleep();

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
void Relax(AtlasKinematics *AK, Skeleton *_atlas, VectorXd &dofs, double kp, double kd, int N) {

  ros::Publisher gPubMode;

  // Advertise in atlas/mode
  gPubMode = rosnode->advertise<std_msgs::String>( "atlas/mode", 100, true );

  dofs(6) = -3.852963655681663e-06;
  dofs(9) = 0.0009848090520510056;
  dofs(12) = 0.00010776096065789886; 
  dofs(16) = 0.7806726847358707;

//  dofs(7) = -0.0012852229728474995;
//  dofs(10) = 0.061783845913243596;
//  dofs(13) = -0.2856717835152658;
//  dofs(18) = 0.5266262672930972; 
//  dofs(23) = -0.21864856475431704; 
////  dofs(27) = -0.062394234133471116;
////  dofs(27) = -dofs(10);

//  dofs(8) = 0.0013642411907399676;
//  dofs(11) = -0.06195623610921519; 
//  dofs(14) = -0.2865128374461472; 
//  dofs(19) = 0.5268958272322948;
//  dofs(24) = -0.21621680953724365;
////  dofs(28) = 0.06138342176132294;
////  dofs(28) = -dofs(11);

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
  leftTarget(2,3) += 0.03;
  leftTarget(1,3) += 0.03;
//  leftTarget(1,3) += 0.1;
  Matrix4d rightTarget = AK->getLimbTransB(_atlas, MANIP_R_FOOT);
  rightTarget(2,3) += 0.03;
  rightTarget(1,3) -= 0.03;
//  rightTarget(1,3) -= 0.1;
  VectorXd nearest(12);
  nearest.setZero();
  VectorXd legAngles(12);
  AK->stanceIK(twb, leftTarget, rightTarget, nearest, legAngles);
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 6; j++) {
      dofs(AK->dof_ind[i][j]) = legAngles(i*6+j);
    }
  }

  dofs(7) = 0.062852229728474995;
  dofs(8) = -0.062642411907399676;
  // Get init position
  // Set in harness mode
  printf("Setting mode no_gravity \n");
  std_msgs::String mode;
  mode.data = "pinned";
//  mode.data = "no_gravity";
  gPubMode.publish( mode );
  printf("Out of setting \n");
  // Set robot in starting position  
  printf("Sending initial position \n");
  MoveDesireDofs(AK, _atlas, dofs, kp, kp, kp, kd, kd, kd, N);
  sleep(2);

  printf("Just sent initial position \n");

  _atlas->setPose(dofs, true);

//  mode.data = "feet";
// gPubMode.publish( mode );
  
  mode.data = "nominal";
  gPubMode.publish( mode );
}


/**
 * @function main
 */
int main( int argc, char** argv ) {
 

  /********************************************
   * Configure ROS
   *****************************************/
  double frequency = 200;
  ros::init( argc, argv, "publish_and_readonce" );
  rosnode = new ros::NodeHandle();
  loop_rate = new ros::Rate(frequency);

  ros::Time last_ros_time_;
  // Wait until sim is active (play)
  bool wait = true;

  while( wait ) {
    last_ros_time_ = ros::Time::now();
    if( last_ros_time_.toSec() > 0 ) {
      wait = false;
    }
  }

  // init ros joints
  RosJointInit();

  // ros topic subscribtions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, GetJointStates,
    ros::VoidPtr(), rosnode->getCallbackQueue());

  jointStatesSo.transport_hints = ros::TransportHints().unreliable();
  ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);

  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);



  /***************************************************************
   * Kinematics happends
   ****************************************************************/
  if(!_ak) {
    DartLoader dart_loader;
    World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH "atlas/atlas_world.urdf");
    _atlas = mWorld->getSkeleton("atlas");
    _ak = new AtlasKinematics();
    _ak->init(_atlas);
  }
  _atlas->setPose(_atlas->getPose().setZero(), true);

  AtlasKinematics *AK = _ak;


  /*************************
   * Move whole body, after that:
   * World starts
   *************************/

  int nDofsNum = _atlas->getNumDofs();
  VectorXd dofs(nDofsNum);
  dofs.setZero();

  _atlas->setPose(dofs, true);

  Relax(AK, _atlas, dofs, 10, 1.2, 1000);



  /***********************************************
   * World starts from current pelvis
   ************************************************/

  /******************************
   * Some vars
   ******************************/
  IK_Mode mode[NUM_MANIPULATORS];
  mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
  mode[MANIP_R_FOOT] = IK_MODE_WORLD;
  mode[MANIP_L_HAND] = IK_MODE_FIXED;
  mode[MANIP_R_HAND] = IK_MODE_FIXED;

  Vector3d comDelta = Vector3d::Zero();
  Vector3d leftDelta = Vector3d::Zero();
  Vector3d rightDelta = Vector3d::Zero();

  int N = 0;

  Matrix4d Twm[NUM_MANIPULATORS];
  Twm[MANIP_L_FOOT] = AK->getLimbTransB(_atlas, MANIP_L_FOOT);
  Twm[MANIP_R_FOOT] = AK->getLimbTransB(_atlas, MANIP_R_FOOT);

  Matrix4d Twb;
  Twb.setIdentity();

  VectorXd dofs_save;


  /*************************
   * Move COM down
   *************************/
   
  comDelta << 0, 0, -0.05;
  leftDelta.setZero();
  rightDelta.setZero();
  cout << "***************************************" << endl;
  cout << "Move COM dowm" << endl;
  MoveCOMIK(AK, _atlas, Twb, Twm, dofs, comDelta, leftDelta, rightDelta, 10, 10, 10, 1.2, 1.2, 1.2, 1000);


//  /*******************************
//   * Move COM to right root
//   *******************************/
//  comDelta <<  0, -1 *( AK->getLimbTransW(_atlas, Twb, MANIP_L_FOOT).block<3,1>(0,3) - AK->getCOMW(_atlas, Twb))(1), 0;
//  leftDelta.setZero();
//  rightDelta.setZero();
//  cout << "***************************************" << endl;
//  cout << "Move COM to right foot" << endl;
//  MoveCOMIK(AK, _atlas, Twb, Twm, dofs, comDelta, leftDelta, rightDelta, 30, 20, 20, 1.3, 1.2, 1.2, 2000);

  

  /************************************
   * ZMP walking
   ************************************/

  /*************************************
   * Set parameters for ZMP walker
   *************************************/
  // number of steps to walk
  int numSteps = 30;
  // length of a half step
  double stepLength = 0.15;
//  double stepLength = 0.2;

  // foot seperation
  dofs_save = _atlas->getPose();
  UpdateDofs(AK, dofs);

  cout << "********************************************" << endl;
  cout << "Start ZMP walking" << endl;
  cout << "*************************************" << endl;
  cout << "POS ERROR: " << (dofs_save - dofs).norm() << endl;
  cout << "*************************************" << endl;

  _atlas->setPose(dofs);

  double footSeparation = (AK->getLimbTransW(_atlas, Twb, MANIP_L_FOOT)(1,3) - 
                          AK->getLimbTransW(_atlas, Twb, MANIP_R_FOOT)(1,3) );
  cout << "foot seperation: " << footSeparation << endl;
  // one step time
//  double stepDuration = 1.0;

  double stepDuration = 3;
  // move ZMP time
  double slopeTime = 1;
  // keep ZMP time
  double levelTime = 2;

  // command sending period
  double dt = 1/frequency;
  // height of COM
  double zg = AK->getCOMW(_atlas, Twb)(2) - AK->getLimbTransW(_atlas, Twb, MANIP_L_FOOT)(2,3);
  cout << "zg " << zg << endl;
  // preview how many steps
  int numPreviewSteps = 2;

// double Qe = 1;
// double R = 1e-6;
  double Qe = 1e7;
  double R = 10;
  double default_kp = 20;
  double strong_kp = 20;
  double weak_kp = 20;

  double default_ki = 1.3;
  double strong_ki = 1.3;
  double weak_ki = 1.3;

  VectorXd double_support_kp, double_support_ki;
  VectorXd left_support_kp, left_support_ki;
  VectorXd right_support_kp, right_support_ki;

  double_support_kp = VectorXd::Constant(28, default_kp);
  double_support_ki = VectorXd::Constant(28, default_ki);

  left_support_kp.resize(28);
  left_support_ki.resize(28);
  right_support_kp.resize(28);
  right_support_ki.resize(28);
  
  left_support_kp << default_kp, default_kp, default_kp, default_kp,
                     default_kp,  strong_kp, 40, strong_kp, 40, strong_kp, 
                     weak_kp, weak_kp, 40, weak_kp, 20, weak_kp,
                     default_kp, default_kp, default_kp, default_kp, default_kp, default_kp,         
                     default_kp, default_kp, default_kp, default_kp, default_kp, default_kp,


  left_support_ki << default_ki, default_ki, default_ki, default_ki,
                     strong_ki,  strong_ki, strong_ki, strong_ki, strong_ki, strong_ki, 
                     weak_ki, weak_ki, weak_ki, weak_ki, weak_ki, weak_ki,
                     default_ki, default_ki, default_ki, default_ki, default_ki, default_ki,
                     default_ki, default_ki, default_ki, default_ki, default_ki, default_ki;

  right_support_kp << default_kp, default_kp, default_kp, default_kp,
                     weak_kp, weak_kp, 40, weak_kp, 20, weak_kp,
                     strong_kp,  strong_kp, 40, strong_kp, 40, strong_kp, 
                     default_kp, default_kp, default_kp, default_kp, default_kp, default_kp,
                     default_kp, default_kp, default_kp, default_kp, default_kp, default_kp;
                  
 
  right_support_ki << default_ki, default_ki, default_ki, default_ki,
                     weak_ki, weak_ki, weak_ki, weak_ki, weak_ki, weak_ki,
                     strong_ki,  strong_ki, strong_ki, strong_ki, strong_ki, strong_ki, 
                     default_ki, default_ki, default_ki, default_ki, default_ki, default_ki,
                     default_ki, default_ki, default_ki, default_ki, default_ki, default_ki;


  cout << "double kp: " << double_support_kp.transpose() << endl;
  cout << "double ki: " << double_support_ki.transpose() << endl;
  cout << "left kp: " << left_support_kp.transpose() << endl;
  cout << "left ki: " << left_support_ki.transpose() << endl;
  cout << "right kp: " << right_support_kp.transpose() << endl;
  cout << "right ki: " << right_support_ki.transpose() << endl;


  /****************************************
  * Generate joint trajectory
  ***************************************/
  gZU.setParameters( dt, 9.81, dofs );
  gZU.generateZmpPositions( numSteps, true,
          stepLength, footSeparation,
          stepDuration,
          slopeTime,
          levelTime );

  gZU.getControllerGains( Qe, R, zg, numPreviewSteps );
  gZU.generateCOMPositions();
  gZU.getJointTrajectories();
  gZU.print( "jointsWholeBody.txt", gZU.mWholeBody );

  // Controller gains G1, G2
  cout << "G1: " << gZU.mG1 << endl;
  cout << "G2: " << gZU.mG2 << endl;

  for (int i = 0; i < gZU.mSupportMode.size(); i++)
    cout << gZU.mSupportMode[i];

  MoveJointTractoryAdv(AK, _atlas, dofs, gZU.mWholeBody, gZU.mSupportMode, 
                        double_support_kp,double_support_ki,
                        left_support_kp, left_support_ki,
                        right_support_kp, right_support_ki);

  
  

//  /******************************
//   * Lift up left foot
//   ********************************/
//  comDelta.setZero();
//  leftDelta << 0, 0, 0.01;
//  rightDelta.setZero();
//  cout << "***************************************" << endl;
//  cout << "Lift up left foot" << endl;
//  MoveCOMIK(AK, _atlas, Twb, Twm, dofs, comDelta, leftDelta, rightDelta, 14, 8, 8, 1.4, 1, 1, 2000);


//  /**************************************
//   * Put left foot down
//   **************************************/
//  comDelta.setZero();
//  leftDelta << 0, 0, -0.01;
//  rightDelta.setZero();
//  cout << "***************************************" << endl;
//  cout << "Put left foot down" << endl;
//  MoveCOMIK(AK, _atlas, Twb, Twm, dofs, comDelta, leftDelta, rightDelta, 8, 8, 8, 1, 1, 1, 2000);


////  AK->printGazeboAngles(_atlas, _atlas->getPose());



  return 0;

}
