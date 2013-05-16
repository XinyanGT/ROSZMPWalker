/** 
 * @file utilities.cpp
 * @brief Functions to calculate preview control stuff and foot print generation
 */
#include "zmpUtilities.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <algorithm> // for fill


// To use dart-atlas library
#include <atlas/AtlasUtils.h>
#include <utils/AtlasPaths.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <dynamics/SkeletonDynamics.h>

#include <fstream>
#include <string>

// For footstep generation
#include <tinyWalker/zmp/atlas-zmp.h>

/**
 * @function zmpUtilities
 * @brief Constructor
 */
zmpUtilities::zmpUtilities() {
  
  mDofIndices.resize(0);


  // Torso and neck
  mDofIndices.push_back(6); // back_lbz
  mDofIndices.push_back(9); // back_mby
  mDofIndices.push_back(12); // back_ubx
  mDofIndices.push_back(16); // neck_ay


  // Left leg
  mDofIndices.push_back(7);
  mDofIndices.push_back(10);
  mDofIndices.push_back(13);
  mDofIndices.push_back(18);
  mDofIndices.push_back(23);
  mDofIndices.push_back(27);

  // Right leg
  mDofIndices.push_back(8);
  mDofIndices.push_back(11);
  mDofIndices.push_back(14);
  mDofIndices.push_back(19);
  mDofIndices.push_back(24);
  mDofIndices.push_back(28);

  // Left arm
  mDofIndices.push_back(15);
  mDofIndices.push_back(20);
  mDofIndices.push_back(25);
  mDofIndices.push_back(29);
  mDofIndices.push_back(31);
  mDofIndices.push_back(33);

  // Right arm
  mDofIndices.push_back(17);
  mDofIndices.push_back(22);
  mDofIndices.push_back(26);
  mDofIndices.push_back(30);
  mDofIndices.push_back(32);
  mDofIndices.push_back(34);

}

/**
 * @function ~zmpUtilities
 * @brief Destructor
 */
zmpUtilities::~zmpUtilities() {

}

/**
 * @function setParameters
 */
void zmpUtilities::setParameters( const double &_dt,
				  const double &_g,
				  const Eigen::VectorXd &_initDofs ) {
  mdt = _dt;
  mG = _g;
  mInitDofVals = _initDofs; // more than 28...
}


/** Generate zmp x and y positions for a straight walk */
void zmpUtilities::generateZmpPositions2( int _numSteps,
					  const bool &_startLeftFoot,
					  const double &_stepLength,
					  const double &_footSeparation,
					  const double &_stepDuration,
					  const double &_slopeTime,
					  const double &_levelTime,
					  const int &_numWaitSteps,
					  const double &_comZ ) {

  // Set the variables
  mStepLength = _stepLength;
  mFootSeparation = _footSeparation;
  mStepDuration = _stepDuration;

  // Variables
  double walk_dist = 1.0;

  double foot_separation_y = _footSeparation / 2.0; // half of total separation
  double foot_liftoff_z = 0.05;

  double step_length = mStepLength; // Total step 2X
  bool walk_sideways = false;

  double com_height = _comZ;
  double com_ik_ascl = 0;

  
  double zmpoff_y = 0; // lateral displacement between zmp and ankle
  double zmpoff_x = 0;

  double lookahead_time = 1*_stepDuration; 
  double startup_time = _numWaitSteps*_stepDuration;
  double shutdown_time = _numWaitSteps*_stepDuration;
  double double_support_time = _slopeTime;
  double single_support_time = _levelTime;

  size_t max_step_count = 20;
  double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller
  
  ZMPWalkGenerator::ik_error_sensitivity ik_sense = ZMPWalkGenerator::ik_strict;
  

  //////////////////////////////////////////////////////////////////////
  // build initial state

  // the actual state
  ZMPWalkGenerator walker(ik_sense,
                          com_height,
                          zmp_jerk_penalty,
			  zmpoff_x,
			  zmpoff_y,
                          com_ik_ascl,
                          single_support_time,
                          double_support_time,
                          startup_time,
                          shutdown_time,
                          foot_liftoff_z,
			  lookahead_time);
  printf("COM Height: %f single st: %f double st: %f startup time: %f, shutdown time: %f foot liftoff: %f lookahead: %f \n", com_height, single_support_time,
	 double_support_time, startup_time, shutdown_time, foot_liftoff_z, lookahead_time );
  ZMPReferenceContext initContext;
  
  // helper variables and classes
  double deg = M_PI/180; // for converting from degrees to radians
  
  // fill in the kstate
  //initContext.state.body_pos = vec3(0, 0, 0.85);
  //initContext.state.body_rot = quat();
  
  // build and fill in the initial foot positions
  Transform3 starting_location(quat::fromAxisAngle(vec3(0,0,1), 0));
  initContext.feet[0] = Transform3(starting_location.rotation(), starting_location * vec3(0, foot_separation_y, 0));
  initContext.feet[1] = Transform3(starting_location.rotation(), starting_location * vec3(0, -foot_separation_y, 0));
  
  // fill in the rest
  initContext.stance = DOUBLE_LEFT;
  initContext.comX = Eigen::Vector3d(zmpoff_x, 0.0, 0.0);
  initContext.comY = Eigen::Vector3d(0.0, 0.0, 0.0);
  initContext.eX = 0.0;
  initContext.eY = 0.0;
  initContext.pX = 0.0;
  initContext.pY = 0.0;
  
  // apply COM IK for init context
  //walker.applyComIK(initContext);
  
  walker.traj.resize(1);
  walker.refToTraj(initContext, walker.traj.back());
  
  walker.initialize(initContext);

  
  //////////////////////////////////////////////////////////////////////
  // build ourselves some footprints
  
  Footprint initLeftFoot = Footprint(initContext.feet[0], true);
  /* Footprint initRightFoot = Footprint(initContext.feet[1], false); */

  std::vector<Footprint> footprints;    
    
  footprints = walkLine(walk_dist, foot_separation_y,
			step_length,
			initLeftFoot);
  printf("Size footprint: %d \n", footprints.size());
  
  if (footprints.size() > max_step_count) {
    footprints.resize(max_step_count);
  }

  //////////////////////////////////////////////////////////////////////
  // and then build up the walker
  printf("Size walk ref before startup: %d \n", walker.ref.size() );
  walker.stayDogStay(startup_time * TRAJ_FREQ_HZ);
    printf("Size walk ref after startup: %d \n", walker.ref.size() );
  for(std::vector<Footprint>::iterator it = footprints.begin(); it != footprints.end(); it++) {
    walker.addFootstep(*it);
  }
  printf("Size walk ref after steps: %d \n", walker.ref.size() );
  walker.stayDogStay(shutdown_time * TRAJ_FREQ_HZ);
   printf("Size walk ref after shutdown: %d \n", walker.ref.size() );
  printf("FREQ: %d \n", TRAJ_FREQ_HZ);
  printf("Num traj points: %d \n", walker.ref.size() );

  // Store
  // Store ZMP, Left foot, right foot and Support Mode
  Eigen::Vector2d zmp;
  Eigen::Vector3d leftFoot;
  Eigen::Vector3d rightFoot;
  int support;
/*
  mZMPT.resize(0);
  mLeftFootT.resize(0);
  mRightFootT.resize(0);
  mSupportModeT.resize(0);

  for( int i = 0; i < walker.ref.size(); ++i ) {
    zmp << walker.ref[i].pX, walker.ref[i].pY;
    leftFoot << walker.ref[i].feet[0].matrix()(0,3), walker.ref[i].feet[0].matrix()(1,3), walker.ref[i].feet[0].matrix()(2,3);
    rightFoot << walker.ref[i].feet[1].matrix()(0,3), walker.ref[i].feet[1].matrix()(1,3), walker.ref[i].feet[1].matrix()(2,3);
    support << walker.ref[i].stance;
    
    mZMPT.push_back( zmp );
    mLeftFootT.push_back( leftFoot );
    mRightFootT.push_back( rightFoot );
    mSupportModeT.push_back( support );

  }
  */

  mZMP.resize(0);
  mLeftFoot.resize(0);
  mRightFoot.resize(0);
  mSupportMode.resize(0);

  for( int i = 0; i < walker.ref.size(); ++i ) {
    zmp << walker.ref[i].pX, walker.ref[i].pY;
    leftFoot << walker.ref[i].feet[0].matrix()(0,3), walker.ref[i].feet[0].matrix()(1,3), walker.ref[i].feet[0].matrix()(2,3);
    rightFoot << walker.ref[i].feet[1].matrix()(0,3), walker.ref[i].feet[1].matrix()(1,3), walker.ref[i].feet[1].matrix()(2,3);
    support << walker.ref[i].stance;
    
    mZMP.push_back( zmp );
    mLeftFoot.push_back( leftFoot );
    mRightFoot.push_back( rightFoot );
    mSupportMode.push_back( support );

  }

}


/**
 * @function getControllerGains
 */
void zmpUtilities::getControllerGains( const double &_Qe,
				       const double &_R,
				       const double &_z_COM,
				       const int &_numPreviewSteps ) {

  // Controller
  Eigen::MatrixXd Qe, Qx, R;
  
  // Controller helpers
  Eigen::MatrixXd _A, _B;
  Eigen::MatrixXd _Q, _I, W, K;

  //-- Store Num preview steps
  mN = _numPreviewSteps*( mStepDuration / mdt );
  mzCOM = _z_COM;

  //-- Set dynamics
  mA = Eigen::MatrixXd::Zero( 6, 6 );
  mA.block( 0, 0, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );
  mA.block( 2, 2, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );
  mA.block( 4, 4, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );

  mA.block( 0, 2, 2, 2 ) = mdt*Eigen::MatrixXd::Identity( 2, 2 );
  mA.block( 0, 4, 2, 2 ) = (mdt*mdt/2.0)*Eigen::MatrixXd::Identity( 2, 2 );
  mA.block( 2, 4, 2, 2 ) = mdt*Eigen::MatrixXd::Identity( 2, 2 );

  mB = Eigen::MatrixXd::Zero( 6, 2 );
  mB.block( 0, 0, 2, 2 ) = (mdt*mdt*mdt/6.0)*Eigen::MatrixXd::Identity( 2, 2 );
  mB.block( 2, 0, 2, 2 ) = (mdt*mdt/2.0)*Eigen::MatrixXd::Identity( 2, 2 );
  mB.block( 4, 0, 2, 2 ) = mdt*Eigen::MatrixXd::Identity( 2, 2 );

  mC = Eigen::MatrixXd::Zero( 2, 6 );
  mC.block( 0, 0, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );
  mC.block( 0, 4, 2, 2 ) = -(mzCOM / mG )*Eigen::MatrixXd::Identity( 2, 2 );

  
  //-- Set LQI gains for optimal controller
  Qe = _Qe*Eigen::MatrixXd::Identity( 2, 2 );
  Qx = Eigen::MatrixXd::Zero( 6, 6 );
  R = _R*Eigen::MatrixXd::Identity( 2, 2 );
  

  //-- Set auxiliar matrices to calculate preview controller gains
  // _A: 8x8
  _A = Eigen::MatrixXd::Zero( 8, 8 );
  _A.block( 0, 0, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );
  _A.block( 0, 2, 2, 6 ) = mC*mA;
  _A.block( 2, 2, 6, 6 ) = mA;

  // _B: 8x2
  _B = Eigen::MatrixXd::Zero( 8, 2 );
  _B.block( 0, 0, 2, 2 ) = mC*mB;
  _B.block( 2, 0, 6, 2 ) = mB;
  
  // _Q: 8x8
  _Q = Eigen::MatrixXd::Zero( 8, 8 );
  _Q.block( 0, 0, 2, 2 ) = Qe;
  _Q.block( 2, 2, 6, 6 ) = Qx;

  // _I: 8x2
  _I = Eigen::MatrixXd::Zero(8,2);
  _I.block(0,0,2,2) = Eigen::MatrixXd::Identity(2,2);
  
  iterative_DARE( K, _A, _B, _Q, R );

  Eigen::MatrixXd temp = ( R + _B.transpose()*K*_B );
  Eigen::MatrixXd tempInv(2,2);
  double a, b, c, d;
  a = temp(0,0); b = temp(0,1); c = temp(1,0); d = temp(1,1);
  tempInv << d, -b, -c, a;
  tempInv = tempInv*( 1.0/(a*d - b*c) );

  std::cout << "K from Iterative: \n" << K <<std::endl;

  // W
  W = tempInv*_B.transpose();

  // G1
  mG1 = W*K*_I;

  // G2
  mG2 = W*K*_A.block( 0, 2, 8, 6 );
  
  // G3
  mG3.resize(0);
  Eigen::MatrixXd G3i = Eigen::MatrixXd::Zero( 2, 2 );

  Eigen::MatrixXd exp = Eigen::MatrixXd::Identity(8,8);
  Eigen::MatrixXd factor1 = ( _A - _B*W*K*_A ).transpose();
  
  // i = 0
  mG3.push_back( G3i );
  
  // i = 1
  G3i = -W*exp*K*_I;
  mG3.push_back( G3i );

  // i = 2 till _numPreviewSteps]
  for( int i = 2; i <= mN; ++i ) {
    exp = exp*factor1;
    G3i = -W*exp*K*_I;
    mG3.push_back( G3i );
  }
  
}

/**
 * @function iterative_DARE
 */
bool zmpUtilities::iterative_DARE( Eigen::MatrixXd &_P, 
				   const Eigen::MatrixXd &_A,
				   const Eigen::MatrixXd &_B,
				   const Eigen::MatrixXd &_Q,
				   const Eigen::MatrixXd &_R,
				   const double &_error_threshold,
				   const int &_numIter ) {
  
  Eigen::MatrixXd At = _A.transpose();
  Eigen::MatrixXd Bt = _B.transpose();
  Eigen::MatrixXd temp; 
  Eigen::MatrixXd tempInv( 2, 2 );
  double a, b, c, d;

  Eigen::MatrixXd Pnew, Pt;

  _P = Eigen::MatrixXd::Identity(8,8);
  Pnew = _P; Pt = _P.transpose();
  
  double error;

  for( int i = 0; i < _numIter; ++i ) {
    temp = ( _R + Bt*_P*_B );
    a = temp(0,0); b = temp(0,1); c = temp(1,0); d = temp(1,1);
    tempInv << d, -b, -c, a;
    tempInv = tempInv*( 1.0/(a*d - b*c) );


    Pnew = At*_P*_A - At*_P*_B*tempInv*Bt*Pt*_A + _Q;
    error = ( (Pnew - _P).norm() ) / ( Pnew.norm() );
    _P = Pnew;
    Pt = _P.transpose();
    if( error < _error_threshold ) {
      printf("DARE converges after %d iterations with an error of %f \n", i, error );
      return true;
    }

  }

  printf("DARE did NOT converge! threshold error: %f Last error in iter[%d]: %f \n", _error_threshold,
	 _numIter, error );
  return false;
  
}


/**
 * @function generateCOMPositions
 */
void zmpUtilities::generateCOMPositions( ) {

  // x: [ pos_x, pos_y, vel_x, vel_y, acc_x, acc_y ]^T  
  Eigen::MatrixXd x(6,1);
  // y: [ zmp_x, zmp_y ]^T
  Eigen::MatrixXd y(2,1);
  // u: [ u_x, u_y ]^T
  Eigen::MatrixXd u(2,1);

  Eigen::MatrixXd G3;
  Eigen::MatrixXd old_x;
  
  // t = 0
  x << 0, 0, 0, 0, 0, 0;
  y = mC*x;
  u << 0, 0;
  old_x = x;
  
  // Add
  mX.resize(0); mY.resize(0); mU.resize(0);
  mX.push_back( x.block(0,0,2,1) );
  mY.push_back( y );
  mU.push_back( u );
 
  // t = 1*dt to ... 
  printf("[COM Calculation] zmp size: %d n: %d \n", mZMP.size(), mN );
  printf("Calculating from i: %d to i: %d \n", 0, mZMP.size() - mN - 1 );
  for( int i = 0; i < mZMP.size() - mN - 1; ++i ) {
    x = mA*x + mB*u;
    y = mC*x;

    // preview factor
    G3 = Eigen::MatrixXd::Zero(2,1);

    for( int j = 1; j < mN; ++j ) {
      G3 = G3 + mG3[j]*( mZMP[i+j] - mZMP[i+j-1] );
    }
    
    u = u - mG1*(y - mZMP[i]) - mG2*(x - old_x) - G3;  

    old_x = x;

    // Add
    mX.push_back( x.block(0,0,2,1) );
    mY.push_back( y );
    mU.push_back( u );
  }
}

/**
 * @function getJointTrajectories 
 */
void zmpUtilities::getJointTrajectories() {
  
  /***************************
   * DOF number in dart
   ***************************/
  int l[6], r[6];
  l[0] = 7;  //= l_leg_uhz
  l[1] = 10; //= l_leg_mhx
  l[2] = 13; //= l_leg_lhy
  l[3] = 18; //= l_leg_kny
  l[4] = 23; //= l_leg_uay
  l[5] = 27; //= l_leg_lax
  
  r[0] = 8;  //= r_leg_uhz
  r[1] = 11; //= r_leg_mhx
  r[2] = 14; //= r_leg_lhy
  r[3] = 19; //= r_leg_kny
  r[4] = 24; //= r_leg_uay
  r[5] = 28; //= r_leg_lax
  

  /******************
   * Declare and Init
   ******************/
   printf("Atlas Kin \n");
  atlas::AtlasKinematics *AK = prepareAtlasKinematics();
  
  int nDofsNum = mAtlasSkel->getNumDofs();
  Eigen::VectorXd dofs(nDofsNum);
  dofs.setZero();
  printf("Num 	DOFs: %d \n", dofs.size() );
  

  //
  for( int i = 0; i < dofs.size(); ++i ) {
    dofs(i) = mInitDofVals(i);
  }

  
  std::cout << "before set: " << dofs << std::endl;
  mAtlasSkel->setPose(dofs, true, false);
  dofs = mAtlasSkel->getPose();
  std::cout << "after set: " << dofs << std::endl;

  
  std::cout << "left foot: \n" << mAtlasSkel->getNode("l_foot")->getWorldTransform() << std::endl;
  std::cout << "right foot: \n" << mAtlasSkel->getNode("r_foot")->getWorldTransform() << std::endl;
  
  Eigen::Vector3d com, comStart;
  comStart = com = mAtlasSkel->getWorldCOM();
  std::cout << "com: " << comStart << std::endl;
  
  Eigen::Matrix4d Twb;
  Twb.setIdentity();
  
  
  Eigen::Vector6d lleg_angle, rleg_angle;
  for (int i = 0; i < 6; i++) {
    rleg_angle(i) = dofs(r[i]);
    lleg_angle(i) = dofs(l[i]);
  }
  
  Eigen::Matrix4d TwlStart = AK->legFK(lleg_angle, true);
  //	TwlStart.col(3) = _atlas->getNode("l_foot")->getWorldTransform().col(3);
  Eigen::Matrix4d TwrStart = AK->legFK(rleg_angle, false);
  //	TwrStart.col(3) = _atlas->getNode("r_foot")->getWorldTransform().col(3);
  
  Eigen::Matrix4d Tm[atlas::NUM_MANIPULATORS];
  Tm[atlas::MANIP_L_FOOT] = TwlStart;
  Tm[atlas::MANIP_R_FOOT] = TwrStart;
  
  atlas::IK_Mode mode[atlas::NUM_MANIPULATORS];
  mode[atlas::MANIP_L_FOOT] = atlas::IK_MODE_SUPPORT;
  mode[atlas::MANIP_R_FOOT] = atlas::IK_MODE_WORLD;
  mode[atlas::MANIP_L_HAND] = atlas::IK_MODE_FIXED;
  mode[atlas::MANIP_R_HAND] = atlas::IK_MODE_FIXED;
  
  std::cout << "com: " << comStart << std::endl;
  std:: cout << "TwlStart: " << TwlStart << std::endl;
  std::cout << "TwrStart: " << TwrStart << std::endl;
  
  /********************************
   * Generate sequence of joints 
   **********************************/
  mLeftLeg.resize(0);
  mRightLeg.resize(0);

  for( int i = 0; i < mX.size(); ++i ) {

    /**************************************
     * Set mode based on stance
     **************************************/
    switch (mSupportMode[i]) {
    case DOUBLE_LEFT:
      mode[atlas::MANIP_L_FOOT] = atlas::IK_MODE_SUPPORT;
      mode[atlas::MANIP_R_FOOT] = atlas::IK_MODE_SUPPORT;
      break;
    case DOUBLE_RIGHT:
      mode[atlas::MANIP_L_FOOT] = atlas::IK_MODE_SUPPORT;
      mode[atlas::MANIP_R_FOOT] = atlas::IK_MODE_SUPPORT;
      break;
    case SINGLE_LEFT:
      mode[atlas::MANIP_L_FOOT] = atlas::IK_MODE_SUPPORT;
      mode[atlas::MANIP_R_FOOT] = atlas::IK_MODE_WORLD;
      break;
    case SINGLE_RIGHT:
      mode[atlas::MANIP_L_FOOT] = atlas::IK_MODE_WORLD;
      mode[atlas::MANIP_R_FOOT] = atlas::IK_MODE_SUPPORT;
      break;
    }

    /*************************
     * comIK
     ************************/
    /*      
    std::cout << "currentCom: \n" << com.transpose() << endl;
    std::cout << "currentLeft: \n" << Tm[MANIP_L_FOOT] << endl;
    std::cout << "currentRight: \n" << Tm[MANIP_R_FOOT] << endl;
    std::cout << "currentTwb: \n" << Twb << endl;
    */
    Tm[atlas::MANIP_L_FOOT] = TwlStart;
    Tm[atlas::MANIP_L_FOOT](0, 3) += mLeftFoot[i](0); // Left Foot X
    Tm[atlas::MANIP_L_FOOT](2, 3) += mLeftFoot[i](2); // Left Foot Z
    
    Tm[atlas::MANIP_R_FOOT] = TwrStart;
    Tm[atlas::MANIP_R_FOOT](0, 3) += mRightFoot[i](0); // Right Foot X
    Tm[atlas::MANIP_R_FOOT](2, 3) += mRightFoot[i](2); // Right Foot Z
    
    com = comStart;
    com(0) += mX[i](0); // CoM X
    com(1) += mX[i](1); // CoM Y
    /*
    cout << "desiredCom: \n" << com.transpose() << endl;
    cout << "desiredLeft: \n" << Tm[MANIP_L_FOOT] << endl;
    cout << "desiredRight: \n" << Tm[MANIP_R_FOOT] << endl;
    */

    if (AK->comIK( mAtlasSkel, com, Twb, mode, Tm, dofs) != true) {
      std::cout << "comIK failed!" << std::endl;
      std::cout << "Support Mode: " << mSupportMode[i] << std::endl;
      std::cout << "Twb: \n" << Twb << std::endl;
      std::cout << "dcom: " << com.transpose() << std::endl;
      std::cout << "Left foot: \n" << Tm[atlas::MANIP_L_FOOT] << std::endl;
      std::cout << "Right foot: \n" << Tm[atlas::MANIP_R_FOOT] << std::endl;

      exit(1);
    }
    else { 
      //std::cout << "comIK success" << std::endl;
    }

    mDartDofs.push_back(dofs);

    // Store
    dofs = mAtlasSkel->getPose();

    Eigen::Vector6d lleg;
    Eigen::Vector6d rleg;

    for (int i = 0; i < 6; i++) {
      lleg(i) = dofs(l[i]);
      rleg(i) = dofs(r[i]);
    }

    mLeftLeg.push_back( lleg );
    mRightLeg.push_back( rleg );

    Eigen::VectorXd wholePose( mDofIndices.size() );
    for( int i = 0; i < mDofIndices.size(); ++i ) {
      wholePose[i] = dofs( mDofIndices[i] );
    }
    mWholeBody.push_back( wholePose );


  }
	
}

/**
 * @function prepareAtlasKinematics 
 */
atlas::AtlasKinematics* zmpUtilities::prepareAtlasKinematics() {

    DartLoader dart_loader;
   printf("Loading skel \n");
    simulation::World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH "atlas/atlas_world.urdf");
	printf("End loading skel \n");
    mAtlasSkel = mWorld->getSkeleton("atlas");
    mAtlasKin = new atlas::AtlasKinematics();
    mAtlasKin->init( mAtlasSkel );


  mAtlasSkel->setPose( mAtlasSkel->getPose().setZero(), true );
  return mAtlasKin;
}


/**
 * @function print
 */
void zmpUtilities::print( const std::string &_name,
			  const std::vector<Eigen::VectorXd> &_zmp ) {

  FILE* pFile;

  pFile = fopen( _name.c_str(), "w" );

  for( int i = 0; i < _zmp.size(); ++i ) {
      fprintf( pFile, "%d ", i );    
    for( int j = 0; j < _zmp[i].rows(); ++j ) {
      fprintf( pFile, " %f ",  _zmp[i](j) );
    }
      fprintf( pFile, "\n" );
  }

  fclose( pFile );

}

