/**
 * @file zmp_utilities.h
 * @author A. Huaman <ahuaman3@gatech.edu> (adapted from .m files by X. Yan)
 * @date 2013/04/22
 */

#ifndef _ZMP_UTILITIES_H_
#define _ZMP_UTILITIES_H_

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <string>

#include <atlas/AtlasKinematics.h>

// For tinyWalker
#include "tinyWalker/zmp/zmpwalkgenerator.h"

/**
 * @class zmpUtilities
 */
class zmpUtilities {
  
 public:
  
  /** Constructor */
  zmpUtilities();
  /** Destructor */
  ~zmpUtilities();
  
  void setParameters( const double &_dt,
		      const double &_g,
          const Eigen::VectorXd &_initDofs ); 

  /** Init walk generator */
  void initZMPWalkGenerator( const double &_CoMZ = 0.8438,
					   const double &_singleSupportTime = 0.85,
					   const double &_doubleSupportTime = 0.1,
					   const double &_startupTime = 1.0,
					   const double &_shutdowntime = 1.0,
					   const double &_previewTime = 2.0,
					   const double &_footLiftoffZ = 0.05,
					   const double &_zmpOffX = 0.0,
					   const double &_zmpOffY = 0.0,
					   const double &_zmpJerkPenalty = 1e-8,
					   ZMPWalkGenerator::ik_error_sensitivity _ikSense = ZMPWalkGenerator::ik_strict,
					   const double &_comIkAscl = 0.0 );

  void generateZmpPositions( double walk_dist, 
					   double foot_separation_y,
					   double step_length );
  
  /** Generate a nice step function with spline blendings between transitions */
  std::vector<double> generateSmoothPattern( const double &_x0,
					     const double &_xf,
					     const int &_numTransitionPts,
					     const int &_numConstantPts );

  /** Controller gains */
  void getControllerGains( const double &_Qe,
			   const double &_R,
			   const double &_z_COM,
			   const double &_previewTime = 2.0 );
  
  /** Solve Discrete Algebraic Riccati equation */
  bool iterative_DARE( Eigen::MatrixXd &_P, 
		       const Eigen::MatrixXd &_A,
		       const Eigen::MatrixXd &_B,
		       const Eigen::MatrixXd &_Q,
		       const Eigen::MatrixXd &_R,
		       const double &_error_threshold = 0.000001,
		       const int &_numIter = 100000 );

  /** generate CoM based on ZMP positions */
  void generateCOMPositions();
  
  void generateSwingPattern( std::vector<Eigen::VectorXd> &_footPos,
			     const double &_x0, const double &_xf,
			     const double &_y0, const double &_yf,
			     const int &_numPts );
   
  /** Prepare Atlas (what does this mean? */
  atlas::AtlasKinematics* prepareAtlasKinematics(); 

  void getJointTrajectories();

  /** Print */
  void print( const std::string &_name,
	      const std::vector<Eigen::VectorXd> &_zmp );
  
  /*******************************
   * Needed ?
   *********************************/
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // General
  int mN; /**< Num preview steps */
  double mdt;
  double mzCOM;
  double mG;

  double mStepLength;
  double mFootSeparation;
  double mStepDuration;


  // Dynamic Equations
  Eigen::MatrixXd mA, mB, mC;

  // Controller gains
  Eigen::MatrixXd mG1;
  Eigen::MatrixXd mG2;

  // Preview gain calculation
  std::vector<Eigen::MatrixXd> mG3;

  // ZMP Steps
  std::vector<Eigen::VectorXd> mZMP;
  std::vector<Eigen::VectorXd> mLeftFoot;
  std::vector<Eigen::VectorXd> mRightFoot;

  std::vector<int> mDofIndices;
  std::vector<Eigen::VectorXd> mWholeBody;
  std::vector<int> mSupportMode;

  std::vector<Eigen::VectorXd> mDartDofs; ///< joint trajs in Dart ordering

  // Stored x, y and u
  std::vector<Eigen::VectorXd> mX; /**< CoM (x,y)T */
  std::vector<Eigen::VectorXd> mY; /**< ZMP (from model) */
  std::vector<Eigen::VectorXd> mU;

  // Atlas info
  atlas::AtlasKinematics *mAtlasKin;
  kinematics::Skeleton *mAtlasSkel;

  std::vector<Eigen::VectorXd> mLeftLeg;
  std::vector<Eigen::VectorXd> mRightLeg;
  Eigen::VectorXd mInitDofVals;

  // ZMP Walk Generator
  ZMPWalkGenerator* mWalker;
  
};

#endif /** _UTILITIES_H_ */
  
