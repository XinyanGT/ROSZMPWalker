/**
 * @file generateTraj.cpp
 */

#include "zmpUtilities.h"
#include <stdio.h>

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  zmpUtilities zp;
  double stepLength = 0.15;
  double footSeparation = 0.282;
  double stepDuration = 1.0;
  double slopeTime = 0.15;
  double levelTime = 0.85;
  double dt = 0.005;
  double zg = 0.8438;
  int numPreviewSteps = 2;
  double Qe = 10000000;
  double R = 10;
  int numWaitSteps = 1;

	Eigen::VectorXd zeroPose = Eigen::VectorXd::Zero(35);
  zp.setParameters( dt, 9.81, zeroPose );
  zp.generateZmpPositions( 8, true, 
			   stepLength, footSeparation,
			   stepDuration,
			   slopeTime,
			   levelTime,
				numWaitSteps );
  printf("Gotten zmp positions \n");
  zp.getControllerGains( Qe, R, zg, numPreviewSteps );
 printf("Gotten controller gains \n");
  zp.generateCOMPositions();
  printf("No problem \n");
  zp.getJointTrajectories();
	printf("Gotten trajectories \n");

  zp.print( std::string("zmpxy.txt"), zp.mZMP );
  zp.print( std::string("com.txt"), zp.mX ); 
  zp.print( std::string("zmpFromDynamics.txt"), zp.mY ); 
  zp.print( std::string("leftFoot.txt"), zp.mLeftFoot ); 
  zp.print( std::string("rightFoot.txt"), zp.mRightFoot );

  zp.print( std::string("leftLeg.txt"), zp.mLeftLeg );
  
  
}
