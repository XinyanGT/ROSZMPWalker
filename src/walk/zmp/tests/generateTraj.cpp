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
  double dt = 0.01;
  double zg = 0.84;
  int numPreviewSteps = 2;
  double Qe = 1;
  double R = 0.000001;

  zp.setParameters( dt, 9.81 );
  zp.generateZmpPositions( 10, true, 
			   stepLength, footSeparation,
			   stepDuration,
			   slopeTime,
			   levelTime );
  zp.print( std::string("zmpxy.txt"), zp.mZMP );
  zp.print( std::string("leftFoot.txt"), zp.mLeftFoot );
  zp.print( std::string("rightFoot.txt"), zp.mRightFoot );

  zp.getControllerGains( Qe, R, zg, numPreviewSteps );

  zp.generateCOMPositions();
  printf("No problem \n");
  zp.print( std::string("com.txt"), zp.mX );
  zp.print( std::string("zmpapprox.txt"), zp.mY );

  zp.getJointTrajectories();
  zp.print( std::string("leftLeg.txt"), zp.mLeftLeg );
  
  
}
