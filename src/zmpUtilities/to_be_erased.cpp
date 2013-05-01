#include "zmpUtilities.h"

/**
 * @function generateZmpPositions
 */
void zmpUtilities::generateZmpPositions( int _numSteps,
					 const bool &_startLeftFoot,
					 const double &_stepLength,
					 const double &_footSeparation,
					 const double &_stepDuration,
					 const double &_slopeTime,
					 const double &_levelTime,
					 const int &_numWaitSteps,
					 const double &_stepHeight,
					 const double &_zmpDeltaY ) {
  std::vector<double> temp;
  double start, end;
  int numSlopePts, numLevelPts;
  int numTotalPts;
  double leftFoot, rightFoot;
  double supportFoot, swingFoot, tempFoot;
  double leftFootZMP, rightFootZMP;
  double supportFootZMP, swingFootZMP, tempFootZMP;

  //-- Prepare the vectors
  std::vector<double> zmpx, zmpy;

  //-- Set a few variables
  mStepLength = _stepLength;
  mFootSeparation = _footSeparation;
  mStepDuration = _stepDuration;
  
  numSlopePts = _slopeTime / mdt;
  numLevelPts = _levelTime / mdt;
  numTotalPts = numSlopePts + numLevelPts;

  std::vector<Eigen::VectorXd> lf( numTotalPts, Eigen::Vector3d() );
  std::vector<Eigen::VectorXd> rf( numTotalPts, Eigen::Vector3d() );
  std::vector<int> support( numTotalPts );

  leftFoot = mFootSeparation / 2.0;
  rightFoot = -1*mFootSeparation / 2.0;

  leftFootZMP = leftFoot - _zmpDeltaY;
  rightFootZMP = rightFoot + _zmpDeltaY;
  
  //-- WAIT TIME START
  for( int i = 0; i < _numWaitSteps; ++i ) {
    //** Generate ZMP x **	
    start = 0; end = 0;
    temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
    zmpx.insert( zmpx.end(), temp.begin(), temp.end() );
    
    //** Generate ZMP y **
    start = 0; end = 0;
    temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
    zmpy.insert( zmpy.end(), temp.begin(), temp.end() );
    
    //** Generate foot placements **
    
    // Stay put right
    Eigen::Vector3d p; p << 0, rightFoot, 0;
    std::fill( rf.begin(), rf.end(), p );
    
    // Stay put left
    p << 0, leftFoot, 0;
    std::fill( lf.begin(), lf.end(), p );

    // Store support
    std::fill( support.begin(), support.end(), DOUBLE_LEFT ); // DOUBLE_RIGHT
    
    mRightFoot.insert( mRightFoot.end(), rf.begin(), rf.end() );
    mLeftFoot.insert( mLeftFoot.end(), lf.begin(), lf.end() );
    mSupportMode.insert( mSupportMode.end(), support.begin(), support.end() );
  }
  
  //-- Start
  if( _startLeftFoot == true ) {
    supportFoot = rightFoot;
    swingFoot = leftFoot;
    supportFootZMP = rightFootZMP;
    swingFootZMP = leftFootZMP;
  }
  else {
    supportFoot = leftFoot;
    swingFoot = rightFoot;
    supportFootZMP = leftFootZMP;
    swingFootZMP = rightFootZMP;
  }


  //-- First step
  //** Generate ZMP x **
  start = 0; end = (1 - 1)*mStepLength;
  temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
  zmpx.insert( zmpx.end(), temp.begin(), temp.end() );

  //** Generate ZMP y **
  start = 0; end = supportFootZMP;
  temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
  zmpy.insert( zmpy.end(), temp.begin(), temp.end() );

  //** Generate foot placements **
  if( supportFoot == leftFoot ) {
    // Swing right
    generateSwingPattern( rf, 
			  0, 1*mStepLength,
			  rightFoot, rightFoot,
			  numTotalPts,
			_stepHeight );
    // Stay put left
    Eigen::Vector3d p; p << (1-1)*mStepLength, leftFoot, 0;
    std::fill( lf.begin(), lf.end(), p );
    std::fill( support.begin(), support.end(), SINGLE_LEFT );
    std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_LEFT );
  }
  
  else { 
    // Swing left
    generateSwingPattern( lf, 
			  0, 1*mStepLength,
			  leftFoot, leftFoot,
			  numTotalPts,
			_stepHeight );
    // Stay put right
    Eigen::Vector3d p; p << (1-1)*mStepLength, rightFoot, 0;
    std::fill( rf.begin(), rf.end(), p );
    std::fill( support.begin(), support.end(), SINGLE_RIGHT );
    std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_RIGHT );
  }
  
  mRightFoot.insert( mRightFoot.end(), rf.begin(), rf.end() );
  mLeftFoot.insert( mLeftFoot.end(), lf.begin(), lf.end() );
  mSupportMode.insert( mSupportMode.end(), support.begin(), support.end() );
  
  // Switch feet
  tempFoot = supportFoot;
  supportFoot = swingFoot;
  swingFoot = tempFoot;

  tempFootZMP = supportFootZMP;
  supportFootZMP = swingFootZMP;
  swingFootZMP = tempFootZMP;


  //-- From step 2 to (N -1)
  for( unsigned int i = 2; i <= _numSteps - 1; ++i ) {
    
    //** Generate ZMP x **
    start = (i-2)*mStepLength; end = (i-1)*mStepLength;
    temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
    zmpx.insert( zmpx.end(), temp.begin(), temp.end() );

    //** Generate ZMP y **
    start = swingFootZMP; end = supportFootZMP;
    temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
    zmpy.insert( zmpy.end(), temp.begin(), temp.end() );

    //** Generate foot placements **
    if( supportFoot == leftFoot ) {
      // Swing right
      generateSwingPattern( rf, 
			    (i-2)*mStepLength, i*mStepLength,
			    rightFoot, rightFoot,
			    numTotalPts,
			    _stepHeight );
      // Stay put left
      Eigen::Vector3d p; p << (i-1)*mStepLength, leftFoot, 0;
      std::fill( lf.begin(), lf.end(), p );
      std::fill( support.begin(), support.end(), SINGLE_LEFT );
      std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_LEFT );
    }

    else { 
      // Swing left
      generateSwingPattern( lf, 
			    (i-2)*mStepLength, i*mStepLength,
			    leftFoot, leftFoot,
			    numTotalPts,
			    _stepHeight );
      // Stay put right
      Eigen::Vector3d p; p << (i-1)*mStepLength, rightFoot, 0;
      std::fill( rf.begin(), rf.end(), p );
      std::fill( support.begin(), support.end(), SINGLE_RIGHT );
      std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_RIGHT );
    }
    
    mRightFoot.insert( mRightFoot.end(), rf.begin(), rf.end() );
    mLeftFoot.insert( mLeftFoot.end(), lf.begin(), lf.end() );
    mSupportMode.insert( mSupportMode.end(), support.begin(), support.end() );
    
    // Switch feet
    tempFoot = supportFoot;
    supportFoot = swingFoot;
    swingFoot = tempFoot;

    tempFootZMP = supportFootZMP;
    supportFootZMP = swingFootZMP;
    swingFootZMP = tempFootZMP;
  } 

  //-- Last step
  // ** Generate ZMP x **
  start = (_numSteps - 2)*mStepLength; end = (_numSteps - 1)*mStepLength;
  temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
  zmpx.insert( zmpx.end(), temp.begin(), temp.end() );

  // ** Generate ZMP y **
  start = swingFootZMP; end = 0;
  temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
  zmpy.insert( zmpy.end(), temp.begin(), temp.end() );

  //** Generate foot placements **
  if( supportFoot == leftFoot ) {
    // Swing right
    generateSwingPattern( rf, 
			  (_numSteps-2)*mStepLength, (_numSteps-1)*mStepLength,
			  rightFoot, rightFoot,
			  numTotalPts,
			_stepHeight );
    // Stay put left
    Eigen::Vector3d p; p << (_numSteps-1)*mStepLength, leftFoot, 0;
    std::fill( lf.begin(), lf.end(), p );
    std::fill( support.begin(), support.end(), SINGLE_LEFT );
    std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_LEFT );
  }
  
  else { 
    // Swing left
    generateSwingPattern( lf, 
			  (_numSteps-2)*mStepLength, (_numSteps-1)*mStepLength,
			  leftFoot, leftFoot,
			  numTotalPts,
			_stepHeight );
    // Stay put right
    Eigen::Vector3d p; p << (_numSteps-1)*mStepLength, rightFoot, 0;
    std::fill( rf.begin(), rf.end(), p );
    std::fill( support.begin(), support.end(), SINGLE_RIGHT );
    std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_RIGHT );
  }
  
  mRightFoot.insert( mRightFoot.end(), rf.begin(), rf.end() );
  mLeftFoot.insert( mLeftFoot.end(), lf.begin(), lf.end() );
  mSupportMode.insert( mSupportMode.end(), support.begin(), support.end() );

  // No need to switch feet
  
  
  //-- Store
  mZMP.resize(0);
  Eigen::MatrixXd zmp(2,1);
  for( int i = 0; i < zmpy.size(); ++i ) {
    zmp(0,0) = zmpx[i];
    zmp(1,0) = zmpy[i];
    mZMP.push_back( zmp );
  }

}


/**
 * @function generateSmoothPattern
 */
std::vector<double> zmpUtilities::generateSmoothPattern( const double &_x0,
							 const double &_xf,
							 const int &_numTransitionPts,
							 const int &_numConstantPts ) {
  
  int numTotalPts = _numTransitionPts + _numConstantPts;
  std::vector<double> pts;

  // Set all the points to constant value (the transition pts come below)
  pts.assign( numTotalPts, _xf );


  // Generate transition points
  double t; double p;
  for( int i = 0; i < _numTransitionPts; ++i ) {
    t = (double) i / ( (double) (_numTransitionPts - 1) );
    p = ( 1 - t )*_x0 + t*_xf + t*(1-t)*( (_x0-_xf)*(1-t) + (_xf-_x0)*t ); 
    pts[i] = p;
  }

  return pts;
}


/**
 * @functio generateSwingPattern
 */
void zmpUtilities::generateSwingPattern( std::vector<Eigen::VectorXd> &_footPos,
					 const double &_x0, const double &_xf,
					 const double &_y0, const double &_yf,
					 const int &_numPts,
					 const double &_stepHeight ) {
  // Reset container
  _footPos.resize(0);

  double dx, dy;
  double t0, tf;
  double dt, t;
  double alpha, cos_a, sin_a;
  double l;
  double a, b;

  dx = _xf - _x0;
  dy = _yf - _y0;
  
  //t0 = 0;
  //tf = 3.1416;
  t0 = 3.1416;
  tf = 0;

  dt = ( tf - t0 ) / ( _numPts - 1 );
  
  t = t0;
  alpha = atan2( dy, dx );
  cos_a = cos( alpha );
  sin_a = sin( alpha );
  l = 0;

  a = 0.5*sqrt( dx*dx + dy*dy );
//  b = a / 3.1416;
  if( _stepHeight == 0 ) { b = a / 2; }
  else { b = _stepHeight; }

  Eigen::Vector3d pos;
  for( int i = 0; i < _numPts; ++i ) {
    pos(0) = _x0 + l*cos_a;
    pos(1) = _y0 + l*sin_a;
//    pos(2) = b*sin( (3.1416/2.0)*(cos(t) + 1) );
    pos(2) = b*sin( t );

    t += dt;
//    l = a*cos( (3.1416/2.0)*(cos(t) + 1) ) + a;
	l = a*cos(t) + a;

    _footPos.push_back( pos );
  }

	// Just to make sure we end fine
	_footPos[_numPts-1](0) = _xf;
	_footPos[_numPts-1](1) = _yf;
	_footPos[_numPts-1] (2)= 0;
  
}

