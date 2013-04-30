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
					 const int &_numWaitSteps ) {
  std::vector<double> temp;
  double start, end;
  int numSlopePts, numLevelPts;
  int numTotalPts;
  double leftFoot, rightFoot;
  double supportFoot, swingFoot, tempFoot;

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
    	std::fill( support.begin(), support.end(), DOUBLE_SUPPORT );

    	// Stay put left
    	p << 0, leftFoot, 0;
    	std::fill( lf.begin(), lf.end(), p );
    	std::fill( support.begin(), support.end(), DOUBLE_SUPPORT );
  
  		mRightFoot.insert( mRightFoot.end(), rf.begin(), rf.end() );
 	 	mLeftFoot.insert( mLeftFoot.end(), lf.begin(), lf.end() );
  		mSupportMode.insert( mSupportMode.end(), support.begin(), support.end() );
  	}

  //-- Start
  if( _startLeftFoot == true ) {
    supportFoot = rightFoot;
    swingFoot = leftFoot;
  }
  else {
    supportFoot = leftFoot;
    swingFoot = rightFoot;
  }


  //-- First step
  //** Generate ZMP x **
  start = 0; end = (1 - 1)*mStepLength;
  temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
  zmpx.insert( zmpx.end(), temp.begin(), temp.end() );

  //** Generate ZMP y **
  start = 0; end = supportFoot;
  temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
  zmpy.insert( zmpy.end(), temp.begin(), temp.end() );

  //** Generate foot placements **
  if( supportFoot == leftFoot ) {
    // Swing right
    generateSwingPattern( rf, 
			  0, 1*mStepLength,
			  rightFoot, rightFoot,
			  numTotalPts );
    // Stay put left
    Eigen::Vector3d p; p << (1-1)*mStepLength, leftFoot, 0;
    std::fill( lf.begin(), lf.end(), p );
    std::fill( support.begin(), support.end(), LEFT_SUPPORT );
    std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_SUPPORT );
  }
  
  else { 
    // Swing left
    generateSwingPattern( lf, 
			  0, 1*mStepLength,
			  leftFoot, leftFoot,
			  numTotalPts );
    // Stay put right
    Eigen::Vector3d p; p << (1-1)*mStepLength, rightFoot, 0;
    std::fill( rf.begin(), rf.end(), p );
    std::fill( support.begin(), support.end(), RIGHT_SUPPORT );
    std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_SUPPORT );
  }
  
  mRightFoot.insert( mRightFoot.end(), rf.begin(), rf.end() );
  mLeftFoot.insert( mLeftFoot.end(), lf.begin(), lf.end() );
  mSupportMode.insert( mSupportMode.end(), support.begin(), support.end() );
  
  // Switch feet
  tempFoot = supportFoot;
  supportFoot = swingFoot;
  swingFoot = tempFoot;

  //-- From step 2 to (N -1)
  for( unsigned int i = 2; i <= _numSteps - 1; ++i ) {

    //** Generate ZMP x **
    start = (i-2)*mStepLength; end = (i-1)*mStepLength;
    temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
    zmpx.insert( zmpx.end(), temp.begin(), temp.end() );

    //** Generate ZMP y **
    start = swingFoot; end = supportFoot;
    temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
    zmpy.insert( zmpy.end(), temp.begin(), temp.end() );

    //** Generate foot placements **
    if( supportFoot == leftFoot ) {
      // Swing right
      generateSwingPattern( rf, 
			    (i-2)*mStepLength, i*mStepLength,
			    rightFoot, rightFoot,
			    numTotalPts );
      // Stay put left
      Eigen::Vector3d p; p << (i-1)*mStepLength, leftFoot, 0;
      std::fill( lf.begin(), lf.end(), p );
      std::fill( support.begin(), support.end(), LEFT_SUPPORT );
    std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_SUPPORT );
    }

    else { 
      // Swing left
      generateSwingPattern( lf, 
			    (i-2)*mStepLength, i*mStepLength,
			    leftFoot, leftFoot,
			    numTotalPts );
      // Stay put right
      Eigen::Vector3d p; p << (i-1)*mStepLength, rightFoot, 0;
      std::fill( rf.begin(), rf.end(), p );
      std::fill( support.begin(), support.end(), RIGHT_SUPPORT );
    std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_SUPPORT );
    }

    mRightFoot.insert( mRightFoot.end(), rf.begin(), rf.end() );
    mLeftFoot.insert( mLeftFoot.end(), lf.begin(), lf.end() );
    mSupportMode.insert( mSupportMode.end(), support.begin(), support.end() );

    // Switch feet
    tempFoot = supportFoot;
    supportFoot = swingFoot;
    swingFoot = tempFoot;
  } 

  //-- Last step
  // ** Generate ZMP x **
  start = (_numSteps - 2)*mStepLength; end = (_numSteps - 1)*mStepLength;
  temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
  zmpx.insert( zmpx.end(), temp.begin(), temp.end() );

  // ** Generate ZMP y **
  start = swingFoot; end = 0;
  temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
  zmpy.insert( zmpy.end(), temp.begin(), temp.end() );

  //** Generate foot placements **
  if( supportFoot == leftFoot ) {
    // Swing right
    generateSwingPattern( rf, 
			  (_numSteps-2)*mStepLength, (_numSteps-1)*mStepLength,
			  rightFoot, rightFoot,
			  numTotalPts );
    // Stay put left
    Eigen::Vector3d p; p << (_numSteps-1)*mStepLength, leftFoot, 0;
    std::fill( lf.begin(), lf.end(), p );
    std::fill( support.begin(), support.end(), LEFT_SUPPORT );
    std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_SUPPORT );
  }
  
  else { 
    // Swing left
    generateSwingPattern( lf, 
			  (_numSteps-2)*mStepLength, (_numSteps-1)*mStepLength,
			  leftFoot, leftFoot,
			  numTotalPts );
    // Stay put right
    Eigen::Vector3d p; p << (_numSteps-1)*mStepLength, rightFoot, 0;
    std::fill( rf.begin(), rf.end(), p );
    std::fill( support.begin(), support.end(), RIGHT_SUPPORT );
    std::fill( support.begin(), support.begin() + numSlopePts, DOUBLE_SUPPORT );
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
