/**
 * @file johnnieWalker.cpp
 * @brief Walker code
 */

#include "johnnieWalker.h"


// Global variables
atlas::AtlasKinematics *gAK;
kinematics::Skeleton* gAtlasSkel;
zmpUtilities gZU;

ros::Publisher gPubAtlasCommand;
ros::Publisher gPubMode;
ros::Subscriber gSubJointStates;
atlas_msgs::AtlasCommand gAtlasCommand;
atlas_msgs::AtlasCommand gAtlasCommand_saved;
Eigen::VectorXd gJointAngles;
ros::Rate* gLoopRate;
ros::NodeHandle* gRosNode;
bool gJohnnieDebug;
std::vector<std::string> gJointNames;
double gFrequency;

// Constant: ORDER OF JOINTS
std::vector<int> gDartTorsoHeadJointInd;
std::vector<int> gDartLeftLegJointInd;
std::vector<int> gDartRightLegJointInd;
std::vector<int> gDartLeftArmJointInd;
std::vector<int> gDartRightArmJointInd;
std::vector<int> gDartJointInd;



/**
 * @function main
 */
int main( int argc, char** argv ) {

  // *****************************
  //  ROS INITIAL CONFIGURATION
  // *****************************

  initVariables();
  gJohnnieDebug = false;
  gFrequency = 200;
  ros::init( argc, argv, "johnnieWalker" );
  gRosNode = new ros::NodeHandle();
  gLoopRate = new ros::Rate( gFrequency );

  // Wait until simulation is ON
  bool wait = true; ros::Time lastRosTime;

  while( wait ) {
    lastRosTime = ros::Time::now();
    if( lastRosTime.toSec() > 0 ) { wait = false; }
  }

  // *****************************
  //  INIT ROS JOINT INFO
  // *****************************
  RosJointInit();

  // *************************************
  //  SET SUBSCRIPTION AND PUBLISH STUFF
  // *************************************
  setSubscriptionPublishing();

  // *****************************
  //  KINEMATICS SETTINGS
  // *****************************
  if( !setKinematics() ) { return 1; }


  // *****************************************
  // SET ATLAS TO START POSITION (bend knees)
  // *****************************************
  Eigen::VectorXd dofs( gAtlasSkel->getNumDofs() );
  UpdateDofs( dofs );

  if( gJohnnieDebug ) { std::cout << "Joint pose starting: \n"<<dofs.transpose() << std::endl; }
  gAtlasSkel->setPose( dofs, true );
  
  // RELAX ATLAS AND TELL SKEL THE LATEST POSITION AFTER RELAXING
  RelaxAtlas( gAK, gAtlasSkel, dofs, 10, 1.2, 1000 );
  UpdateDofs( dofs );
  gAtlasSkel->setPose( dofs, true );

 
  // ***********************************************
  // Move COM down 
  // ***********************************************

  Eigen::Vector3d comDelta = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftDelta = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightDelta = Eigen::Vector3d::Zero();

  Eigen::Matrix4d Twm[atlas::NUM_MANIPULATORS];
  Twm[atlas::MANIP_L_FOOT] = gAK->getLimbTransB( gAtlasSkel, atlas::MANIP_L_FOOT );
  Twm[atlas::MANIP_R_FOOT] = gAK->getLimbTransB( gAtlasSkel, atlas::MANIP_R_FOOT );

  Eigen::Matrix4d Twb;
  Twb.setIdentity();

  Eigen::VectorXd dofs_save;

  /*************************
   * Move COM down
   *************************/
  comDelta << 0, 0, -0.05;
  leftDelta.setZero();
  rightDelta.setZero();

  //MoveCOMIK( gAK, gAtlasSkel, Twb, Twm, dofs, comDelta, leftDelta, rightDelta, 10, 10, 10, 1.2, 1.2, 1.2, 1000);


  /************************************
   * ZMP walking
   ************************************/

  // Set parameters for ZMP walker
  int numSteps = 10; //30;
  double stepLength = 0.15; // half step
 
  // Update states to get foot separation and CoM height (z)  
  dofs_save = gAtlasSkel->getPose();
  UpdateDofs(dofs);

  std::cout << "********************************************" << std::endl;
  std::cout << "Start ZMP walking" << std::endl;
  std::cout << "*************************************" << std::endl;
  std::cout << "POS ERROR: " << (dofs_save - dofs).norm() << std::endl;
  std::cout << "*************************************" << std::endl;

  gAtlasSkel->setPose(dofs);

  double footSeparation = ( gAK->getLimbTransW( gAtlasSkel, Twb, atlas::MANIP_L_FOOT)(1,3) - 
			    gAK->getLimbTransW( gAtlasSkel, Twb, atlas::MANIP_R_FOOT)(1,3) );
  std::cout << "Foot separation: " << footSeparation << std::endl;
  double stepDuration = 5; //3;
  double slopeTime = 1; // move ZMP time
  double levelTime = 4; // keep ZMP time
  double dt = 1/gFrequency; /**< command sending period */
  // height of COM
  double zg = gAK->getCOMW( gAtlasSkel, Twb)(2) - gAK->getLimbTransW( gAtlasSkel, Twb, atlas::MANIP_L_FOOT)(2,3);
  std::cout << "zg " << zg << std::endl;
  int numPreviewSteps = 2;


  double Qe = 1e7; // 1
  double R = 10;  // 1e-6
  double default_kp = 20;
  double strong_kp = 20;
  double weak_kp = 20;

  double default_ki = 1.2; // 1.3;
  double strong_ki = 1.3;
  double weak_ki = 1.1; //1.3;

  Eigen::VectorXd double_support_kp, double_support_ki;
  Eigen::VectorXd left_support_kp, left_support_ki;
  Eigen::VectorXd right_support_kp, right_support_ki;

  double_support_kp = Eigen::VectorXd::Constant(28, default_kp);
  double_support_ki = Eigen::VectorXd::Constant(28, default_ki);
  
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
  
  if( gJohnnieDebug ) {
    std::cout << "double kp: " << double_support_kp.transpose() << std::endl;
    std::cout << "double ki: " << double_support_ki.transpose() << std::endl;
    std::cout << "left kp: " << left_support_kp.transpose() << std::endl;
    std::cout << "left ki: " << left_support_ki.transpose() << std::endl;
    std::cout << "right kp: " << right_support_kp.transpose() << std::endl;
    std::cout << "right ki: " << right_support_ki.transpose() << std::endl;
  }

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
  //  gZU.print( "jointsWholeBody.txt", gZU.mWholeBody );
  printf("Whole body traj size: %d", gZU.mWholeBody.size() );


  MoveJointTractoryAdv( gAK, gAtlasSkel, dofs, 
			gZU.mWholeBody, gZU.mSupportMode, 
                        double_support_kp,double_support_ki,
                        left_support_kp, left_support_ki,
                        right_support_kp, right_support_ki);

  // *********
  // THE END
  // *********
  printf("End test run \n");

  return 0;
}


/**
 * @function initVariables 
 */
void initVariables() {

  // Init torso/head joint indices
  gDartTorsoHeadJointInd.resize(0);
  gDartTorsoHeadJointInd.push_back( 6 );
  gDartTorsoHeadJointInd.push_back( 9 );
  gDartTorsoHeadJointInd.push_back( 12 );
  gDartTorsoHeadJointInd.push_back( 16 );

  // Init left leg joint indices
  gDartLeftLegJointInd.resize(0);
  gDartLeftLegJointInd.push_back(7);
  gDartLeftLegJointInd.push_back(10);
  gDartLeftLegJointInd.push_back(13);
  gDartLeftLegJointInd.push_back(18);
  gDartLeftLegJointInd.push_back(23);
  gDartLeftLegJointInd.push_back(27);

  // Init right leg joint indices
  gDartRightLegJointInd.resize(0);
  gDartRightLegJointInd.push_back(8);
  gDartRightLegJointInd.push_back(11);
  gDartRightLegJointInd.push_back(14);
  gDartRightLegJointInd.push_back(19);
  gDartRightLegJointInd.push_back(24);
  gDartRightLegJointInd.push_back(28);

  // Init left arm joint indices
  gDartLeftArmJointInd.resize(0);
  gDartLeftArmJointInd.push_back(15);
  gDartLeftArmJointInd.push_back(20);
  gDartLeftArmJointInd.push_back(25);
  gDartLeftArmJointInd.push_back(29);
  gDartLeftArmJointInd.push_back(31);
  gDartLeftArmJointInd.push_back(33);

  // Init right arm joint indices
  gDartRightArmJointInd.resize(0);
  gDartRightArmJointInd.push_back(17);
  gDartRightArmJointInd.push_back(22);
  gDartRightArmJointInd.push_back(26);
  gDartRightArmJointInd.push_back(30);
  gDartRightArmJointInd.push_back(32);
  gDartRightArmJointInd.push_back(34);

  // All joints
  gDartJointInd.resize(0);
  gDartJointInd.insert( gDartJointInd.end(), gDartTorsoHeadJointInd.begin(), gDartTorsoHeadJointInd.end() );
  gDartJointInd.insert( gDartJointInd.end(), gDartLeftLegJointInd.begin(), gDartLeftLegJointInd.end() );
  gDartJointInd.insert( gDartJointInd.end(), gDartRightLegJointInd.begin(), gDartRightLegJointInd.end() );
  gDartJointInd.insert( gDartJointInd.end(), gDartLeftArmJointInd.begin(), gDartLeftArmJointInd.end() );
  gDartJointInd.insert( gDartJointInd.end(), gDartRightArmJointInd.begin(), gDartRightArmJointInd.end() );
}


/**
 * @function RosJonitInit
 */
void RosJointInit() {


  // Set name of joints to be commanded 
  // (must match these inside AtlasPlugin)
  gJointNames.resize(0);

  gJointNames.push_back("atlas::back_lbz");
  gJointNames.push_back("atlas::back_mby");
  gJointNames.push_back("atlas::back_ubx");

  gJointNames.push_back("atlas::neck_ay");

  gJointNames.push_back("atlas::l_leg_uhz");
  gJointNames.push_back("atlas::l_leg_mhx");
  gJointNames.push_back("atlas::l_leg_lhy");
  gJointNames.push_back("atlas::l_leg_kny");
  gJointNames.push_back("atlas::l_leg_uay");
  gJointNames.push_back("atlas::l_leg_lax");


  gJointNames.push_back("atlas::r_leg_uhz");
  gJointNames.push_back("atlas::r_leg_mhx");
  gJointNames.push_back("atlas::r_leg_lhy");
  gJointNames.push_back("atlas::r_leg_kny");
  gJointNames.push_back("atlas::r_leg_uay");
  gJointNames.push_back("atlas::r_leg_lax");

  gJointNames.push_back("atlas::l_arm_usy"); 
  gJointNames.push_back("atlas::l_arm_shx");
  gJointNames.push_back("atlas::l_arm_ely");
  gJointNames.push_back("atlas::l_arm_elx");
  gJointNames.push_back("atlas::l_arm_uwy");
  gJointNames.push_back("atlas::l_arm_mwx");

  gJointNames.push_back("atlas::r_arm_usy");
  gJointNames.push_back("atlas::r_arm_shx");
  gJointNames.push_back("atlas::r_arm_ely");
  gJointNames.push_back("atlas::r_arm_elx");
  gJointNames.push_back("atlas::r_arm_uwy");
  gJointNames.push_back("atlas::r_arm_mwx"); 
  

  unsigned int n = gJointNames.size();
  gAtlasCommand.position.resize(n); 
  gAtlasCommand.velocity.resize(n);
  gAtlasCommand.effort.resize(n);
  gAtlasCommand.kp_position.resize(n);
  gAtlasCommand.ki_position.resize(n);
  gAtlasCommand.kd_position.resize(n);
  gAtlasCommand.kp_velocity.resize(n);
  gAtlasCommand.i_effort_min.resize(n);
  gAtlasCommand.i_effort_max.resize(n);
  gAtlasCommand.k_effort.resize(n, 255); // VERY IMPORTANT - TO USE CONTROL PID (IF NOT, NO MOVE! DRCSIM 2.5)
  
  for( unsigned int i = 0; i < n; ++i ) {
    std::vector<std::string> pieces;
    boost::split( pieces, gJointNames[i], boost::is_any_of(":") );
    std::string a( pieces[2] ); double val;
    gRosNode->getParam("atlas_controller/gains/" + pieces[2] + "/p", val ); gAtlasCommand.kp_position[i] = val;
    gRosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i", val ); gAtlasCommand.ki_position[i] = val;
    gRosNode->getParam("atlas_controller/gains/" + pieces[2] + "/d", val ); gAtlasCommand.kd_position[i] = val;

    gRosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", val ); gAtlasCommand.i_effort_min[i] = val;
    gAtlasCommand.i_effort_min[i] = -gAtlasCommand.i_effort_min[i];
    
    gRosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", val ); gAtlasCommand.i_effort_max[i] = val;
    
    printf(" [%d] Kp: %f, Ki: %f Kd: %f, I_clamp min: %f  clamp max: %f\n", i, 
	   gAtlasCommand.kp_position[i], gAtlasCommand.ki_position[i], 
	   gAtlasCommand.kd_position[i], gAtlasCommand.i_effort_min[i], 
	   gAtlasCommand.i_effort_max[i]);

    gAtlasCommand.velocity[i] = 0;
    gAtlasCommand.effort[i] = 0;
    gAtlasCommand.kp_velocity[i] = 0;
  }

  gAtlasCommand_saved = gAtlasCommand;
  gJointAngles.resize( gJointNames.size() );  
}


/**
 * @function setSubscriptionPublishing
 */
void setSubscriptionPublishing() {

  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>("/atlas/joint_states", 1, GetJointStates,
							   ros::VoidPtr(), gRosNode->getCallbackQueue());
  // Subscribe to jointStates
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();
  gSubJointStates = gRosNode->subscribe(jointStatesSo);

  // Advertise atlasCommand
  gPubAtlasCommand =
    gRosNode->advertise<atlas_msgs::AtlasCommand>(
    "/atlas/atlas_command", 1, true);

  // Advertise in atlas/mode
  gPubMode = gRosNode->advertise<std_msgs::String>( "atlas/mode", 100, true );
}


/**
 * @function GetJointState
 * @brief Callback to jointState msg
 */
void GetJointStates( const sensor_msgs::JointState::ConstPtr &_js ) {

  for( int i = 0; i < _js->name.size(); ++i ) {
    gJointAngles[i] = _js->position[i];
  }

}


/**
 * @function setKinematics
 * @brief Load Dart Skeleton, gives false if something goes wrong
 */
bool setKinematics() {

  {
    DartLoader dart_loader;
    robotics::World *world = dart_loader.parseWorld( ATLAS_DATA_PATH "atlas/atlas_world.urdf" );
    gAtlasSkel = world->getSkeleton( "atlas" );
    gAK = new atlas::AtlasKinematics();
    gAK->init( gAtlasSkel );
  }
  
  gAtlasSkel->setPose( gAtlasSkel->getPose().setZero(), true );
  
  return true;
}

/**
 * @function mapJointState2DartPose
 * @brief First call always AFTER setKinematics!!!!
 */
void mapJointState2DartPose( Eigen::VectorXd &_dartPose,
			     const Eigen::VectorXd  &_jointAngles  ) {

  _dartPose.setZero();
  for( int i = 0; i < _jointAngles.size(); ++i ) {
    _dartPose( gDartJointInd[i] ) = _jointAngles(i);
  }

}

/**
 * @function RelaxAtlas
 * @brief Relax Atlas to a better pos for walking. Start from _dofs to a new _dofs with different leg angles
 */
void RelaxAtlas( atlas::AtlasKinematics *_AK, 
		 kinematics::Skeleton *_atlasSkel, 
		 Eigen::VectorXd &_dofs, 
		 double _kp, 
		 double _kd, 
		 int _N ) {


  // use legIK, move legs
  Eigen::Matrix4d twb = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d leftTarget = _AK->getLimbTransB( _atlasSkel, atlas::MANIP_L_FOOT );
  leftTarget(2,3) += 0.03;
  leftTarget(1,3) += 0.03;
//  leftTarget(1,3) += 0.1;

  Eigen::Matrix4d rightTarget = _AK->getLimbTransB(_atlasSkel, atlas::MANIP_R_FOOT );
  rightTarget(2,3) += 0.03;
  rightTarget(1,3) -= 0.03;
  //  rightTarget(1,3) -= 0.1;
  
  Eigen::VectorXd nearest(12);
  nearest.setZero();
  Eigen::VectorXd legAngles(12);

  _AK->stanceIK( twb, leftTarget, rightTarget, nearest, legAngles );
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 6; j++) {
      _dofs( _AK->dof_ind[i][j] ) = legAngles(i*6+j);
    }
  }


  // Set robot in position with more bended knees w.r.t. initial position
  MoveDesireDofs( _AK, _atlasSkel, _dofs, _kp, _kp, _kp, _kd, _kd, _kd, _N );

}


/**
 * @function
 * @brief  Move Atlas based on desired joint angles
 */
void MoveDesireDofs( atlas::AtlasKinematics* _AK, kinematics::Skeleton *_atlas, 
		     const Eigen::VectorXd &_dofs, 
		     const double &_leftKp, const double &_rightKp, 
		     const double &_otherKp, const double &_leftKd, 
		     const double &_rightKd, const double &_otherKd, 
		     const int &_N ) {

  Eigen::VectorXd tmpPos = _atlas->getPose();
  Eigen::VectorXd deltaPos = (_dofs- tmpPos) / ( (double) _N );

  for ( int i = 0; i < _N; i++) {

    PublishCommand( _AK, tmpPos, _leftKp, _rightKp, _otherKp, _leftKd, _rightKd, _otherKd );
    gLoopRate->sleep();
    tmpPos += deltaPos;
  }

}


/**
 * @function PublishCommand
 * @briefPublish a joint commands based on a dart pos 
 */
void PublishCommand( atlas::AtlasKinematics *_AK, const Eigen::VectorXd &_dofs, 
		     const double &_leftKp, const double &_rightKp, const double &_otherKp, 
		     const double &_leftKd, const double &_rightKd, const double &_otherKd ) {

  for (int j = 0; j < 4; j++) {
    gAtlasCommand.position[j] = _dofs( _AK->dof_misc[j] );
    gAtlasCommand.kp_position[j] = gAtlasCommand_saved.kp_position[j] * _otherKp;
    gAtlasCommand.kd_position[j] = gAtlasCommand_saved.kd_position[j] * _otherKd;
  }
  
  for (int j = 0; j < atlas::NUM_MANIPULATORS; j++) {
    for (int k = 0; k < 6; k++) {
      gAtlasCommand.position[j*6+k+4] = _dofs( _AK->dof_ind[j][k] );
      gAtlasCommand.kp_position[j*6+k+4] = gAtlasCommand_saved.kp_position[j*6+k+4] * _otherKp;
      gAtlasCommand.kd_position[j*6+k+4] = gAtlasCommand_saved.kd_position[j*6+k+4] * _otherKd;
    }
  }
  
  // left foot kp, ki adjustment
  for (int k = 0; k < 6; k++) {
    gAtlasCommand.kp_position[atlas::MANIP_L_FOOT*6+k+4] = gAtlasCommand_saved.kp_position[atlas::MANIP_L_FOOT*6+k+4] * _leftKp;
    gAtlasCommand.kd_position[atlas::MANIP_L_FOOT*6+k+4] = gAtlasCommand_saved.kd_position[atlas::MANIP_L_FOOT*6+k+4] * _leftKd;
  }
  
  // right foot kp, ki adjustment
  for (int k = 0; k < 6; k++) {
    gAtlasCommand.kp_position[atlas::MANIP_R_FOOT*6+k+4] = gAtlasCommand_saved.kp_position[atlas::MANIP_R_FOOT*6+k+4] * _rightKp;
    gAtlasCommand.kd_position[atlas::MANIP_R_FOOT*6+k+4] = gAtlasCommand_saved.kd_position[atlas::MANIP_R_FOOT*6+k+4] * _rightKd;
  }
  
  // Publish them
  gPubAtlasCommand.publish( gAtlasCommand );
  
}
 

/**
 * @function
 * @brief Move Atlas based on comIK
 */
void  MoveCOMIK( atlas::AtlasKinematics *_AK, 
		 kinematics::Skeleton *_atlasSkel, 
		 Eigen::Matrix4d &_Twb,
		 Eigen::Matrix4d _Twm[atlas::NUM_MANIPULATORS],
		 Eigen::VectorXd &_dofs, 
		 Eigen::Vector3d _comDelta, 
		 Eigen::Vector3d _leftDelta, 
		 Eigen::Vector3d _rightDelta, 
		 const double &_leftKp, 
		 const double &_rightKp, 
		 const double &_otherKp,
		 const double &_leftKd, 
		 const double &_rightKd, 
		 const double &_otherKd,
		 const int _N ) {

  printf(" Start MoveCOMIK \n");

  // Save dofs
  Eigen::VectorXd dofs_save; 
  dofs_save = _atlasSkel->getPose();

  std::cout << "[MoveCOMIK] POS ERROR: " << (dofs_save - _dofs).norm() << std::endl;

  _dofs = _atlasSkel->getPose();

  // Intial 
  double theta = M_PI / (_N-1);   

  Eigen::Vector3d comStart = _AK->getCOMW( _atlasSkel, _Twb );
  Eigen::Vector3d dcom = comStart;

  _Twm[atlas::MANIP_L_FOOT] = _AK->getLimbTransW( _atlasSkel, _Twb, atlas::MANIP_L_FOOT );
  _Twm[atlas::MANIP_R_FOOT] = _AK->getLimbTransW( _atlasSkel, _Twb, atlas::MANIP_R_FOOT );

  Eigen::Vector3d leftStart = _Twm[atlas::MANIP_L_FOOT].block<3,1>(0,3);
  Eigen::Vector3d rightStart = _Twm[atlas::MANIP_R_FOOT].block<3,1>(0,3);

  atlas::IK_Mode mode[atlas::NUM_MANIPULATORS];
  mode[atlas::MANIP_L_FOOT] = atlas::IK_MODE_SUPPORT;
  mode[atlas::MANIP_R_FOOT] = atlas::IK_MODE_WORLD;
  mode[atlas::MANIP_L_HAND] = atlas::IK_MODE_FIXED;
  mode[atlas::MANIP_R_HAND] = atlas::IK_MODE_FIXED;


  /**************************************
   * Print out info
   **************************************/
  _comDelta /= 2.0;
  _leftDelta /= 2.0;
  _rightDelta /= 2.0;
  std::cout << "com delta: " << _comDelta.transpose() << std::endl;
  std::cout << "left delta: " << _leftDelta.transpose() << std::endl;
  std::cout << "right delta: " << _rightDelta.transpose() << std::endl;

  std::cout << "com start: " << dcom.transpose() << std::endl;
  std::cout << "left start:\n" << _Twm[atlas::MANIP_L_FOOT] << std::endl; 
  std::cout << "right start:\n" << _Twm[atlas::MANIP_R_FOOT] << std::endl; 

  _dofs = _atlasSkel->getPose();


  /****************************************
   * COMIK and publish
   ***************************************/
  for (int i = 0; i < _N; i++) {

    dcom = comStart + _comDelta * (1 - cos(theta * i));
    _Twm[atlas::MANIP_L_FOOT].block<3,1>(0,3) = leftStart + _leftDelta * (1 - cos(theta * i));
    _Twm[atlas::MANIP_R_FOOT].block<3,1>(0,3) = rightStart + _rightDelta * (1 - cos(theta * i));

    assert( _AK->comIK( _atlasSkel, dcom, _Twb, mode, _Twm, _dofs) == true);

    _dofs = _atlasSkel->getPose();

    PublishCommand( _AK, _dofs, _leftKp, _rightKp, _otherKp, _leftKd, _rightKd, _otherKd );
    gLoopRate->sleep();

  }


  /********************************************
   * Print end info
   ********************************************/
  std::cout << "dcom end: " << dcom.transpose() << std::endl;
  std::cout << "left foot end:\n" << _Twm[atlas::MANIP_L_FOOT] << std::endl; 
  std::cout << "right foot end:\n" << _Twm[atlas::MANIP_R_FOOT] << std::endl; 

  printf(" End MoveCOMIK \n");
}

/**
 * @function UpdateDofs
 @brief Read gJointAngles and update _dofs
 */
void UpdateDofs( Eigen::VectorXd &_dofs ) {

  // Update gJointAngles - spinOnce
  sleep(1);
  std::cout << "\nSpin Once NOW!" << std::endl;
  ros::spinOnce();
  _dofs.resize( gAtlasSkel->getNumDofs() );

  // Set dart Dofs
  mapJointState2DartPose( _dofs, gJointAngles );

}

/**
 * @function MoveJointTrajectoryAdv
 * @brief Move Atlas based on joint trajectory and support info
 */
void MoveJointTractoryAdv( atlas::AtlasKinematics *AK, 
			   kinematics::Skeleton *_atlasSkel, 
			   Eigen::VectorXd &dofs,
			   const std::vector<Eigen::VectorXd> &_zmp,
			   const std::vector<int> supportInfo,
			   const Eigen::VectorXd &double_support_kp, const Eigen::VectorXd &double_support_ki,
			   const Eigen::VectorXd &left_support_kp, const Eigen::VectorXd &left_support_ki,
			   const Eigen::VectorXd &right_support_kp, const Eigen::VectorXd &right_support_ki) {

  // Update pos
  Eigen::VectorXd dofs_save; 
  dofs_save = _atlasSkel->getPose();

  UpdateDofs( dofs );


  std::cout << "*************************************" << std::endl;
  std::cout << "Before publishing trajectory" << std::endl;
  std::cout << "*************************************" << std::endl;
  std::cout << "POS ERROR: " << (dofs_save - dofs).norm() << std::endl;
  std::cout << "*************************************" << std::endl;


  for (int i = 0; i < _zmp.size(); i++) {
    switch (supportInfo[i]) {

      case DOUBLE_SUPPORT:

        for (int j = 0; j < 4; j++) {
          gAtlasCommand.position[j] = _zmp[i](j);
          gAtlasCommand.kp_position[j] = gAtlasCommand_saved.kp_position[j] * double_support_kp(j);
          gAtlasCommand.kd_position[j] = gAtlasCommand_saved.kd_position[j] * double_support_ki(j);
        }

        for (int j = 0; j < atlas::NUM_MANIPULATORS; j++) {
          for (int k = 0; k < 6; k++) {
            gAtlasCommand.position[j*6+k+4] = _zmp[i](j*6+k+4);
            gAtlasCommand.kp_position[j*6+k+4] = gAtlasCommand_saved.kp_position[j*6+k+4] * 
	      double_support_kp(j*6+k+4);
            gAtlasCommand.kd_position[j*6+k+4] = gAtlasCommand_saved.kd_position[j*6+k+4] * 
	      double_support_ki(j*6+k+4);
          }
        }
        if( gJohnnieDebug ) std::cout << "DOUBLE_SUPPORT: " << double_support_kp.transpose() << std::endl;
        break;
	
      case LEFT_SUPPORT:
        for (int j = 0; j < 4; j++) {
          gAtlasCommand.position[j] = _zmp[i](j);
          gAtlasCommand.kp_position[j] = gAtlasCommand_saved.kp_position[j] * left_support_kp(j);
          gAtlasCommand.kd_position[j] = gAtlasCommand_saved.kd_position[j] * left_support_ki(j);
        }
        for (int j = 0; j < atlas::NUM_MANIPULATORS; j++) {
          for (int k = 0; k < 6; k++) {
            gAtlasCommand.position[j*6+k+4] = _zmp[i](j*6+k+4);
            gAtlasCommand.kp_position[j*6+k+4] = gAtlasCommand_saved.kp_position[j*6+k+4] * 
                                                  left_support_kp(j*6+k+4);
            gAtlasCommand.kd_position[j*6+k+4] = gAtlasCommand_saved.kd_position[j*6+k+4] * 
	      left_support_ki(j*6+k+4);
          }
        }
	if( gJohnnieDebug ) { std::cout << "LEFT_SUPPORT: " << left_support_kp.transpose() << std::endl; }
	break;
	
      case RIGHT_SUPPORT:
        for (int j = 0; j < 4; j++) {
          gAtlasCommand.position[j] = _zmp[i](j);
          gAtlasCommand.kp_position[j] = gAtlasCommand_saved.kp_position[j] * right_support_kp(j);
          gAtlasCommand.kd_position[j] = gAtlasCommand_saved.kd_position[j] * right_support_ki(j);
        }
        for (int j = 0; j < atlas::NUM_MANIPULATORS; j++) {
          for (int k = 0; k < 6; k++) {
            gAtlasCommand.position[j*6+k+4] = _zmp[i](j*6+k+4);
            gAtlasCommand.kp_position[j*6+k+4] = gAtlasCommand_saved.kp_position[j*6+k+4] * 
                                                  right_support_kp(j*6+k+4);
            gAtlasCommand.kd_position[j*6+k+4] = gAtlasCommand_saved.kd_position[j*6+k+4] * 
                                                  right_support_ki(j*6+k+4);
          }
        }
	if( gJohnnieDebug ) { std::cout << "RIGHT_SUPPORT: " << right_support_kp.transpose() << std::endl; }
	break;
	
    }
    if( gJohnnieDebug ) std::cout << "JOINT_KP:";
    for (int i = 0; i < 28; i++)
      if( gJohnnieDebug ) std::cout << "\t" << gAtlasCommand.kp_position[i];
    
    if( gJohnnieDebug ) std::cout << std::endl;
    if( gJohnnieDebug ) std::cout << "JOINT_KD:";
    for (int i = 0; i < 28; i++)
      if( gJohnnieDebug ) std::cout << "\t" << gAtlasCommand.kd_position[i];
    
    if( gJohnnieDebug ) std::cout << std::endl;
    
    gAtlasCommand.header.stamp = ros::Time::now();
    gPubAtlasCommand.publish( gAtlasCommand );
    gLoopRate->sleep();
  }
  
  
  /************************************************
   * Update dofs in DART
   ************************************************/
  for (int i = 0; i < 4; i++) {
    dofs(AK->dof_misc[i]) = _zmp[_zmp.size()-1](i);
  }

  for (int i = 0; i < atlas::NUM_MANIPULATORS; i++) {
    for (int j = 0; j < 6; j++) {
      dofs(AK->dof_ind[i][j]) = _zmp[_zmp.size()-1][i*6+j+4];
    }
  }

  dofs_save = dofs;
  UpdateDofs(dofs);

  std::cout << "*************************************" << std::endl;
  std::cout << "After publishing trajectory" << std::endl;
  std::cout << "*************************************" << std::endl;
  std::cout << "POS ERROR: " << (dofs_save - dofs).norm() << std::endl;
  std::cout << "*************************************" << std::endl;

}
