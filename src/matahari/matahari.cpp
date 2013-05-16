/**
 * @file matahari.cpp
 */
#include <adaptedZMP/zmpWalkGenerator.h>
#include <adaptedZMP/atlas_zmp.h>
#include <adaptedZMP/footprint.h>

#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/BodyNode.h>
#include <robotics/parser/dart_parser/DartLoader.h>

#include <utils/AtlasPaths.h> // For ATLAS_DATA_PATH

bool mainDebug = true;

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  //-- Initialize atlasKin
  atlas::AtlasKinematics atlasKin;
  dynamics::SkeletonDynamics *atlasSkel;
  
  DartLoader dl;
  simulation::World *world = dl.parseWorld( ATLAS_DATA_PATH "atlas/atlas_world.urdf" );
  atlasSkel = world->getSkeleton( "atlas" );

  if( !atlasSkel ) {
    std::cout<<" Atlas Skeleton no loaded, exiting!"<< std::endl;
    return 1;
  }
  
  atlasKin.init( atlasSkel );


  //-- Set some walking variables

  // helper variables and classes
  double deg = M_PI/180; // for converting from degrees to radians

  // Set initial pose to find footsep
  Eigen::VectorXd initPose = atlasSkel->getPose().setZero();
  initPose( 20 ) =  15*deg; initPose( 22 ) = -15*deg; // LSR / RSR
  initPose( 15 ) =  20*deg; initPose( 17 ) =  20*deg; // LSP / RSP
  initPose( 29 ) = -40*deg; initPose( 30 ) = -40*deg; // LEP / REP
  atlasSkel->setPose( initPose );

  /**< half of horizontal separation distance between feet */
  double footsep_y = ( atlasKin.getLimbTransW( atlasSkel, Eigen::Matrix4d::Identity(), atlas::MANIP_L_FOOT)(1,3) - 
		       atlasKin.getLimbTransW( atlasSkel, Eigen::Matrix4d::Identity(), atlas::MANIP_R_FOOT)(1,3) ) / 2.0;
    
  /**<  height of COM above ANKLE */
  double com_height = atlasKin.getCOMW( atlasSkel, Eigen::Matrix4d::Identity() )(2) - atlasKin.getLimbTransW( atlasSkel, 
													      Eigen::Matrix4d::Identity(), 
													      atlas::MANIP_L_FOOT )(2,3);

  double foot_liftoff_z = 0.05; // foot liftoff height
  double step_length = 0.3;

  bool walk_sideways = false;  
  walktype walk_type = walk_canned;
  double walk_circle_radius = 5.0;
  double walk_dist = 5;

  double com_ik_ascl = 0;
  
  double zmpoff_y = 0; // lateral displacement between zmp and ankle
  double zmpoff_x = 0;
  
  double lookahead_time = 2.5;
  
  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.05;
  double single_support_time = 0.70;
  
  size_t max_step_count = 25;
  
  double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller
  
  zmp::ik_error_sensitivity ik_sense = zmp::ik_strict;

  //-- build initial state

  // the actual state
  ZMPWalkGenerator walker( atlasKin,
			   atlasSkel,
			   ik_sense,
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
			   lookahead_time );

  if( mainDebug ) { std::cout<<"\n Foot separation: "<<footsep_y<<"COM Height: "<<com_height<<std::endl; }

  // Fill init context
  ZMPReferenceContext initContext;

  initContext.stance = DOUBLE_LEFT;
  initContext.comX = Eigen::Vector3d( atlasSkel->getWorldCOM().x(), 0, 0 );
  initContext.comY = Eigen::Vector3d( atlasSkel->getWorldCOM().y(), 0, 0 );
  initContext.comZ = Eigen::Vector3d( atlasSkel->getWorldCOM().z(), 0, 0 );
  initContext.eX = 0.0; initContext.eY = 0.0;
  initContext.pX = 0.0; initContext.pY = 0.0;  

  // fill in the kstate
  initContext.state.jvalues.resize( atlasSkel->getNumDofs() );
  initContext.state.jvalues = initPose;

  Eigen::Affine3d bodyXform;
  Eigen::Matrix4d Twb = atlasSkel->getNode("pelvis")->getWorldTransform();
  bodyXform.matrix() = Twb;
  initContext.state.setXform( bodyXform );
  
  // Store feet positions in global frame
  initContext.feet[0] = atlasSkel->getNode( "l_foot" )->getWorldTransform();
  initContext.feet[1] = atlasSkel->getNode( "r_foot" )->getWorldTransform();

  if( mainDebug ) { 
    std::cout << "Init context body transform: \n" << bodyXform.matrix() << std::endl; 
    std::cout << "Init context pose: \n"<< initContext.state.jvalues.transpose() << std::endl; 
    std::cout << "CoM initial: " << initContext.comX <<", "<< initContext.comY <<", "<<initContext.comZ << std::endl; 
    std::cout << "Transformation to left leg: \n" << initContext.feet[0].matrix() << std::endl;
    std::cout << "Transformation to right leg: \n" << initContext.feet[1].matrix() << std::endl;
  }

  // initialize
  walker.initialize(initContext);
    
  //-- build ourselves some footprints
  Footprint initLeftFoot = Footprint( initContext.feet[0], true );

  std::vector<Footprint> footprints;
  
  if( mainDebug ) {
    std::cout <<" Walk dist: "<< walk_dist << std::endl;
    std::cout <<" Foot separation: "<< footsep_y << std::endl;
    std::cout <<" Step length: "<< step_length << std::endl;
    std::cout <<" Initial left foot position: "<< initLeftFoot.x()<<", "<<initLeftFoot.y()<<std::endl;
  }
  
  footprints = walkLine(walk_dist, footsep_y,
			step_length,
			initLeftFoot );

  printf( "Num footprints: %d \n", footprints.size() );
  
  FILE* fpraw;
  fpraw = fopen("footprint_raw.txt", "w");
  for( int i = 0; i < footprints.size(); ++i ) {
    printf("Footprint[%d] pos: %f %f %f \n", i, footprints[i].x(), footprints[i].y(), footprints[i].theta() );
    fprintf( fpraw, "%d, %f %f %f \n", i, footprints[i].x(), footprints[i].y(), footprints[i].theta() );
  }
  fclose( fpraw );


  // Feet positions at init
  //Tw_leftFoot = atlasKin.getLimbTransW( atlasSkel, Twb, atlas::MANIP_L_FOOT );
  //Tw_rightFoot = atlasKin.getLimbTransW( atlasSkel, Twb, atlas::MANIP_R_FOOT );

  //-- and then build up the walker

  // Start wait time
  walker.stayDogStay(startup_time * TRAJ_FREQ_HZ);
  // Foot steps
  for(std::vector<Footprint>::iterator it = footprints.begin(); it != footprints.end(); it++) {
    walker.addFootstep(*it);
  }
  // Close wait time
  walker.stayDogStay(shutdown_time * TRAJ_FREQ_HZ);
  
  printf("Size ref for walkLine : %d \n", walker.ref.size() );

  FILE* zmp; FILE* leftFoot; FILE* rightFoot; 
  zmp = fopen( "zmp.txt", "w" );
  leftFoot = fopen( "leftFoot.txt", "w" );
  rightFoot = fopen( "rightFoot.txt", "w" );

  for( int i = 0; i < walker.ref.size(); ++i ) {
    fprintf( zmp, "%d  %f %f \n", i, walker.ref[i].pX, walker.ref[i].pY );
    Eigen::Vector3d lf = walker.ref[i].feet[0].matrix().block(0,3,3,1);
    Eigen::Vector3d rf = walker.ref[i].feet[1].matrix().block(0,3,3,1);
    fprintf( leftFoot, "%d %f %f %f %d \n", i, lf(0), lf(1), lf(2), walker.ref[i].stance );
    fprintf( rightFoot, "%d %f %f %f \n", i, rf(0), rf(1), rf(2) );
  }

  fclose( zmp );
  fclose( leftFoot );
  fclose( rightFoot );

  
  //-- have the walker run preview control and pass on the output
  walker.bakeIt();
  
  return 0;
}
