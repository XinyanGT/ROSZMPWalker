/**
 * @file footprint.cpp
 */
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <iomanip>

#include "footprint.h"

/**
 * @function Footprint
 * @brief Constructor
 */
Footprint::Footprint(Eigen::Affine3d t, bool is_left) {

  this->transform = t;
  this->is_left = is_left;
}

/**
 * @function Footprint
 * @brief Constructor
 */
Footprint::Footprint( double x, 
		      double y, 
		      double theta, 
		      bool is_left ) {
  
  Eigen::Vector3d translation(x, y, 0);
  Eigen::AngleAxisd rotation( theta, Eigen::Vector3d(0.0,0.0,1.0) );

  Eigen::Affine3d transform( Eigen::Affine3d::Identity() );

  // Respect the order! translation and then rotation
  transform.translate( translation );
  transform.rotate( rotation );

  this->transform = transform;
  this->is_left = is_left;
}

/**
 * @function Footprint
 * @brief Yet another constructor
 */
Footprint::Footprint(){
    Footprint( 0.0, 0.0, 0.0, false );
}


/**
 * @function x
 * @brief Get x
 */
double Footprint::x() const { 
  return this->transform.translation().x(); 
}

/**
 * @function y
 * @brief Get y
 */
double Footprint::y() const{ 
  return this->transform.translation().y(); 
}

/**
 * @function theta
 * @brief Get theta 
 */
double Footprint::theta() const {
  Eigen::Vector3d forward = this->transform.rotation() * Eigen::Vector3d(1.0, 0.0, 0.0);
  return atan2(forward.y(), forward.x());
}

/**
 * @function getTransform
 * @brief Get the transform
 */
Eigen::Affine3d Footprint::getTransform() const {

    return transform;
}

/**
 * @function setTransform
 * @brief Set transform
 */
void Footprint::setTransform( Eigen::Affine3d transform ){
    this->transform = transform;
}

/**
 * @function getMidTransform 
 * @brief
 */
Eigen::Affine3d Footprint::getMidTransform( double width ) const {
  return getTransform() * Eigen::Translation3d( 0, is_left?-width:width, 0 );
}

/**
 * @function is_even
 * @brief Helper is even?
 */
bool is_even(int i) {
    return (i%2) == 0;
}

/**
 * @function is_odd
 * @brief Helper is odd?
 */
bool is_odd(int i) {
    return !is_even(i);
}

/**
 * @function walkline
 * @brief
 */
std::vector<Footprint> walkLine(double distance,
				double width,
				double max_step_length,
				Footprint stance_foot) {
  return walkCircle( 1e13,
		     distance,
		     width,
		     max_step_length,
		     3.14,
		     stance_foot );
}

/**
 * @function walkCircle
 * @brief 
 */
std::vector<Footprint> walkCircle( double radius,
				   double distance,
				   double width,
				   double max_step_length,
				   double max_step_angle,
				   Footprint stance_foot ) {
  assert(distance > 0);
  assert(width > 0);
  assert(max_step_length > 0);
  assert(max_step_angle >= -3.14159265359);
  assert(max_step_angle < 3.14159265359);
  
  bool left_is_stance_foot = stance_foot.is_left;
  
  // select stance foot, fill out transforms
  Eigen::Affine3d T_circle_to_world = stance_foot.transform* Eigen::Translation3d(0, left_is_stance_foot?-width:width, 0);
 
  // For some reason abs acts crazy so I have to do this
  double _radius_;
  if( radius > 0 ) { _radius_ = radius; }
  else { _radius_ = -1*radius; }

  double alpha = distance / _radius_;
  double outer_dist = (_radius_ + width) * alpha;
  int K_step_length = ceil(outer_dist / max_step_length);
  int K_angle = ceil(alpha / max_step_angle);
  int K = std::max(K_step_length, K_angle);
  double dTheta = alpha/K * (radius > 0 ? 1 : -1 );

    std::cout << "Alpha is: " << alpha << std::endl;
    std::cout << "outer_dist IS " << outer_dist << std::endl;
    std::cout << outer_dist / max_step_length << std::endl;
    std::cout << "Ksl IS " << K_step_length << std::endl;
    std::cout << "Kang IS " << K_angle << std::endl;
    std::cout << "K IS " << K << std::endl;
    std::cout << "dTheta IS " << dTheta << std::endl;

  // init results list
  std::vector<Footprint> result;
  
  // fill out results
  for(int i = 2; i < K + 1; i++) {
    double theta_i = dTheta * (i - 1);
    if (is_even(i) xor left_is_stance_foot) { // i odd means this step is for the stance foot
      result.push_back(Footprint((radius - width) * sin(theta_i),
				 radius - ((radius - width) * cos(theta_i)),
				 theta_i,
				 true));
    }
    else {
      result.push_back(Footprint((radius + width) * sin(theta_i),
				 radius - ((radius + width) * cos(theta_i)),
				 theta_i,
				 false));
    }
  }
  
  // fill out the last two footsteps
  double theta_last = dTheta * K;
  if (is_even(K) xor left_is_stance_foot) { // K even means we end on the stance foot
    result.push_back(Footprint((radius + width) * sin(theta_last),
			       radius - ((radius + width) * cos(theta_last)),
			       theta_last,
			       false));
    result.push_back(Footprint((radius - width) * sin(theta_last),
			       radius - ((radius - width) * cos(theta_last)),
			       theta_last,
			       true));
  }
  else {
    result.push_back(Footprint((radius - width) * sin(theta_last),
			       radius - ((radius - width) * cos(theta_last)),
			       theta_last,
			       true));
    result.push_back(Footprint((radius + width) * sin(theta_last),
			       radius - ((radius + width) * cos(theta_last)),
			       theta_last,
			       false));
  }
  
  // run through results transforming them back into the original frame of reference
  for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++) {
    it->transform = T_circle_to_world * it->transform;
  }
  result.insert(result.begin(), stance_foot);
  
  // return the result
  return result;
}

/**
 * @function turnInPlace
 */
std::vector<Footprint> turnInPlace( double desired_angle, /// The desired angle
				    double width, /// The desired angle
				    double max_step_angle, /// The maximum HALF angle between successive steps
				    Footprint from /// Where we start from. Note that this exact foot will be repeated in the output
				    ) {
  
  assert(max_step_angle >= -3.14159265359);
  assert(max_step_angle <   3.14159265359);
  assert(desired_angle >=  -3.14159265359);
  assert(desired_angle <    3.14159265359);
  assert(from.theta() >=   -3.14159265359);
  assert(from.theta() <     3.14159265359);

  double goal_angle = desired_angle - from.theta();
  if(goal_angle >= 3.14159265359) goal_angle -= 3.14159265359;
  else if( goal_angle < -3.14159265359) goal_angle -= 3.14159265359;
  double eps = 1e-10;
  return walkCircle(eps, eps*goal_angle, width, 1000, max_step_angle, from);
}

/**
 * @function compensate
 */
Eigen::Affine3d compensate( double width, bool is_left ) {

  Eigen::Affine3d c = Eigen::Affine3d::Identity();
  c.translate( Eigen::Vector3d(0, is_left?-width:width, 0) );
  return c;
}

/**
 * @function walkTo
 */
std::vector<Footprint> walkTo(
			      double width, /// The maximum HALF angle between successive steps
			      double max_step_length, /// The maximum HALF allowed length the robot may step
			      double max_step_angle, /// The maximum HALF angle between successive steps
			      Footprint from, /// Where we start from. Note that this exact foot will be repeated in the output
			      Footprint to /// Where we should end at. Note that this exact foot will be repeated in the output
			      ) {

  /* double walk_angle = to. */
  Eigen::Affine3d T_delta = from.getMidTransform(width).inverse() * to.getMidTransform(width);

  /* to.y(), to.x() */
  Eigen::Vector3d trans = T_delta.translation();
  double walk_angle = atan2( trans.y(), trans.x() );
  
  std::vector<Footprint> turn1 = turnInPlace( walk_angle, width, max_step_angle, from );
  Footprint f1 = turn1.back();
  turn1.pop_back();
  
  double walk_length = trans.norm();
  std::vector<Footprint> turn2 = walkLine(walk_length, width, max_step_length, f1);
  Footprint f2 = turn2.back();
  turn2.pop_back();
  
  std::vector<Footprint> turn3 = turnInPlace(to.theta(), width, max_step_angle, f2);
  
  std::vector<Footprint> total = turn1;
  total.insert(total.end(), turn2.begin(), turn2.end());
  total.insert(total.end(), turn3.begin(), turn3.end());
  return total;
}
