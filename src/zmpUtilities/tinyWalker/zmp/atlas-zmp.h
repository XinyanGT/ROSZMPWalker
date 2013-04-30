/**
 * @file atlas-zmp.h
 * @brief 
 * @author A. Huaman (based on original code from M. Zucker)
 * @date 2012/04/26
 */

#ifndef _ATLAS_ZMP_H_
#define _ATLAS_ZMP_H_

#include <stdlib.h>
#include <src/atlas_joint_count.h>

/**
 * @enum stance_t
 * @brief In what foot is Atlas standing?
 */
enum stance_t {
  DOUBLE_LEFT  = 0,
  DOUBLE_RIGHT = 1,
  SINGLE_LEFT  = 2,
  SINGLE_RIGHT = 3,
};

/**
 * @struct zmp_traj_element
 */
typedef struct zmp_traj_element {

  double angles[ATLAS_JOINT_COUNT];
  // XYZ pos/vel/accel in frame of stance ANKLE
  // (translated up ~10cm from foot -- i.e. z coord is 10cm less)
  double com[3][3];
  double zmp[2]; // XY of zmp in frame of stance ANKLE
  double forces[2][3]; // right/left predicted normal forces
  double torque[2][3]; // right/left predicted moments XYZ
  // TODO: add orientation for IMU
  stance_t stance;
} zmp_traj_element_t;


enum {
  TRAJ_FREQ_HZ = 200,
  MAX_TRAJ_SIZE = 2000,
};

/**
 * @struct zmp_traj
 */
typedef struct zmp_traj {
  zmp_traj_element_t traj[MAX_TRAJ_SIZE];
  size_t count;
} zmp_traj_t;


#endif /** _ATLAS_ZMP_H_ */


