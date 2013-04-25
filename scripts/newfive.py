#!/usr/bin/env python
import roslib; roslib.load_manifest('tutorial_atlas_control')
import rospy, yaml, sys
from osrf_msgs.msg import JointCommands
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil
from std_msgs.msg import String


atlasJointNames = [
  'atlas::back_lbz', 'atlas::back_mby', 'atlas::back_ubx', 'atlas::neck_ay',
  'atlas::l_leg_uhz', 'atlas::l_leg_mhx', 'atlas::l_leg_lhy', 'atlas::l_leg_kny', 'atlas::l_leg_uay', 'atlas::l_leg_lax',
  'atlas::r_leg_uhz', 'atlas::r_leg_mhx', 'atlas::r_leg_lhy', 'atlas::r_leg_kny', 'atlas::r_leg_uay', 'atlas::r_leg_lax',
  'atlas::l_arm_usy', 'atlas::l_arm_shx', 'atlas::l_arm_ely', 'atlas::l_arm_elx', 'atlas::l_arm_uwy', 'atlas::l_arm_mwx',
  'atlas::r_arm_usy', 'atlas::r_arm_shx', 'atlas::r_arm_ely', 'atlas::r_arm_elx', 'atlas::r_arm_uwy', 'atlas::r_arm_mwx']

command = JointCommands()
tranum = 0
traj_len = 0
traj_name = ''

def jointStatesCallback(msg):
  global command
  global tranum
  global traj_len
  global traj_name

  if tranum == traj_len:
    print "Reach the end of trajectory, should quit now"
    sys.exit(1) 

  y = traj_yaml[traj_name][tranum]
  commandPosition = array([ float(x) for x in y[0].split() ])
  command.position = [ float(x) for x in commandPosition ]
#  command.position  = zeros(len(command.position))
  print tranum
#  print command.position
  pub.publish(command)
  tranum += 1
  print command


if __name__ == '__main__':

  # first make sure the input arguments are correct
  if len(sys.argv) != 3:
    print "usage: traj_yaml.py YAML_FILE TRAJECTORY_NAME"
    print "  where TRAJECTORY is a dictionary defined in YAML_FILE"
    sys.exit(1)


  traj_yaml = yaml.load(file(sys.argv[1], 'r'))
  traj_name = sys.argv[2]
  if not traj_name in traj_yaml:
    print "unable to find trajectory %s in %s" % (traj_name, sys.argv[1])
    sys.exit(1)

  traj_len = len(traj_yaml[traj_name])

  # initialize JointCommands message
  command.name = list(atlasJointNames)
  n = len(command.name)
  command.position     = zeros(n)
  command.velocity     = zeros(n)
  command.effort       = zeros(n)
  command.kp_position  = zeros(n)
  command.ki_position  = zeros(n)
  command.kd_position  = zeros(n)
  command.kp_velocity  = zeros(n)
  command.i_effort_min = zeros(n)
  command.i_effort_max = zeros(n)

  # now get gains from parameter server
  rospy.init_node('tutorial_atlas_control')

  for i in xrange(len(command.name)):
    name = command.name[i]
    command.kp_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/p') * 8;
#    command.kp_position[i]  = 100;
    command.ki_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i');
    command.kd_position[i]  = rospy.get_param('atlas_controller/gains/' + name[7::] + '/d') * 1;
#    command.kd_position[i]  = 1;
#    command.i_effort_max[i] = rospy.get_param('atlas_controller/gains/' + name[7::] + '/i_clamp')
    command.i_effort_max[i] = 0;
#    command.i_effort_min[i] = -command.i_effort_max[i]
    command.i_effort_min[i] = -0;


  print command
  # set up the publisher
  pub = rospy.Publisher('/atlas/joint_commands', JointCommands)

  # Setup subscriber to atlas states
  rospy.Subscriber("/atlas/joint_states", JointState, jointStatesCallback)
  rospy.spin()
