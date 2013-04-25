/**
 * @file simpleWalk.h
 * @brief
 * @author A. Huaman 
 */
void setInitPosition( zmpUtilities &_zu );
void setJointStates( const sensor_msgs::JointState::ConstPtr &_js );
void setZMPConfig( zmpUtilities &_zu );
void setJointConfig( osrf_msgs::JointCommands &_jointCommands,
		     ros::NodeHandle* _rosNode );

