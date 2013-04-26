#include "zmp_walk.h"
#include <zmp/ZmpGUI.h>
#include <getopt.h>

#include <dynamics/SkeletonDynamics.h>
#include <kinematics/FileInfoSkel.hpp>


///////////////////////////////////////////////////////////////////////////////
// Usage and input handling
///////////////////////////////////////////////////////////////////////////////
void usage(std::ostream& ostr) {
	ostr <<
			"usage: publish_and_readonce [OPTIONS]\n"
			"\n"
			"OPTIONS:\n"
			"\n"
			"  -g, --show-gui                    Show a GUI after computing trajectories.\n"
			"  -R, --use-ros                     Send trajectory via ROS after computing.\n"
			"  -I, --ik-errors                   IK error handling: strict/sloppy\n"
			"  -w, --walk-type                   Set type: canned/line/circle\n"
			"  -D, --walk-distance               Set maximum distance to walk\n"
			"  -r, --walk-circle-radius          Set radius for circle walking\n"
			"  -c, --max-step-count=NUMBER       Set maximum number of steps\n"
			"  -y, --foot-separation-y=NUMBER    Half-distance between feet\n"
			"  -z, --foot-liftoff-z=NUMBER       Vertical liftoff distance of swing foot\n"
			"  -l, --step-length=NUMBER          Max length of footstep\n"
			"  -S, --walk-sideways               Should we walk sideways? (canned gait only)\n"
			"  -h, --com-height=NUMBER           Height of the center of mass\n"
			"  -a, --comik-angle-weight=NUMBER   Angle weight for COM IK\n"
			"  -Y, --zmp-offset-y=NUMBER         Lateral distance from ankle to ZMP\n"
			"  -X, --zmp-offset-x=NUMBER         Forward distance from ankle to ZMP\n"
			"  -T, --lookahead-time=NUMBER       Lookahead window for ZMP preview controller\n"
			"  -p, --startup-time=NUMBER         Initial time spent with ZMP stationary\n"
			"  -n, --shutdown-time=NUMBER        Final time spent with ZMP stationary\n"
			"  -d, --double-support-time=NUMBER  Double support time\n"
			"  -s, --single-support-time=NUMBER  Single support time\n"
			"  -P, --zmp-jerk-penalty=NUMBER     P-value for ZMP preview controller\n"
			"  -H, --help                        See this message\n";
}

double getdouble(const char* str) {
	char* endptr;
	double d = strtod(str, &endptr);
	if (!endptr || *endptr) {
		std::cerr << "Error parsing number on command line!\n\n";
		usage(std::cerr);
		exit(1);
	}
	return d;
}

long getlong(const char* str) {
	char* endptr;
	long d = strtol(str, &endptr, 10);
	if (!endptr || *endptr) {
		std::cerr << "Error parsing number on command line!\n\n";
		usage(std::cerr);
		exit(1);
	}
	return d;
}

enum walktype {
	walk_canned,
	walk_line,
	walk_circle
};

walktype getwalktype(const std::string& s) {
	if (s == "canned") {
		return walk_canned;
	} else if (s == "line") {
		return walk_line;
	} else if (s == "circle") {
		return walk_circle;
	} else {
		std::cerr << "bad walk type " << s << "\n";
		usage(std::cerr);
		exit(1);
	}
}

enum ik_error_sensitivity {
	ik_strict,
	ik_sloppy,
	ik_swing_permissive
};

ik_error_sensitivity getiksense(const std::string& s) {
	if (s == "strict") {
		return ik_strict;
	} else if (s == "sloppy") {
		return ik_sloppy;
	} else if (s == "permissive") {
		return ik_swing_permissive;
	} else {
		std::cerr << "bad ik error sensitivity " << s << "\n";
		usage(std::cerr);
		exit(1);
	}
}

///////////////////////////////////////////////////////////////////////////////
// MAIN
///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv ) {
	// Command line inputs
	bool show_gui = false;
	bool use_ros = false;

	walktype walk_type = walk_canned;
	double walk_circle_radius = 5.0;
	double walk_dist = 20;

	double footsep_y = 0.085; // half of horizontal separation distance between feet
	double foot_liftoff_z = 0.05; // foot liftoff height

	double step_length = 0.05;
	bool walk_sideways = false;

	double com_height = 0.48; // height of COM above ANKLE
	double com_ik_ascl = 0;

	double zmpoff_y = 0; // lateral displacement between zmp and ankle
	double zmpoff_x = 0;

	double lookahead_time = 2.5;

	double startup_time = 1.0;
	double shutdown_time = 1.0;
	double double_support_time = 0.05;
	double single_support_time = 0.70;

	size_t max_step_count = 4;

	double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller

	ik_error_sensitivity ik_sense = ik_strict;

	// Parse command line inputs
	const struct option long_options[] = {
			{ "show-gui",            no_argument,       0, 'g' },
			{ "use-ros",             no_argument,       0, 'R' },
			{ "ik-errors",           required_argument, 0, 'I' },
			{ "walk-type",           required_argument, 0, 'w' },
			{ "walk-distance",       required_argument, 0, 'D' },
			{ "walk-circle-radius",  required_argument, 0, 'r' },
			{ "max-step-count",      required_argument, 0, 'c' },
			{ "foot-separation-y",   required_argument, 0, 'y' },
			{ "foot-liftoff-z",      required_argument, 0, 'z' },
			{ "step-length",         required_argument, 0, 'l' },
			{ "walk-sideways",       no_argument,       0, 'S' },
			{ "com-height",          required_argument, 0, 'h' },
			{ "comik-angle-weight",  required_argument, 0, 'a' },
			{ "zmp-offset-y",        required_argument, 0, 'Y' },
			{ "zmp-offset-x",        required_argument, 0, 'X' },
			{ "lookahead-time",      required_argument, 0, 'T' },
			{ "startup-time",        required_argument, 0, 'p' },
			{ "shutdown-time",       required_argument, 0, 'n' },
			{ "double-support-time", required_argument, 0, 'd' },
			{ "single-support-time", required_argument, 0, 's' },
			{ "zmp-jerk-penalty",    required_argument, 0, 'P' },
			{ "help",                no_argument,       0, 'H' },
			{ 0,                     0,                 0,  0  },
	};

	const char* short_options = "gRI:w:D:r:c:y:z:l:Sh:a:Y:X:T:p:n:d:s:P:H";

	int opt, option_index;

	while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 ) {
		switch (opt) {
		case 'g': show_gui = true; break;
		case 'R': use_ros = true; break;
		case 'I': ik_sense = getiksense(optarg); break;
		case 'w': walk_type = getwalktype(optarg); break;
		case 'D': walk_dist = getdouble(optarg); break;
		case 'r': walk_circle_radius = getdouble(optarg); break;
		case 'c': max_step_count = getlong(optarg); break;
		case 'y': footsep_y = getdouble(optarg); break;
		case 'z': foot_liftoff_z = getdouble(optarg); break;
		case 'l': step_length = getdouble(optarg); break;
		case 'S': walk_sideways = true; break;
		case 'h': com_height = getdouble(optarg); break;
		case 'a': com_ik_ascl = getdouble(optarg); break;
		case 'Y': zmpoff_y = getdouble(optarg); break;
		case 'X': zmpoff_x = getdouble(optarg); break;
		case 'T': lookahead_time = getdouble(optarg); break;
		case 'p': startup_time = getdouble(optarg); break;
		case 'n': shutdown_time = getdouble(optarg); break;
		case 'd': double_support_time = getdouble(optarg); break;
		case 's': single_support_time = getdouble(optarg); break;
		case 'P': zmp_jerk_penalty = getdouble(optarg); break;
		case 'H': usage(std::cout); exit(0); break;
		default:  usage(std::cerr); exit(1); break;
		}
	}

	/* Initialize ROS */
	double frequency = 200;
	//FIXME: ROS dependent
	if(use_ros) {
		ros::init( argc, argv, "publish_and_readonce" );
		rosnode = new ros::NodeHandle();
		loop_rate = new ros::Rate(frequency);

		ros::Time last_ros_time_;
		// Wait until sim is active (play)
		bool wait = true;

		while( wait ) {
			last_ros_time_ = ros::Time::now();
			if( last_ros_time_.toSec() > 0 ) {
				wait = false;
			}
		}

		// init ros joints
		RosJointInit();

		// ros topic subscribtions
		ros::SubscribeOptions jointStatesSo =
				ros::SubscribeOptions::create<sensor_msgs::JointState>(
						"/atlas/joint_states", 1, GetJointStates,
						ros::VoidPtr(), rosnode->getCallbackQueue());

		jointStatesSo.transport_hints = ros::TransportHints().unreliable();
		ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);

		pub_joint_commands_ =
				rosnode->advertise<osrf_msgs::JointCommands>(
						"/atlas/joint_commands", 1, true);
	}

	/* Initialize AK */
	if(!_ak) {
		DartLoader dart_loader;
		World *mWorld = dart_loader.parseWorld(ATLAS_DATA_PATH "atlas/atlas_world.urdf");
		_atlas = mWorld->getSkeleton("atlas");
		_ak = new AtlasKinematics();
		_ak->init(_atlas);
	}
	_atlas->setPose(_atlas->getPose().setZero(), true);
	AtlasKinematics *AK = _ak;


	/* Begin generating trajectories */

	/* Trajectory that stores dof ticks */
	vector<VectorXd> joint_traj;

	/* Setup dofs initial conditions */
	/* Relax */
	VectorXd dofs = _atlas->getPose().setZero();
	_atlas->setPose(dofs, true);

	const int relax_ticks = 1000;
	Relax(AK, _atlas, dofs, joint_traj, relax_ticks);


	/* Walking variables */
	IK_Mode mode[NUM_MANIPULATORS];
	mode[MANIP_L_FOOT] = IK_MODE_SUPPORT;
	mode[MANIP_R_FOOT] = IK_MODE_WORLD;
	mode[MANIP_L_HAND] = IK_MODE_FIXED;
	mode[MANIP_R_HAND] = IK_MODE_FIXED;

	Vector3d comDelta = Vector3d::Zero();
	Vector3d leftDelta = Vector3d::Zero();
	Vector3d rightDelta = Vector3d::Zero();

	int N = 0;

	Matrix4d Twm[NUM_MANIPULATORS];
	Twm[MANIP_L_FOOT] = AK->getLimbTransB(_atlas, MANIP_L_FOOT);
	Twm[MANIP_R_FOOT] = AK->getLimbTransB(_atlas, MANIP_R_FOOT);

	Matrix4d Twb;
	Twb.setIdentity();

	VectorXd dofs_save;

	/* Move COM down */
	comDelta << 0, 0, -0.04;
	leftDelta.setZero();
	rightDelta.setZero();
	const int com_ticks = 1000;
	genCOMIKTraj(AK, _atlas, Twb, Twm, dofs, comDelta, leftDelta, rightDelta, joint_traj, com_ticks);

	/* ZMP Walking */

	/* ZMP parameters */
	// number of steps to walk
	int numSteps = 12;
	// lenght of a half step
	double stepLength = 0.15;
	// half foot seperation
	dofs_save = _atlas->getPose();

	cout << "********************************************" << endl;
	cout << "Start ZMP walking" << endl;
	cout << "*************************************" << endl;
	cout << "POS ERROR: " << (dofs_save - dofs).norm() << endl;
	cout << "*************************************" << endl;

	_atlas->setPose(dofs);

	double footSeparation = (AK->getLimbTransW(_atlas, Twb, MANIP_L_FOOT)(1,3) -
			AK->getLimbTransW(_atlas, Twb, MANIP_R_FOOT)(1,3) ) / 2;
	cout << "Half foot seperation: " << footSeparation << endl;
	// one step time
	double stepDuration = 1.0;
	// move ZMP time
	double slopeTime = 0.15;
	// keep ZMP time
	double levelTime = 0.85;
	// command sending period
	double dt = 1/frequency;
	// height of COM
	double zg = AK->getCOMW(_atlas, Twb)(2) - AK->getLimbTransW(_atlas, Twb, MANIP_L_FOOT)(2,3);
	cout << "zg " << zg << endl;
	// preview how many steps
	int numPreviewSteps = 2;

	//  double Qe = 1;
	//  double R = 1e-6;
	double Qe = 1e7;
	double R = 10;

	/****************************************
	 * Generate joint trajectory
	 ***************************************/
	gZU.setParameters( dt, 9.81, dofs  );
	gZU.generateZmpPositions( numSteps, true,
			stepLength, footSeparation,
			stepDuration,
			slopeTime,
			levelTime );

	gZU.getControllerGains( Qe, R, zg, numPreviewSteps );
	gZU.generateCOMPositions();
	gZU.getJointTrajectories();
	gZU.print( "jointsWholeBody.txt", gZU.mWholeBody );

	gZU.mDartDofs;
	joint_traj.insert(joint_traj.end(), gZU.mDartDofs.begin(), gZU.mDartDofs.end());




	// Bake me some GUI viz
	FileInfoSkel<SkeletonDynamics> model;
	model.loadFile(ATLAS_DATA_PATH"/skel/ground1.skel", SKEL);
	SkeletonDynamics *ground = dynamic_cast<SkeletonDynamics *>(model.getSkel());
	ground->setName("ground");

	vector<SkeletonDynamics *> skels;
	skels.push_back(ground);
	skels.push_back(_atlas);


	ZmpGUI gui(skels);
	gui.bake(joint_traj);
	glutInit(&argc, argv);
	gui.initWindow(640, 480, "Atlas ZMP Walking");
    glutMainLoop();

	/**************************************
	 * Publish joint trajectory
	 ****************************************/
	//FIXME: ROS Dependent
	//MoveJointTractory(AK, _atlas, dofs, gZU.mWholeBody, 20, 20, 20, 1.2, 1.2, 1.2);



	return 0;

}
