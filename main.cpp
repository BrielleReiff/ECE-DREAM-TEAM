#include "CurTime.h"
#include "DeadReck.h"
#include "tLightSHM.h"
#include "Road.h"
#include "GenericController.h"
#include "CircleController.h"
#include <iostream>
#include <string>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include "GroupManager.hh"
#include "FollowerControl.hh"
#include <signal.h>
#include "LLCInterface.hh"

#include "llcplayer.hh"

using namespace std;
using PathPlan::mc_distance;

#if (__UCLIBC__ || DENSO_SEM)
int process_id = 1;
struct shared_with_critical *vars;
#else
sem_t *data_ready;
#endif

bool run;
void sig_int( int sig )
{
	run = false;
}

class UserFollowerControl : public FollowerControlBase 
{
	public:

		double error_i = 0;
		double last_error = 0;

		// Description of parameters:
		// int leader : robot id of leader
		// double sep_dist : desired separation distance between this robot and the leader
		// double x : this robot's x position (units: meters)
		// double y : this robot's y position (units: meters)
		// double yaw : this robot's heading angle (units: radians, compass coordinate system)
		// double req_speed : speed input generated by leader/follower controller
		//
		// return value : function returns true if vehicle should stop
		bool getSpeed( int leader, double sep_dist, double x, double y, double yaw, double & req_speed ) 
		{
			bool stop = false;
			// get distance to leader
            double dist = getDistanceToObstacle( leader, x, y, yaw );

            // get current speed
            double my_speed = alldrp->drp[ myid ].speed;
            // get road speed limit
            double road_speed = myRoad->getSpeed();
            // get leader's speed
			double leader_speed = alldrp->drp[ leader ].speed;

			//TODO: Set req_speed to get your agent within sep_dist of leader and hold that distance
			//req_speed = 0.0;
			
			//simple proportional controller using difference in curent and desired separation distance as error signal
			
			//initialize variables 
			double kp = 0.04; //proportional gain constant 
			double ki = 0;
			double kd = 0;


			double error_p = dist-sep_dist;
			error_i += error_p;
			double error_d = last_error-error_p;
			
			//calculate required speed
			//maybe use my_speed instead of 0.3?
			req_speed = kp*(error_p)+ ki*(error_i) + kd*(error_d);

			// cap req_speed to road speed
			//if ( req_speed > road_speed) {
			//	req_speed = road_speed;
			//}

			//print to display window error signal
			printf("error_p: %f\nerror_p: %f\nerror_p: %f\n",error_p,error_i,error_d);

			// check to see if robot should stop
			if( req_speed < 0.0 || dist < CONVOY_MIN_SEP_DIST || ( ( dist < CONVOY_MIN_SEP_DIST ) && alldrp->drp[ leader ].speed < STOPPED_SPEED ) ){
        		stop = true;
	            req_speed = 0.0;
            }
		
            return stop;
		}
};

int main(int argc, char *argv[])
{
	run = true;

	int robotID = getMyDeadReckID();
	bool local = false;
	int portnum = PLAYER_PORTNUM;
	string mapname = "simville";
	string follow_control_type = "simple";
	int opt;
	int leader = -1;
	double sep_dist = DEFAULT_CONVOY_SEP_DIST;
	bool stop_init = false;
	bool show_path = false;
	bool verbose = false;
	bool use_ranger_avoid = false;
	while( (opt = getopt( argc, argv, "r:svp:ld:f:" ) ) != -1 )
	{
		switch( opt )
		{
			case 'r':  // robot id
				robotID = atoi( optarg );
				break;
			case 's':  // force stop
				stop_init = true;
				break;
			case 'v':  // verbose
				verbose = true;
				break;
			case 'p':  // port number
				portnum = atoi( optarg );
				break;
			case 'l':  // run robot locally (stage simulation)
				local = true;
				break;
			case 'd':  // separation distance
				sep_dist = atof( optarg );
				break;
			case 'f':  // leader robot id
				leader = atoi( optarg );
				break;
			default:
				fprintf( stderr, "Usage: %s [-r robotID] [-f ID of robot to follow] [-d follow distance] [-s (force stop)] [-v (output mapfiles)] [-p portnumber ] [-l (local simulation)]RNDF MDF\n", argv[ 0 ] );
				exit( 0 );
				break;
		}
	}

	//printf( "Argc %d - optind %d\n", argc, optind );
	if( (optind + 1) >= argc )
	{
		fprintf( stderr, "Expected RNDF and MDF arguments after options\n" );
		exit( EXIT_FAILURE );
	}

	// Initialize road network
	Road* myRoad = new Road( verbose, ROAD_SAMPLES_DISTANCE );
	if( !myRoad->initializeRoadNetwork( argv[ optind ], argv[ optind + 1 ] ) )
	{
		printf( "Road failed init!!\n" );
		exit( -2 );
	}

	// Initialize vehicle controller
	BaseController* controller = new Controller< CircleControlPolicy, Road >( myRoad );
	controller->setupControllerFromCmdLine( argc, argv, optind + 2 );

	printf( "Using RobotID %d\n", robotID );

	if( robotID < 0 )
		exit( EXIT_FAILURE );

	bool powerokay = true;

	// Initialize shared memory for receiving states for all robots in simulation
	int shm_id;
	int shmkey = DEADRECK_SHM_KEY + robotID;
	printf( "DR SHM KEY %X\n", shmkey );
	AllDeadReckoningPos * alldrp = initialize_all_deadreck_shared_memory( ALL_DEADRECK_SHM_KEY, &shm_id );
	DeadReckoningPos *my_pos = initialize_deadreck_shared_memory( shmkey, &shm_id );

	int dpID = robotID;
	my_pos->robot_id = dpID;
	my_pos->checkpoint = -1;

	if( dpID < 0 )
	{
		printf( "Invalid DRID %d!!!\n", dpID );
		exit( EXIT_FAILURE );
	}
	if( init_sem( dpID ) != 0 )
		return -1;

	// Initialize low level control
	LLCInterface *llci = new LLCInterfacePlayer( robotID, alldrp, my_pos, local, show_path, use_ranger_avoid, mapname.c_str(), portnum );

	struct sigaction act;
	act.sa_handler = &sig_int;
	sigaction( SIGINT, &act, NULL );

	printf( "Waiting for LLC to start!\n" );

	if( wait_sem() != 0 )
		return -1;

	// Open file for logging robot's path
	char filename[100];
	snprintf( filename, sizeof( filename ), "robot%d-path.dat", robotID );
	FILE *pathfd = fopen( filename, "w" );

	// Robot position in meters
	double x = my_pos->xpos / 1000.0;
	double y = my_pos->ypos / 1000.0;
	double yaw = NORMALIZE( my_pos->yaw );

	printf( "Start Position X %f Y %f Yaw %f\n", x, y, yaw / M_PI * 180.0 );

	// Generate path from current position to road network
	if( !myRoad->generateRoadFromOffNetwork( x, y, M_PI/2.0-yaw ) )
	{
		printf( "No path onto network!!\n" );
		exit( -1 );
	}

	// Get next checkpoint to reach
	my_pos->checkpoint = myRoad->getCurrentCheckpoint();

	// check to see if this robot is a follower
	bool follower = ( leader >= 0 );

	FollowerControlBase *followctrl = NULL;
	if( follower )
	{
		followctrl = new UserFollowerControl();
		followctrl->setAllDRP( alldrp );
		followctrl->setRobotID( dpID );
		followctrl->setRoad( myRoad );
		followctrl->setGroupManager( NULL );
	}

	double req_speed = -1.0;
	double ob_distance[ NUM_DRP ];
	// indices of valid distances
	// This simulation has 4 total robots, but are different robotids between simulation and physical robots.
	int ob_valid_dist[ 4 ];
	bool ob_valid_foundall = false;

	printf( "Looking for robot for separation distance calculations:\n" );
	int ob_valid_index = 0;
	for( int i = 0; i < NUM_DRP; i++ )
	{
		if( alldrp->valid[ i ] )
		{
			ob_valid_dist[ ob_valid_index ] = i;
			ob_valid_index++;
			printf( "Found robotid %d\n", i );
		}
	}
	if( ob_valid_index != 4 )
	{
		printf( "Warning: Not all robots found. Start other robots\n" );
	}
	else
	{
		ob_valid_foundall = true;
	}

	// Simulation update loop
	while( run )
	{
		// Update low level controller
		llci->update( myRoad, controller );

		// Get current position, orientation
		x = my_pos->xpos / 1000.0;
		y = my_pos->ypos / 1000.0;
		yaw = NORMALIZE( my_pos->yaw );
		double yawrate = -1.0 * my_pos->yawrate;

		bool stop = stop_init;
		// Update vehicle state on the road
		if( !myRoad->updatePointIndexAndCheckForNewRoad( x, y, yaw ) )
		{
			// Stop simulation if all checkpoints have been reached
			llci->setSpeed( 0.0 );
			stop = true;
			printf( "\n\nNo More CheckPoints!!!\n\n" );
			break;
		}

		my_pos->checkpoint = myRoad->getCurrentCheckpoint();
		my_pos->next_check = myRoad->getNextCheckpoint();

		// print current state information
		printf( "X:%f Y:%f Yaw:%f YawRate %f Speed: %f\n", x, y, yaw / M_PI * 180.0, yawrate, controller->getSpeed() );

		// Update speed controller
		if( follower )
		{
			if( alldrp->valid[ leader ] > 0 )
			{
				if( followctrl->getSpeed( leader, sep_dist, x, y, yaw, req_speed ) )
				{
					stop = true;
					my_pos->stopped = DR_STOP_LEADCE;
					llci->setSpeed( 0.0 );
				}
			}
			else
			{
				printf( "Leader DRP is INVALID -- Stopping!!\n" );
				stop = true;
				llci->setSpeed( 0.0 );
			}
			controller->Update( x, y, yaw, yawrate, req_speed );
		}
		else
		{
			controller->Update( x, y, yaw, yawrate );
		}

		// Check for stop conditions
		if( llci->getStatus() == LLCI_STOPPING )
			stop = true;

		if( llci->checkSensorsForStop() )
			stop = true;

		// Print control parameters
		printf( "Speed = %f, steer = %f stop is %s powerokay is %s\n", controller->getSpeed(), -1.0 * controller->getSteeringRate(), (stop ? "true" : "false" ), ( powerokay ? "true" : "false" ) );

		// look for new robots
		if( !ob_valid_foundall )
		{
			ob_valid_index = 0;
			for( int i = 0; i < NUM_DRP; i++ )
			{
				if( alldrp->valid[ i ] )
				{
					ob_valid_dist[ ob_valid_index ] = i;
					ob_valid_index++;
					//printf( "Found robotid %d\n", i );
				}
			}
			if( ob_valid_index == 4 )
			{
				ob_valid_foundall = true;
				printf( "Found all robots\n" );
				for( int i = 0; i < 4; i++ )
				{
					printf( "Robot %d\n", i );
				}
			}
		}
		// calculate obstacle distances
		for( int ob = 0; ob < NUM_DRP; ++ob ) 
		{
			double ox = alldrp->drp[ ob ].xpos / 1000.0;
			double oy = alldrp->drp[ ob ].ypos / 1000.0;
			double dist = PathPlan::mc_distance( x, y, ox, oy );
			ob_distance[ ob ] = 0.0;
			if( ob != dpID && alldrp->valid[ ob ] )
			{
				//printf( "\n\n%f DR ID %d dist %f\n\n", get_currenttime(), ob, dist);
				ob_distance[ ob ] = dist;
			} 
		}

		// Send calculated speed and steering rate to low level control
		if( !stop )
		{
			llci->setSpeedAndYawRate( controller, powerokay );
		}

		// Log robot state
		if( ob_valid_foundall )
			fprintf( pathfd, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", get_currenttime(), x, y, yaw, controller->getSpeed(), my_pos->speed, ob_distance[ ob_valid_dist[ 0 ] ], ob_distance[ ob_valid_dist[ 1 ] ], ob_distance[ ob_valid_dist[ 2 ] ], ob_distance[ ob_valid_dist[ 3 ] ] );
		else
			fprintf( pathfd, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", get_currenttime(), x, y, yaw, controller->getSpeed(), my_pos->speed, 0.0, 0.0, 0.0, 0.0 );
		fflush( pathfd );

		if( run && wait_sem() != 0 )
			break;
	}

	// stop robot
	run = false;
	llci->setSpeed( 0.0 );

	delete llci;
	fclose( pathfd );
	return 0;
}



