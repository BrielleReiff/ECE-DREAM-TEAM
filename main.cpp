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

#define MAX_ROBOTS 2

bool run;
void sig_int( int sig )
{
	run = false;
}

int main( int argc, char *argv[] )
{
	run = true;

	int robotID = getMyDeadReckID();
	bool local = false;
	int portnum = PLAYER_PORTNUM;
	string mapname = "simville";
	string follow_control_type = "simple";
	int opt;
	bool stop_init = false;
	bool show_path = false;
	bool verbose = false;
	bool use_ranger_avoid = false;
	bool sign_flag = false;
	bool light_flag = false;
	bool stop_flag = false;
	while( ( opt = getopt( argc, argv, "r:svp:l" ) ) != -1 )
	{
		switch( opt )
		{
			case 'r':
				robotID = atoi( optarg );
				break;
			case 's':
				stop_init = true;
				break;
			case 'v':
				verbose = true;
				break;
			case 'l':  // run robot locally (stage simulation)
				local = true;
				break;
			case 'p':
				portnum = atoi( optarg );
				break;
			default:
				fprintf( stderr, "Usage: %s [-r robotID] [-s (force stop)] [-v (output mapfiles)] [-p portnumber ] [-l (local simulation)] RNDF MDF\n", argv[ 0 ] );
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
		
	// attach to traffic light information
	int tl_shm_id;
	AllTLights *alltraffic = initialize_tlights_shared_memory( TLIGHT_SHM_KEY, &tl_shm_id );

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
	
	double req_speed = -1.0;
	double ob_distance[ NUM_DRP ];
	// indices of valid distances
	// This simulation has 2 total robots, but are different robotids between simulation and physical robots.
	int ob_valid_dist[ MAX_ROBOTS ];
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
	if( ob_valid_index != MAX_ROBOTS )
	{
		printf( "Warning: Not all robots found. Start other robots\n" );
	}
	else
	{
		ob_valid_foundall = true;
	}

	#define UNKNOWN 0
	#define EWL_GREEN 1
	#define EWL_YELLOW 2
	#define EW_GREEN 3
	#define EW_YELLOW 4
	#define ALL_RED_FIRST 5
	#define NS_GREEN 6
	#define NS_YELLOW 7
	#define ALL_RED_SECOND 8
	bool continue_flag = true;

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

		// reset the required speed
		req_speed = -1;

		//factor for converting counts to time, taking into account simulation speed
		#define SIMT3 0
		#define SIMT5 1
		#define SIMT1 2

		int current_simt = SIMT5;

		float factor;

		switch(current_simt)
		{
			case(SIMT3):
				factor = 0.03;
				break;
			case(SIMT5):
				factor = 0.05;
				break;
			case(SIMT1):
				factor =  0.1;
				break;
		}

		float nec_vel;
		int stopline_min_prev; 

		// Look for traffic lights
		vector<TLight*> lights = getValidTLights( alltraffic );
		for( vector<TLight*>::iterator iter = lights.begin(); iter != lights.end(); iter++ )
		{
			TLight* traffic = ( *iter );
			double dist_to_center = PathPlan::mc_distance( traffic->center_x, traffic->center_y, x, y );
			printf( "Traffic Light ID %d center is %f m away\n", traffic->tLight_ID, dist_to_center );
			// TODO: 
			// 1) detect which stop line is in your lane
			// 2) if you're approaching the traffic light calculate the robot speed to hit the stop line when the light status is GREEN_LIGHT
			// 3) if you don't make the green phase, stop at the line if light status is RED_LIGHT or YELLOW_LIGHT
			//concerned with status?
			int current_state = UNKNOWN; 

			if (dist_to_center < 4 && continue_flag==true)
			{
				float stopline_min = 100;
				int stopline; 
				int flag;

				for( int i = 0; i < traffic->valid_stops; i++ )
				{
					double stopdist = PathPlan::mc_distance( traffic->x_stop[ i ], traffic->y_stop[ i ], x, y );
					// traffic->status[ i ] can equal RED_LIGHT, YELLOW_LIGHT, GREEN_LIGHT
					// traffic->timer[ i ] will give you the time remaining at the current phase for green, yellow, and all red states. You will need to check other
					//     light directions to see how much time is remaining in red. Units are in 0.1 seconds
					// set req_speed = 0 if the robot should stop
					// set req_speed = any other value < 0.5 m/s to drive at that speed			
					if (stopdist < stopline_min)
					{

						stopline_min = stopdist;
						if(stopline_min>stopline_min_prev)
						{
							continue_flag==false;
							break;
						}
						stopline = i;
						stopline_min_prev=stopline_min;
					}	
				}
				if(continue_flag==false) break;
				printf("Closest stopline %d is %f m away\n",stopline,stopline_min);
			
				//light decisions
				if (traffic->status[stopline] == GREEN_LIGHT)
				{
					//is there enough time to make this green?
					//necessary velocity to make green
					nec_vel = (stopline_min - 0.4)/(traffic->timer[stopline]*factor);
					if (nec_vel < 0.25)//go default speed
					{
						nec_vel = 0.25;
					} 
					else if(nec_vel > 0.5)//go slow enough to catch next green
					{
						nec_vel = (stopline_min - 0.4)/(traffic->timer[stopline]*factor + 16.5);
					}
					else //speed up to catch green
					{
						nec_vel = (stopline_min - 0.4)/(traffic->timer[stopline]*factor);
					}
					current_state = EW_GREEN;
				}
				else if (traffic->status[stopline] ==YELLOW_LIGHT)
				{
					nec_vel = (stopline_min - 0.4)/(traffic->timer[stopline]*factor + 15);
					current_state = EW_YELLOW;
				}

				else //traffic light red
				{
					int prev_state = current_state;

					if((traffic->timer[0]==traffic->timer[1]) && (traffic->timer[0]==traffic->timer[2]) && (traffic->timer[0]==traffic->timer[3]) && (traffic->timer[0]==traffic->timer[4]) && (traffic->timer[0]==traffic->timer[5])) 
					{
						switch(prev_state)
						{							
							case(EW_YELLOW):
								current_state = ALL_RED_FIRST;		
								break;
							case(NS_YELLOW):
								current_state = ALL_RED_SECOND;
								break;
							default:
								current_state = prev_state;
								break;
						}
					}
					else
					{
						if (traffic->status[2] == GREEN_LIGHT)
						{
							current_state = EW_GREEN;
						}
						else if(traffic->status[2] == YELLOW_LIGHT)
						{
							current_state = EW_YELLOW;
						}
						else if(traffic->status[1] == GREEN_LIGHT)
						{
							current_state = NS_GREEN;
						}
						else if(traffic->status[1] == YELLOW_LIGHT)
						{
							current_state = NS_YELLOW;
						}
					}
					//slow lough it
					switch (current_state)
					{
						case(ALL_RED_FIRST):
						nec_vel = (stopline_min - 0.4)/(traffic->timer[stopline]*factor + 7.5);
						break;

						case(ALL_RED_SECOND):
						nec_vel = (stopline_min - 0.4)/(traffic->timer[stopline]*factor);
						break; 

						case(EW_GREEN):
						nec_vel = (stopline_min - 0.4)/(traffic->timer[2]*factor + 10);
						break; 

						case(EW_YELLOW):
						nec_vel = (stopline_min - 0.4)/(traffic->timer[2]*factor + 8);
						break; 

						case(NS_GREEN):
						nec_vel = (stopline_min - 0.4)/(traffic->timer[1]*factor + 2.5);
						break; 

						case(NS_YELLOW):
						nec_vel = (stopline_min - 0.4)/(traffic->timer[1]*factor + 0.5);
						break;

					}
				}
			req_speed = nec_vel;
			}
			
			else
			{
				current_state = UNKNOWN;
			}
			printf("continue flag = %d\n",(int)continue_flag);
			printf("current state = %d\n", current_state);
		}

		// Look for new robots in the simulation
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
			if( ob_valid_index == MAX_ROBOTS )
			{
				ob_valid_foundall = true;
				printf( "Found all robots\n" );
				for( int i = 0; i < MAX_ROBOTS; i++ )
				{
					printf( "Robot %d\n", i );
				}
			}
		}

		// calculate obstacle distances
		for( int ob = 0; ob < NUM_DRP; ob++ ) 
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
		
		// Check the other robot's location
		if( ob_valid_foundall )
		{
			for( int i = 0; i < MAX_ROBOTS; i++ )
			{
				int ob_id = ob_valid_dist[ i ];
				double ob_dist = ob_distance[ ob_id ];
				double ob_x = alldrp->drp[ ob_id ].xpos / 1000.0;
				double ob_y = alldrp->drp[ ob_id ].ypos / 1000.0;
				double ob_v = alldrp->drp[ ob_id ].speed;
	
				// one of the robots is this robot
				if( ob_dist != 0 )
				{
					// TODO:
					// 1) check to see if you're approaching the stop sign intersection
					// Determine distance to stopline from each robot
					double ob0_x = 7.348;
					double ob0_y = -0.779;	
					double ob1_x = 6.896;
					double ob1_y = 0.519;					

					double dist0_stop = pow(pow(x - ob0_x,2) + pow(y-ob0_y,2),0.5);
					double dist1_stop = pow(pow(ob_x - ob1_x,2) + pow(ob_y-ob1_y,2),0.5);
					
					// Determine dot product of robot and stopsign center

					bool my_pres = true;

					double stop_x =  7.271;
					double stop_y = -0.124;					

					double A1 = stop_x - x;
					double A2 = stop_y - y;
					double new_yaw = -yaw+M_PI/2;
					if (new_yaw < 0)
						new_yaw = new_yaw + 2*M_PI;

					double dot_prod = (A1*cos(new_yaw)+A2*sin(new_yaw));

					//driving toward intersection
					if(dist0_stop < 1.0 && dot_prod > 0 && sign_flag == false)
					{
					// 2) check to see if another robot has precedence at the intersection
						if (dist1_stop < 1.0 && ob_v < 0.1)
							my_pres = false; //Car1 has pres
											
					// 3) stop once you hit your stop line
						if (dist0_stop < 0.4){
							req_speed = 0;
						}
						
					// 4) if you have precedence, remain stopped for 2.0 sec before continuing
						if (my_pres == true && stop_flag == true){
							sleep(2);
							stop_flag = false; 
							sign_flag = true;
							req_speed = -1;
						}
					// 5) if you don't have precedence, remain stopped until the other robot has cleared the intersection
						else if (my_pres == false && stop_flag == true && dist1_stop < 1.0){
							//stop_flag = false; 
							//sign_flag = true;
							req_speed = 0;
						}

						//throw stop flag if too close to line
						if (dist0_stop < 0.4){
							stop_flag = true;
						}
					}

					if (sign_flag == true && dist0_stop > 1.5){
						sign_flag = false;
						stop_flag = false;
					}
						
							
					// 6) check if you're too close to the robot as you're following it
						
					// set req_speed = 0 if need to stop
					// set req_speed = -1 if to continue at the normal speed
					// set req_speed = any other value < 0.5 m/s to drive at that speed
					double A12 = ob_x - x;
					double A22 = ob_y - y;
					double new_yaw2 = -yaw+M_PI/2;
					if (new_yaw2 < 0)
						new_yaw2 = new_yaw2 + 2*M_PI;

					double dot_prod2 = (A12*cos(new_yaw2)+A22*sin(new_yaw2));

					// one of the robots is this robot
					if( ob_dist != 0 && dot_prod2 > 0.0)
					{
						if( ob_dist <= 0.5 ){
							req_speed = 0.0;
						}
						else if( ob_dist <= 1.0 && req_speed != 0.0){
							//req_speed = ob_v;
						}

					}	
				
				}
			}
		}


		// Update speed controller
		controller->Update( x, y, yaw, yawrate, req_speed );

		// Check for stop conditions
		if( llci->getStatus() == LLCI_STOPPING )
			stop = true;

		if( llci->checkSensorsForStop() )
			stop = true;

		// Print control parameters
		printf( "Speed = %f, steer = %f stop is %s powerokay is %s\n", controller->getSpeed(), -1.0 * controller->getSteeringRate(), (stop ? "true" : "false" ), ( powerokay ? "true" : "false" ) );

		// Send calculated speed and steering rate to low level control
		if( !stop )
		{
			llci->setSpeedAndYawRate( controller, powerokay );
		}

		// Log robot state
		fprintf( pathfd, "%f\t%f\t%f\t%f\t%f\t%f\n", get_currenttime(), x, y, yaw, controller->getSpeed(), my_pos->speed );
		fflush( pathfd );

		if( run && wait_sem() != 0 )
			break;
			
		usleep( 100000 );
	}

	// stop robot
	run = false;
	llci->setSpeed( 0.0 );

	delete llci;
	fclose( pathfd );
	return 0;
}