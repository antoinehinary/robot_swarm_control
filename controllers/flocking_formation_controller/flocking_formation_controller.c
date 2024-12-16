/*****************************************************************************/
/* File:         flocking_formation_controller.c                             */
/* Version:      1.0                                                         */
/* Date:         8-Nov-24                                                    */
/* Description:  Reynolds flocking control                                   */
/*                                                                           */
/* Author:       8-Nov-24 by Antoine Hinary                                  */
/* Last revision:                                                            */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>

#define V_MAX 			0.1288  // Maximum speed of a robot
#define NB_SENSORS      8       // Number of distance sensors
#define MIN_SENS        60      // Minimum sensibility value
#define MAX_SENS        250     // Maximum sensibility value
#define MAX_SPEED       700     // Maximum speed
#define MAX_SPEED_WEB   6.28    // Maximum speed webots
#define FLOCK_SIZE      5       // Size of flock
#define TIME_STEP       64      // [ms] Length of time step
#define AXLE_LENGTH     0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS    0.0205  // Wheel radius (meters)
#define DELTA_T         0.064   // Timestep (seconds)
#define RULE1_THRESHOLD 0.2    	// Threshold to activate aggregation rule. default 0.20
#define RULE2_THRESHOLD 0.15    // Threshold to activate dispersion rule. default 0.15
#define MIGRATION_WEIGHT 0.001	// Wheight of attraction towards the common goal. default 0.01/10
#define MIGRATORY_URGE 	1       // Tells the robots if they should move towards a specific migratory direction
#define NEIGHBOURHOOD 	1       // Tells the robot considering neighbors or all robots during flocking
#define NEIGH_THRESHOLD 0.5     // Threshold to consider neighbourhood
#define INTER_VEHICLE_COM 1     // Set 1 if there is intervehicle communication
#define VERBOSE 		0		// Set 1 to print information about the controller
#define DEBUG	 		0		// Set 1 to print information to debug the controller
#define DATA_MESSAGE 	1		// To send data
#define WEIGHTS_MESSAGE 2		// To send weights
#define DATASIZE 		4		// Size of data array per particle
#define ARRIVAL_THRESHOLD 0.4
#define SWITCH_MIN_THRESHOLD 0.3
#define SWITCH_MAX_THRESHOLD 1.5
#define EXIT_THRESHOLD 2.5

// Function definitions
#define ABS(x) ((x>=0)?(x):-(x))									// Compute the absolute value of x
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)	// Convertion from radians to degrees

// Define to select wether to use IMU or not
#define USE_IMU

// PSO Reynold rules definitions
double RULE1_WEIGHT=0.6/10;		// Weight of aggregation rule. default 0.6/10
double RULE2_WEIGHT=0.02/10;	// Weight of dispersion rule. default 0.02/10
double RULE3_WEIGHT=1.0/10;		// Weight of alignment rule. default 1.0/10
double DISTANCE_ROBOT=0.1;		//separation distance between robots

// Webits device tags
WbDeviceTag left_motor; 		// Handle for left wheel of the robot
WbDeviceTag right_motor;		// Handle for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS]; 	// Handle for the infrared distance sensors
WbDeviceTag receiver;     		// Handle for the receiver node
WbDeviceTag emitter;      		// Handle for the emitter node
WbDeviceTag gps;      			// Handle for the gps node
WbDeviceTag imu;     			// Handle for the imu node

char Controller[8] = "reynold"; 		// Control mode
int robot_id_u, robot_id; 				// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID
float loc[FLOCK_SIZE][3];				// X, Y, Theta of all robots
float prev_loc[FLOCK_SIZE][3]; 			// Previous X, Y, Theta values
float speed[FLOCK_SIZE][2]; 			// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE]; 			// != 0 if initial positions have been received
float migr[2] = {0.8, 1.6}; 			// Initial Migration vector
float final_migration[2] = {4.2, 1.7}; 	// Final Migration vector
double X_next[FLOCK_SIZE][2];			// Next X position computed for Laplace control
int message_type;

float arrived[FLOCK_SIZE] = {0.0,0.0,0.0,0.0,0.0};	// 1 if robot has arrived at 0.4 switching to laplace
int e_puck_matrix[16] = {50,35,20,0,0,-20,-35,-45,-45,-35,-20,0,0,20,35,50}; // Weights for neurlal network control

// Laplace matrix
double L[FLOCK_SIZE][FLOCK_SIZE] = {
	{2,  -1, 0,  0,  -1},
	{ -1, 2,  -1, 0,  0},
	{ 0, -1,  1,  0,  0},
	{ 0,  0, 0,  1, -1},
	{ -1,  0,  0, -1,  2}
};


/*
 * Reset the robot's devices and get its ID
 *
 */
static void reset() {
    // Initialize robot
    wb_robot_init();

    // Device initialization
    receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");
    gps = wb_robot_get_device("gps");
    imu = wb_robot_get_device("inertial_unit");
    wb_gps_enable(gps, TIME_STEP);
    wb_inertial_unit_enable(imu, TIME_STEP);

    // Motor initialization
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    // Distance sensor initialization
    char sensor_name[4] = "ps0";
    for (int i = 0; i < NB_SENSORS; i++) {
        ds[i] = wb_robot_get_device(sensor_name);
        wb_distance_sensor_enable(ds[i], TIME_STEP);
        sensor_name[2]++;
    }

    // Receiver setup
    wb_receiver_enable(receiver, TIME_STEP);

    // Get robot ID from name
    char *robot_name = (char *)wb_robot_get_name();
    sscanf(robot_name, "epuck%d", &robot_id_u);
    robot_id = robot_id_u % FLOCK_SIZE;

    // Initialize position tracking
    memset(initialized, 0, sizeof(initialized)); // All robots not initialized
    memset(loc, 0, sizeof(loc));
    memset(prev_loc, 0, sizeof(prev_loc));
	#if VERBOSE
    	printf("Robot %d reset completed.\n", robot_id_u);
	#endif
}


/*
 * Update speed according to Reynold's rules
 */
void reynolds_rules() {
    float avg_loc[2] = {0, 0}; // Flock average positions
    float avg_speed[2] = {0, 0}; // Flock average speeds
    float cohesion[2] = {0, 0};
    float dispersion[2] = {0, 0};
    float alignment[2] = {0, 0};
    float dist = 0; // Use it to define distance between robots
    float n_robots = 1; // Use to assign initial number of robots

	#ifdef NEIGHBOURHOOD
		for (int i = 0; i < FLOCK_SIZE; i++) {
			if (i == robot_id) {
				continue; // Skip self
			}
			dist = sqrt(pow(loc[i][0] - loc[robot_id][0], 2.0) + pow(loc[i][1] - loc[robot_id][1], 2.0));
			if (dist < NEIGH_THRESHOLD) { // Check if within neighborhood
				for (int j = 0; j < 2; j++) {
					avg_speed[j] += speed[i][j];
					avg_loc[j] += loc[i][j];
				}
				n_robots++;
			}
		}
	#else
		for (int i = 0; i < FLOCK_SIZE; i++) {
			if (i == robot_id) {
				continue; // Skip self
			}
			for (int j = 0; j < 2; j++) {
				avg_speed[j] += speed[i][j];
				avg_loc[j] += loc[i][j];
			}
		}
		n_robots = FLOCK_SIZE; // Count all robots as neighbors
	#endif

	// Compute averages
	for (int j = 0; j < 2; j++) {
		avg_speed[j] /= (n_robots - 1); // Avoid self in the average
		avg_loc[j] /= (n_robots - 1);   // Avoid self in the average
	}


    /* Reynold's rules */
    // Rule 1 - Cohesion
    for (int j = 0; j < 2; j++) {
        if (sqrt(pow(loc[robot_id][0] - avg_loc[0], 2) + pow(loc[robot_id][1] - avg_loc[1], 2)) > RULE1_THRESHOLD) {
            cohesion[j] = avg_loc[j] - loc[robot_id][j];
        }
    }

    // Rule 2 - Dispersion
    for (int k = 0; k < FLOCK_SIZE; k++) {
        if (k != robot_id) {
            if (pow(loc[robot_id][0] - loc[k][0], 2) + pow(loc[robot_id][1] - loc[k][1], 2) < RULE2_THRESHOLD) {
                for (int j = 0; j < 2; j++) {
                    dispersion[j] += 1 / (loc[robot_id][j] - loc[k][j]);
                }
            }
        }
    }

    // Rule 3 - Alignment
    for (int j = 0; j < 2; j++) {
        alignment[j] = avg_speed[j] - speed[robot_id][j];
    }

    for (int j = 0; j < 2; j++) {
        speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
        speed[robot_id][j] += dispersion[j] * RULE2_WEIGHT;
        speed[robot_id][j] += alignment[j] * RULE3_WEIGHT;
    }

    #ifdef MIGRATORY_URGE
		speed[robot_id][0] += MIGRATION_WEIGHT * (migr[0] - loc[robot_id][0]);
		speed[robot_id][1] += MIGRATION_WEIGHT * (migr[1] - loc[robot_id][1]);
    #endif
}

void multiply_matrix_vector(double mat[FLOCK_SIZE][FLOCK_SIZE], double vec[FLOCK_SIZE][2], double result[FLOCK_SIZE][2]) {
	for (int i = 0; i < FLOCK_SIZE; i++) {
		for (int j = 0; j < 2; j++) {
			result[i][j] = 0.0;
			for (int k = 0; k < FLOCK_SIZE; k++) {
				result[i][j] += mat[i][k] * vec[k][j];
			}
		}
	}
}

/*
 * Keep given float number within interval {-limit, limit}
 */
void limitf(float *number, double limit) {

	if (*number > limit)
		*number = (float)limit;
	if (*number < -limit)
		*number = (float)-limit;
}

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {

	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

// Main Laplacian controller
void laplacian_rules(int *msl, int *msr) {
    double VGx = 0.18, VGy = 0.0;  // Group migration velocity
    float x, y;

    // Desired positions (straight line configuration)
    double b[FLOCK_SIZE][2] = {
        {0.4, 0},
        {0.6, 0},
        {0.8, 0},
        {0.0, 0},
        {0.2, 0}
    };

    // Save current positions
    for (int i = 0; i < FLOCK_SIZE; i++) {
        for (int j = 0; j < 2; j++) {
            X_next[i][j] = loc[i][j];
        }
    }

    // Calculate delta positions
    double delta_pos[FLOCK_SIZE][2];
    for (int i = 0; i < FLOCK_SIZE; i++) {
        for (int j = 0; j < 2; j++) {
            delta_pos[i][j] = loc[i][j] - b[i][j];
        }
    }

    // Apply Laplacian feedback control
    double LX[FLOCK_SIZE][2];
    multiply_matrix_vector(L, delta_pos, LX);

    // Update positions using Laplacian control
    for (int i = 0; i < FLOCK_SIZE; i++) {
        for (int j = 0; j < 2; j++) {
            X_next[i][j] -= LX[i][j];  // Correct positions
        }
    }

    // Add constant velocity for group migration
    for (int i = 0; i < FLOCK_SIZE; i++) {
        X_next[i][0] += VGx;
        X_next[i][1] += VGy;
    }

    // Compute control input for the current robot
    x = X_next[robot_id][0] - loc[robot_id][0];
    y = 1.58 - loc[robot_id][1];

    // Parameters for proportional control
    float Ku = 0.15;  // Forward control coefficient
	float Kw = 0.15;  // Rotational control coefficient

    // Compute the distance (range) and angle (bearing) to the target
    float range = sqrtf(x * x + y * y);
    float dersired_bearing = atan2f(y, x);
    float bearing = loc[robot_id][2] - dersired_bearing;

    // Compute forward and rotational speeds
    float u = Ku * range * cosf(bearing);
    float w = Kw * bearing;

    // Convert to wheel speeds
    *msl = (u - AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
    *msr = (u + AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);

    // Apply limits to avoid excessive speeds
    limit(msl, MAX_SPEED);
    limit(msr, MAX_SPEED);

	#if DEBUG
    	// Debugging information
		printf("Robot: %d, X : %f, Y : %f\n", robot_id, X_next[robot_id][0], X_next[robot_id][1]);
		printf("Range: %f, U: %f, Bearing: %f, W: %f\n", range, u, radToDeg(bearing), w);
	#endif
    
}

/*
 * Updates robot position with wheel speeds
 * Used for odometry
 */
void update_self_motion(int msl, int msr) {

	float theta = loc[robot_id][2];

	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;

	// Compute deltas in the environment
	float dx = du * cosf(theta);
	float dy = du * sinf(theta);

	// Update position
	loc[robot_id][0] += dx;
	loc[robot_id][1] += dy;
	loc[robot_id][2] += dtheta;

	// Keep orientation within 0, 2pi
	if (loc[robot_id][2] > 2*M_PI) loc[robot_id][2] -= 2.0*M_PI;
	if (loc[robot_id][2] < 0) loc[robot_id][2] += 2.0*M_PI;
}

/*
 * Computes wheel speed given a certain X,Y speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
	float x = speed[robot_id][0]*cosf(loc[robot_id][2]) + speed[robot_id][1]*sinf(loc[robot_id][2]);  // x in robot coordinates
	float y = -speed[robot_id][0]*sinf(loc[robot_id][2]) + speed[robot_id][1]*cosf(loc[robot_id][2]); // y in robot coordinates

	float Ku = 0.2;   				// Forward control coefficient
	float Kw = 0.5;  				// Rotational control coefficient
	float range = sqrtf(x*x + y*y);	// Distance to the wanted position
	float bearing = atan2(y, x);	// Orientation of the wanted position
	
	float u = Ku*range*cosf(bearing);// Compute forward control
	float w = Kw*bearing;			 // Compute rotational control
	
	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);

	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);

	#if DEBUG
		printf("bearing: %f, range: %f\n",bearing * (180.0 / M_PI), range);
	#endif

}

void update_position() {
    const double *gps_values = wb_gps_get_values(gps);

    #ifdef USE_IMU
        const double *imu_values = wb_inertial_unit_get_roll_pitch_yaw(imu);

        // Update position with IMU
		prev_loc[robot_id][0] = loc[robot_id][0];
		prev_loc[robot_id][1] = loc[robot_id][1];
		prev_loc[robot_id][2] = loc[robot_id][2];
        loc[robot_id][0] = gps_values[0];
        loc[robot_id][1] = gps_values[1];
        loc[robot_id][2] = imu_values[2]; // Use yaw from IMU
    #else
        // Alternative method: estimate angle using odometry
        float dx = gps_values[0] - prev_loc[robot_id][0];
        float dy = gps_values[1] - prev_loc[robot_id][1];
        loc[robot_id][2] = atan2(dy, dx); // Compute angle based on movement

        // Update position
        loc[robot_id][0] = gps_values[0];
        loc[robot_id][1] = gps_values[1];
    #endif

    // Normalize orientation to [0, 2*PI]
    if (loc[robot_id][2] > 2 * M_PI)
        loc[robot_id][2] -= 2 * M_PI;
    if (loc[robot_id][2] < 0)
        loc[robot_id][2] += 2 * M_PI;

	#if DEBUG
	    printf("Robot %d position updated to (%f, %f, %f)\n",
           robot_id, loc[robot_id][0], loc[robot_id][1], loc[robot_id][2]);
	#endif
}

// Helper function to count robots meeting a condition
int count_robots_meeting_condition(bool (*condition)(int)) {
    int count = 0;
    for (int i = 0; i < FLOCK_SIZE; i++) {
        if (condition(i)) {
            count++;
        }
    }
    return count;
}

/*
	Checck if robots arrived at first checkpoint
*/
bool attained_first_checkpoint(int id) {
    return loc[id][0] > ARRIVAL_THRESHOLD;
}

/*
	Checck if robots arrived at first checkpoint and within switch range
*/
bool is_within_switch_range(int id) {
    return loc[id][0] > SWITCH_MIN_THRESHOLD && loc[id][0] < SWITCH_MAX_THRESHOLD;
}
/*
	Checck if robots arrived at second checkpoint
*/
bool attained_second_checkpoint(int id) {
    return loc[id][0] > EXIT_THRESHOLD;
}

/*
	Update controller state based on robot positions
*/
void update_controller_state() {
    // Mark the current robot as arrived if it meets the arrival condition
    if (attained_first_checkpoint(robot_id)) {
        arrived[robot_id] = 1;
    }

    // Count robots in switch range
    int arrived_count = count_robots_meeting_condition(is_within_switch_range);
    if (arrived_count == FLOCK_SIZE && strcmp(Controller, "reynold") == 0) {
        strcpy(Controller, "laplace");
        #if VERBOSE
			printf("Switched to Laplace\n");
		#endif
    }

    // Count robots that have exited
    int exited_count = count_robots_meeting_condition(attained_second_checkpoint);
    if (exited_count == FLOCK_SIZE && strcmp(Controller, "laplace") == 0) {
        strcpy(Controller, "reynold");
		#if VERBOSE
        	printf("Switched to Reynold, new migration point: %f, %f\n", migr[0], migr[1]);
		#endif
    }
}

/*
 * Initialize robot's position
 */
void initial_pos(void){

	char *inbuffer;	
	int rob_nb;
	float rob_x, rob_y, rob_theta; // Robot position and orientation
	
	while (initialized[robot_id] == 0) {
		
		// wait for message
		while (wb_receiver_get_queue_length(receiver) == 0)	wb_robot_step(TIME_STEP);
		
		inbuffer = (char*) wb_receiver_get_data(receiver);
		sscanf(inbuffer,"%d#%f#%f#%f##%f#%f#%d", &rob_nb,&rob_x,&rob_y,&rob_theta, &migr[0], &migr[1],&message_type);

    	// robot_nb %= FLOCK_SIZE;
		if (rob_nb == robot_id) {
			// Initialize self position	
			loc[rob_nb][0] = rob_x; 		// x-position
			loc[rob_nb][1] = rob_y; 		// y-position
			loc[rob_nb][2] = rob_theta; 	// theta
			prev_loc[rob_nb][0] = loc[rob_nb][0]; // previous x-position 
			prev_loc[rob_nb][1] = loc[rob_nb][1]; // previous y-position 
			initialized[rob_nb] = 1; 		// initialized = true

			#if VERBOSE
				printf("Robot %d initialized with message type %d\n", robot_id, message_type);
			#endif
		}	

		wb_receiver_next_packet(receiver);
	}
}


/*
 * Main function
 */
int main(){ 	

	int i;							// Counter
	int rob_nb;						// Robot number
	int max_sens=0;					// Store highest sensor value
	char *inbuffer;					// Buffer for the receiver node
	int msl=0, msr=0;				// Wheel speeds
	float msl_w, msr_w; 			// Wheel speeds in webots
	char outbuffer[255];			// Buffer for the emitter node
	float first_checkpoint;			// Flag for robot arrival at the initial migration point
	int distances[NB_SENSORS];		// Array for the distance sensor readings
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	float rob_x, rob_y, rob_theta;  // Robot position and orientation

 	reset();						// Resetting the robot
	initial_pos();					// Initializing the robot's position

	strncpy(outbuffer, " ", 1);		// dummy assignment
	
	// Forever Loop
	for(;;){
		bmsl = 0; bmsr = 0; sum_sensors = 0; max_sens = 0; 	// Reset sensor values
	
		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) {
			//Read sensor values
			distances[i]=wb_distance_sensor_get_value(ds[i]); 
			
			// Add up sensor values
			sum_sensors += distances[i]; 	

			// Check if new highest sensor value					
			max_sens = max_sens>distances[i]?max_sens:distances[i]; 

			// Weighted sum of distance sensor values for Braitenberg vehicle
			bmsr += e_puck_matrix[i] * distances[i];
			bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
		}

		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;

		/* Get information from other robots*/
		int count = 0;
		while (wb_receiver_get_queue_length(receiver) > 0 && count < FLOCK_SIZE) 
		{
			inbuffer = (char*) wb_receiver_get_data(receiver);
			sscanf(inbuffer,"%d#%f#%f#%f#%f#%d",&rob_nb,&rob_x,&rob_y,&rob_theta, &first_checkpoint, &message_type);
			
			if (message_type == WEIGHTS_MESSAGE) {
				RULE1_WEIGHT = rob_x;
				RULE2_WEIGHT = rob_y;
				RULE3_WEIGHT = rob_theta;
				#if DEBUG
					printf("Robot %d received weights : %f, %f, %f\n", robot_id, RULE1_WEIGHT, RULE2_WEIGHT, RULE3_WEIGHT);
				#endif
			}

			rob_nb %= FLOCK_SIZE;
			if (initialized[rob_nb] == 0) {
				// Get initial positions
				#if DEBUG
					printf("Robot %d got initial position robot[%d] = (%f,%f)\n",robot_id, rob_nb,rob_x,rob_y);
				#endif

				loc[rob_nb][0] = rob_x; 	//x-position
				loc[rob_nb][1] = rob_y; 	//y-position
				loc[rob_nb][2] = rob_theta; //theta
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				initialized[rob_nb] = 1;
			} else if(message_type==DATA_MESSAGE) {
				// Get position update
				#if DEBUG
					printf("\n Robot [%d] got update robot[%d] = (%f,%f) \n",robot_id, rob_nb,loc[rob_nb][0],loc[rob_nb][1]);
				#endif

				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				loc[rob_nb][0] = rob_x; 	//x-position
				loc[rob_nb][1] = rob_y; 	//y-position
				loc[rob_nb][2] = rob_theta; //theta
			}
			
			speed[rob_nb][0] = (1/DELTA_T)*(loc[rob_nb][0]-prev_loc[rob_nb][0]);
			speed[rob_nb][1] = (1/DELTA_T)*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
			
			arrived[rob_nb] = first_checkpoint;
			count++;

			wb_receiver_next_packet(receiver);
		}

		// Compute self position & speed
		prev_loc[robot_id][0] = loc[robot_id][0];
		prev_loc[robot_id][1] = loc[robot_id][1];

		update_position();				// Update robot's position
		update_self_motion(msl,msr);	// Update self motion used for Odometry

		speed[robot_id][0] = (1/DELTA_T)*(loc[robot_id][0]-prev_loc[robot_id][0]);
		speed[robot_id][1] = (1/DELTA_T)*(loc[robot_id][1]-prev_loc[robot_id][1]);
		
		update_controller_state(); 	// Update controller state

		// Controller logic
		if (strcmp(Controller, "reynold") == 0) {
			
			reynolds_rules();					// Apply Reynold's rules
			compute_wheel_speeds(&msl, &msr);	// Compute wheels speed from Reynold's speed

		} else if (strcmp(Controller, "laplace") == 0) {

			migr[0] = 5; 	// New x-value for migration vector
			migr[1] = 1.5; 	// New y-value for migration vector

			laplacian_rules(&msl, &msr); 	// Apply Laplace's rules
			
			#if DEBUG
				printf("Applying Laplacian rules\n");
			#endif
		}

		if(strcmp(Controller, "reynold") == 0){
			// Adapt speed instinct to distance sensor values
			if (sum_sensors > NB_SENSORS*MIN_SENS) {
				msl -= msl*max_sens/(2*MAX_SENS);
				msr -= msr*max_sens/(2*MAX_SENS);
			}
				// Add Braitenberg
				msl += bmsl;
				msr += bmsr;
		}	

		// Set speed to webots speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;

		// Set speed according to motor limits
		limitf(&msl_w, MAX_SPEED_WEB);
		limitf(&msr_w, MAX_SPEED_WEB);

		int final_count = 0;
		for (int i = 0; i < FLOCK_SIZE; i++) {
			double final_dist = sqrt((loc[i][0] - final_migration[0]) * (loc[i][0] - final_migration[0]) + (loc[i][1] - final_migration[1]) * (loc[i][1] - final_migration[1]));
			if (final_dist < 0.5) {
				final_count++;
			}
		}

		if (final_count == FLOCK_SIZE) {
			msl_w = 0;
			msr_w = 0;

			#if DEBUG
				printf("All robots have reached the final destination\n");
			#endif
		}

		// Send velocity command
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);

		// Send current position to neighbors
		if (INTER_VEHICLE_COM) {
			sprintf(outbuffer,"%1d#%f#%f#%f#%f#%1d",robot_id,loc[robot_id][0],loc[robot_id][1], loc[robot_id][2], first_checkpoint, DATA_MESSAGE);
			wb_emitter_send(emitter,outbuffer,strlen(outbuffer));
		}

		// Advance one step
		wb_robot_step(TIME_STEP);
	}
}  

