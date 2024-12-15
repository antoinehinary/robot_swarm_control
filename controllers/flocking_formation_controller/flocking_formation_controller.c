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
#include <webots/gyro.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>

#define V_MAX 0.1288          // Maximum speed of a robot
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
#define RULE1_THRESHOLD 0.2    // Threshold to activate aggregation rule. default 0.20

#define RULE2_THRESHOLD 0.15    // Threshold to activate dispersion rule. default 0.15


#define MIGRATION_WEIGHT (0.01/10)// Wheight of attraction towards the common goal. default 0.01/10
#define MIGRATORY_URGE 1         // Tells the robots if they should just go forward or move towards a specific migratory direction
#define NEIGHBOURHOOD 1          // Tells the robot considering neighbors or all robots during flocking
#define NEIGH_THRESHOLD 0.5      // Threshold to consider neighbourhood
#define INTER_VEHICLE_COM 1      // Set 1 if there is intervehicle communication
#define VERBOSE 0
#define ABS(x) ((x>=0)?(x):-(x))



double RULE1_WEIGHT=0.6/10;// Weight of aggregation rule. default 0.6/10
double RULE2_WEIGHT=0.02/10;// Weight of dispersion rule. default 0.02/10
double RULE3_WEIGHT=1.0/10;// Weight of alignment rule. default 1.0/10
double DISTANCE_ROBOT=0.1;		 //separation distance between robots

#define DATASIZE 4		  // Size of data array per particle


#define USE_IMU

WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS]; // Handle for the infrared distance sensors
WbDeviceTag receiver;     // Handle for the receiver node
WbDeviceTag emitter;      // Handle for the emitter node
WbDeviceTag gps;      // Handle for the gps node
WbDeviceTag imu;      // Handle for the imu node

char Controller[8] = "reynold"; // Control mode
int e_puck_matrix[16] = {50,35,20,0,0,-20,-35,-45,-45,-35,-20,0,0,20,35,50}; // Custom
int robot_id_u, robot_id; // Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID
float loc[FLOCK_SIZE][3]; // X, Y, Theta of all robots
float prev_loc[FLOCK_SIZE][3]; // Previous X, Y, Theta values
float speed[FLOCK_SIZE][2]; // Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE]; // != 0 if initial positions have been received
float migr[2] = {0.8, 1.6}; // Migration vector
int arrived[FLOCK_SIZE] = {0,0,0,0,0}; // 1 if robot has arrived at 0.4 switching to laplace
double z_ang_vel;
double X_next[FLOCK_SIZE][2];

// Laplace matrix
double L[FLOCK_SIZE][FLOCK_SIZE] = {
	{ 1, -1,  0,  0,  0},
	{-1,  2, -1,  0,  0},
	{ 0, -1,  2, -1,  0},
	{ 0,  0, -1,  2, -1},
	{ 0,  0,  0, -1,  1}
};

/*
 * Reset the robot's devices and get its ID
 *
 */
static void reset() {
    wb_robot_init();

    receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");
    gps = wb_robot_get_device("gps");
    imu = wb_robot_get_device("inertial_unit");
    wb_gps_enable(gps, TIME_STEP);
    wb_inertial_unit_enable(imu, TIME_STEP);

    //get motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    int i;
    char s[4] = "ps0";
    for (i = 0; i < NB_SENSORS; i++) {
        ds[i] = wb_robot_get_device(s); // the device name is specified in the world file
        s[2]++;                         // increases the device number
    }
    char* robot_name;
    robot_name = (char*) wb_robot_get_name();

    for (i = 0; i < NB_SENSORS; i++) {
        wb_distance_sensor_enable(ds[i], 64);
    }
    wb_receiver_enable(receiver, 64);

    // Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
    sscanf(robot_name, "epuck%d", &robot_id_u); // read robot id from the robot's name
    robot_id = robot_id_u % FLOCK_SIZE;        // normalize between 0 and FLOCK_SIZE-1

    for (i = 0; i < FLOCK_SIZE; i++) {
        initialized[i] = 0; // Set initialization to 0 (= not yet initialized)
    }

    printf("Reset: robot %d\n", robot_id_u);
}

/*
 * Update speed according to Reynold's rules
 */
void reynolds_rules() {
    int i, j, k;            // Loop counters
    float avg_loc[2] = {0, 0}; // Flock average positions
    float avg_speed[2] = {0, 0}; // Flock average speeds
    float cohesion[2] = {0, 0};
    float dispersion[2] = {0, 0};
    float alignment[2] = {0, 0};
    float dist; // Use it to define distance between robots
    float n_robots; // Use to assign initial number of robots

    dist = 0;
    n_robots = 1;

    #ifdef NEIGHBOURHOOD
    for (i = 0; i < FLOCK_SIZE; i++) {
        if (i == robot_id) {
            continue; // Skip self
        }
        dist = sqrt(pow(loc[i][0] - loc[robot_id][0], 2.0) + pow(loc[i][1] - loc[robot_id][1], 2.0));
        if (dist < NEIGH_THRESHOLD) {
            for (j = 0; j < 2; j++) {
                avg_speed[j] += speed[i][j];
                avg_loc[j] += loc[i][j];
            }
            n_robots++;
        }
    }
    for (j = 0; j < 2; j++) {
        avg_speed[j] /= n_robots - 1;
        avg_loc[j] /= n_robots - 1;
    }
    #else
    for (i = 0; i < FLOCK_SIZE; i++) {
        if (i == robot_id) {
            continue; // Skip self
        }
        for (j = 0; j < 2; j++) {
            avg_speed[j] += speed[i][j];
            avg_loc[j] += loc[i][j];
        }
    }
    for (j = 0; j < 2; j++) {
        avg_speed[j] /= FLOCK_SIZE - 1;
        avg_loc[j] /= FLOCK_SIZE - 1;
    }
    #endif

    /* Reynold's rules */
    // Rule 1 - Cohesion
    for (j = 0; j < 2; j++) {
        if (sqrt(pow(loc[robot_id][0] - avg_loc[0], 2) + pow(loc[robot_id][1] - avg_loc[1], 2)) > RULE1_THRESHOLD) {
            cohesion[j] = avg_loc[j] - loc[robot_id][j];
        }
    }

    // Rule 2 - Dispersion
    for (k = 0; k < FLOCK_SIZE; k++) {
        if (k != robot_id) {
            if (pow(loc[robot_id][0] - loc[k][0], 2) + pow(loc[robot_id][1] - loc[k][1], 2) < RULE2_THRESHOLD) {
                for (j = 0; j < 2; j++) {
                    dispersion[j] += 1 / (loc[robot_id][j] - loc[k][j]);
                }
            }
        }
    }

    // Rule 3 - Alignment
    for (j = 0; j < 2; j++) {
        alignment[j] = avg_speed[j] - speed[robot_id][j];
    }

    for (j = 0; j < 2; j++) {
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

void laplacian_rules(int *msl, int *msr){
    // Goal velocities
    double VGx = 0.0 , VGy = 0.0;
	float x, y;

    double b[FLOCK_SIZE][2] = {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}};

	// Save current positions
	for (int i = 0; i < FLOCK_SIZE; i++) {
		for (int j = 0; j < 2; j++) {
			X_next[i][j] = loc[i][j];
		}
	}

	// Apply Laplacian feedback control
	double LX[FLOCK_SIZE][2];
	double Lb[FLOCK_SIZE][2];
	multiply_matrix_vector(L, X_next, LX);
	multiply_matrix_vector(L, b, Lb);

	for (int i = 0; i < FLOCK_SIZE; i++) {
		for (int j = 0; j < 2; j++) {
			X_next[i][j] += -DELTA_T  * (LX[i][j] - Lb[i][j]);
		}
	}

	printf("X_next of robot %d : %f, %f\n",robot_id, X_next[robot_id][0], X_next[robot_id][1]);

	// Apply constant velocity
	for (int i = 0; i < FLOCK_SIZE; i++) {
		X_next[i][0] += TIME_STEP  * VGx;
		X_next[i][1] += TIME_STEP  * VGy;
	}

	// control law
	x = X_next[robot_id][0]-prev_loc[robot_id][0];
	y = X_next[robot_id][1]-prev_loc[robot_id][1];
	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.01;  // Rotational control coefficient
	float range = sqrtf(x*x + y*y);	  // Distance to the wanted position
	float bearing = atan2(y, x);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);

	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);

	printf(" robot : %d msl: %d, msr: %d\n", robot_id, *msl, *msr);
	printf("range is %f\n", range);
	printf("U is %f\n", u);
	printf("Bearing is %f\n", bearing);
	printf("W is %f\n", w);
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
	float x = speed[robot_id][0]*cosf(loc[robot_id][2]) + speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	float y = -speed[robot_id][0]*sinf(loc[robot_id][2]) + speed[robot_id][1]*cosf(loc[robot_id][2]); // y in robot coordinates

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + y*y);	  // Distance to the wanted position
	float bearing = atan2(y, x);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);

	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);

	// printf("bearing: %f, range: %f\n",bearing * (180.0 / M_PI), range);

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

    // printf("Robot %d position updated to (%f, %f, %f)\n",
    //        robot_id, loc[robot_id][0], loc[robot_id][1], loc[robot_id][2]);
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
		sscanf(inbuffer,"%d#%f#%f#%f##%f#%f",&rob_nb,&rob_x,&rob_y,&rob_theta, &migr[0], &migr[1]);
		// Only info about self will be taken into account at first.

    	// robot_nb %= FLOCK_SIZE;
		if (rob_nb == robot_id) {
			// Initialize self position
			loc[rob_nb][0] = rob_x; 		// x-position
			loc[rob_nb][1] = rob_y; 		// y-position
			loc[rob_nb][2] = rob_theta; 		// theta
			prev_loc[rob_nb][0] = loc[rob_nb][0];
			prev_loc[rob_nb][1] = loc[rob_nb][1];
			initialized[rob_nb] = 1; 		// initialized = true
		}		
		wb_receiver_next_packet(receiver);
	}
}


// Find the fitness for obstacle avoidance of the passed controller
double fitfunc(double weights[DATASIZE],int its) {
    //double left_speed,right_speed; // Wheel speeds
    //double old_left, old_right; // Previous wheel speeds (for recursion)

	RULE1_WEIGHT = weights[0];
    RULE2_WEIGHT = weights[1];
    RULE3_WEIGHT = weights[2];
    DISTANCE_ROBOT = weights[3];

    // Fitness variables
    double fitness=0;             // Fitness of controller
	
	//wb_robot_step(128); // run two steps ????????????????
	//update_position();?????????????????needed ?
    // Evaluate fitness repeatedly
    for (int j=0;j<its;j++) {
		float o_t = 0.0, d_t = 0.0, v_t = 0.0;
		//orientation

		float o_t_real = 0, o_t_imag = 0;
    	for (int i = 0; i < FLOCK_SIZE; i++) {
			float angle = loc[i][2];
			o_t_real += cos(angle);
			o_t_imag += sin(angle);
		}
    	o_t=sqrt(o_t_real * o_t_real + o_t_imag * o_t_imag) / FLOCK_SIZE;

		//distance
		float com_x = 0.0, com_y = 0.0;

		// Step 1: Calculate the center of mass (COM) of the flock
		for (int i = 0; i < FLOCK_SIZE; i++) {
			com_x += loc[i][0];  // Sum up all x-coordinates
			com_y += loc[i][1];  // Sum up all y-coordinates
		}
		com_x /= FLOCK_SIZE;  // Average x-coordinate
		com_y /= FLOCK_SIZE;  // Average y-coordinate

		// Step 2: Compute the deviation of each robot's distance from the COM
		for (int i = 0; i < FLOCK_SIZE; i++) {
			float dist = sqrtf(powf(loc[i][0] - com_x, 2) + powf(loc[i][1] - com_y, 2)); // Distance to COM
			d_t += fabs(dist - RULE1_THRESHOLD); // Deviation from the threshold
		}

   		 // Step 3: Normalize the result and apply the final formula
   		 d_t=1.0 / (1.0 + (d_t / FLOCK_SIZE));

		//velocity
		com_x = 0.0;
		com_y = 0.0;
		float prev_com_x = 0.0, prev_com_y = 0.0;

		// Step 1: Calculate the COM at the current time step
		for (int i = 0; i < FLOCK_SIZE; i++) {
			com_x += loc[i][0];
			com_y += loc[i][1];
		}
		com_x /= FLOCK_SIZE;
		com_y /= FLOCK_SIZE;

		// Step 2: Calculate the COM at the previous time step
		for (int i = 0; i < FLOCK_SIZE; i++) {
			prev_com_x += prev_loc[i][0];
			prev_com_y += prev_loc[i][1];
		}
		prev_com_x /= FLOCK_SIZE;
		prev_com_y /= FLOCK_SIZE;

		// Step 3: Compute the velocity of the COM
		float vel_x = (com_x - prev_com_x) / DELTA_T; // Velocity in x-direction
		float vel_y = (com_y - prev_com_y) / DELTA_T; // Velocity in y-direction

		// Step 4: Compute the projection onto the migration direction

		float migrx = 0.8, migry = 1.6;  // Migration vector !!! HARDCODED !!!


		float flock_migrx = migrx - com_x;
		float flock_migry = migry - com_y;
		float migration_magnitude = sqrtf(flock_migrx * flock_migrx + flock_migry * flock_migry); // Magnitude of migration vector
		float proj_migr = (vel_x * flock_migrx + vel_y * flock_migry) / migration_magnitude;

		// Step 5: Normalize the projection and ensure it is non-negative
		v_t =fmax(proj_migr, 0) / V_MAX;

        fitness += o_t * d_t * v_t;

    
    }
	fitness /= its;
    return fitness;
}


/*
 * Main function
 */
int main(){ 	

	int msl, msr;			// Wheel speeds
	float msl_w, msr_w;
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int rob_nb;			// Robot number
	float rob_x, rob_y, rob_theta;  // Robot position and orientation
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int isthere;			// Flag for robot arrival
	char *inbuffer;			// Buffer for the receiver node
	int max_sens;			// Store highest sensor value
	char outbuffer[255];

 	reset();			// Resetting the robot
	initial_pos();			// Initializing the robot's position

	msl = 0; msr = 0;  
	max_sens = 0;
	strncpy(outbuffer, " ", 1); // dummy assignment
	
	
	// Forever
	for(;;){
		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;
		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) {
			distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value

			// Weighted sum of distance sensor values for Braitenberg vehicle
			bmsr += e_puck_matrix[i] * distances[i];
			bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
		}

		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;

		/* Get information */
		int count = 0;
		while (wb_receiver_get_queue_length(receiver) > 0 && count < FLOCK_SIZE) 
		{
			inbuffer = (char*) wb_receiver_get_data(receiver);
			sscanf(inbuffer,"%d#%f#%f#%f#%d",&rob_nb,&rob_x,&rob_y,&rob_theta, &isthere);
			
			rob_nb %= FLOCK_SIZE;
			if (initialized[rob_nb] == 0) {
				// Get initial positions
				loc[rob_nb][0] = rob_x; //x-position
				loc[rob_nb][1] = rob_y; //y-position
				loc[rob_nb][2] = rob_theta; //theta
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				initialized[rob_nb] = 1;
			} else {
				// Get position update
				// printf("\n Robot [%d] got update robot[%d] = (%f,%f) \n",robot_id, rob_nb,loc[rob_nb][0],loc[rob_nb][1]);
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				loc[rob_nb][0] = rob_x; //x-position
				loc[rob_nb][1] = rob_y; //y-position
				loc[rob_nb][2] = rob_theta; //theta
			}
			
			speed[rob_nb][0] = (1/DELTA_T)*(loc[rob_nb][0]-prev_loc[rob_nb][0]);
			speed[rob_nb][1] = (1/DELTA_T)*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
		
			arrived[rob_nb] = isthere;
			
			count++;

			wb_receiver_next_packet(receiver);
		}

		// Compute self position & speed
		prev_loc[robot_id][0] = loc[robot_id][0];
		prev_loc[robot_id][1] = loc[robot_id][1];

		update_position();

		update_self_motion(msl,msr);
		inbuffer = (char*) wb_receiver_get_data(receiver);
		sscanf(inbuffer,"%d#%f#%f#%f#%d",&rob_nb,&rob_x,&rob_y,&rob_theta, &isthere);

		speed[robot_id][0] = (1/DELTA_T)*(loc[robot_id][0]-prev_loc[robot_id][0]);
		speed[robot_id][1] = (1/DELTA_T)*(loc[robot_id][1]-prev_loc[robot_id][1]);
		
		if (loc[robot_id][0] > 0.4){
			arrived[robot_id] = 1;
		}

		int arrived_count = 0;
		for (int i = 0; i < FLOCK_SIZE; i++) {
			if ((loc[i][0] > 0.3) & (loc[i][0] < 1.5)) {
				arrived_count++; // Increment count if robot has passed x = 0.3
			}
		}

		// If all robots have passed x = 0.3, switch controller
		if (arrived_count == FLOCK_SIZE && strcmp(Controller, "reynold") == 0) {
			strcpy(Controller, "laplace");
			printf("Switched to Laplace\n");
		}

		int exited_count = 0;
		for (int i = 0; i < FLOCK_SIZE; i++) {
			if (loc[i][0] > 1.8) {
				exited_count++; // Increment count if robot has passed x = 0.3
			}
		}

		if (exited_count == FLOCK_SIZE && strcmp(Controller, "laplace") == 0) {
			strcpy(Controller, "reynold");
			printf("Switched to Reynold\n");
		}

		// Controller logic
		if (strcmp(Controller, "reynold") == 0) {
			// Apply Reynold's rules
			reynolds_rules();
			// printf("Applying REYNOLD rules\n");

			// Compute wheels speed from Reynold's speed
			compute_wheel_speeds(&msl, &msr);
		} else if (strcmp(Controller, "laplace") == 0) {
			// Placeholder for Laplace rules
			migr[0] = 4.3; // Migration vector
			migr[1] = 1.6;
			laplacian_rules(&msl, &msr); // Replace with laplacian_rules() when implemented
			// printf("Applying Laplacian rules\n");
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

		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;

		limitf(&msl_w, MAX_SPEED_WEB);
		limitf(&msr_w, MAX_SPEED_WEB);

		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);

		// Send current position to neighbors, uncomment for I15, don't forget to add the declaration of "outbuffer" at the begining of this function.
		/*Implement your code here*/
		if (INTER_VEHICLE_COM) {
			sprintf(outbuffer,"%1d#%f#%f#%f#%d",robot_id,loc[robot_id][0],loc[robot_id][1], loc[robot_id][2], isthere);
			wb_emitter_send(emitter,outbuffer,strlen(outbuffer));
		}

		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}  

