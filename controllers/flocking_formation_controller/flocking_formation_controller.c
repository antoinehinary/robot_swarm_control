/*****************************************************************************/
/* File:         flocking_formation_controller.c                             */
/* Version:      1.1                                                         */
/* Date:         8-Nov-24                                                    */
/* Description:  Reynolds flocking control                                   */
/*                                                                           */
/* Author:       8-Nov-24 by Antoine Hinary                                  */
/* Last revision: 5-Dec-24                                                  */
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

#define NB_SENSORS      8       // Number of distance sensors
#define MIN_SENS        60      // Minimum sensibility value
#define MAX_SENS        250     // Maximum sensibility value
#define MAX_SPEED       800     // Maximum speed
#define MAX_SPEED_WEB   6.28    // Maximum speed webots
#define FLOCK_SIZE      5       // Size of flock
#define TIME_STEP       64      // [ms] Length of time step
#define AXLE_LENGTH     0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS    0.0205  // Wheel radius (meters)
#define DELTA_T         0.064   // Timestep (seconds)
#define RULE1_THRESHOLD 0.20    // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT    (0.7/10)// Weight of aggregation rule. default 0.6/10
#define RULE2_THRESHOLD 0.15    // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT    (0.02/10)// Weight of dispersion rule. default 0.02/10
#define RULE3_WEIGHT    (1.0/10)// Weight of alignment rule. default 1.0/10
#define MIGRATION_WEIGHT (0.03/10)// Weight of attraction towards the common goal. default 0.01/10
#define MIGRATORY_URGE 1         // Tells the robots if they should just go forward or move towards a specific migratory direction
#define NEIGHBOURHOOD 1          // Tells the robot considering neighbors or all robots during flocking
#define NEIGH_THRESHOLD 0.5      // Threshold to consider neighbourhood
#define INTER_VEHICLE_COM 1      // Set 1 if there is intervehicle communication
#define VERBOSE 0
#define ABS(x) ((x>=0)?(x):-(x))

WbDeviceTag left_motor, right_motor, ds[NB_SENSORS], receiver, emitter, gyro, gps;
int e_puck_matrix[16] = {50, 35, 20, 0, 0, -20, -35, -45, -45, -35, -20, 0, 0, 20, 35, 50};
int robot_id_u, robot_id;
float loc[FLOCK_SIZE][3], prev_loc[FLOCK_SIZE][3], speed[FLOCK_SIZE][2];
int initialized[FLOCK_SIZE];
float migr[2] = {0.8, 1.6};
double z_ang_vel;

static void reset() {
    wb_robot_init();

    receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");
    gyro = wb_robot_get_device("gyro");
    gps = wb_robot_get_device("gps");

    wb_gyro_enable(gyro, TIME_STEP);
    wb_gps_enable(gps, TIME_STEP);

    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    char s[4] = "ps0";
    for (int i = 0; i < NB_SENSORS; i++) {
        ds[i] = wb_robot_get_device(s);
        s[2]++;
        wb_distance_sensor_enable(ds[i], TIME_STEP);
    }

    char* robot_name = (char*)wb_robot_get_name();
    sscanf(robot_name, "epuck%d", &robot_id_u);
    robot_id = robot_id_u % FLOCK_SIZE;

    for (int i = 0; i < FLOCK_SIZE; i++) {
        initialized[i] = 0;
    }

    printf("Reset: robot %d\n", robot_id_u);
}

void initial_pos() {
    const double *gps_values;

    // Wait until GPS and Gyro values are available
    while (initialized[robot_id] == 0) {
		printf("Init Position \n");
        // Read GPS values
        gps_values = wb_gps_get_values(gps);

        // Initialize position and orientation
        loc[robot_id][0] = gps_values[0]; // x-position
        loc[robot_id][1] = gps_values[1]; // y-position
        loc[robot_id][2] = 0.0;           // Start with zero orientation (will update later)
    }
}


void update_position() {
    const double *gps_values = wb_gps_get_values(gps);
    const double *gyro_values = wb_gyro_get_values(gyro);

    // Update position
    loc[robot_id][0] = gps_values[0];
    loc[robot_id][1] = gps_values[1];
    loc[robot_id][2] += gyro_values[2] * DELTA_T; // Update orientation with gyro angular velocity

    // Normalize orientation to [0, 2*PI]
    if (loc[robot_id][2] > 2 * M_PI)
        loc[robot_id][2] -= 2 * M_PI;
    if (loc[robot_id][2] < 0)
        loc[robot_id][2] += 2 * M_PI;

    printf("Robot %d position updated to (%f, %f, %f)\n",
           robot_id, loc[robot_id][0], loc[robot_id][1], loc[robot_id][2]);
}


void reynolds_rules() {
    float avg_loc[2] = {0, 0};
    float avg_speed[2] = {0, 0};
    float cohesion[2] = {0, 0};
    float dispersion[2] = {0, 0};
    float alignment[2] = {0, 0};
    float dist;
    int n_robots = 1; // Include self

    // Calculate averages for neighbors
    for (int i = 0; i < FLOCK_SIZE; i++) {
        if (i == robot_id) continue; // Skip self

        dist = sqrt(pow(loc[i][0] - loc[robot_id][0], 2.0) + pow(loc[i][1] - loc[robot_id][1], 2.0));
        if (dist < NEIGH_THRESHOLD) {
            for (int j = 0; j < 2; j++) {
                avg_speed[j] += speed[i][j];
                avg_loc[j] += loc[i][j];
            }
            n_robots++;
        }

        printf("Robot %d -> Robot %d, Dist: %f\n", robot_id, i, dist);
    }

    if (n_robots > 1) {
        for (int j = 0; j < 2; j++) {
            avg_speed[j] /= (n_robots - 1);
            avg_loc[j] /= (n_robots - 1);
        }
    } else {
        for (int j = 0; j < 2; j++) {
            avg_speed[j] = 0.0;
            avg_loc[j] = loc[robot_id][j]; // Default to own position
        }
    }

    // Cohesion
    for (int j = 0; j < 2; j++) {
        cohesion[j] = avg_loc[j] - loc[robot_id][j];
    }

    // Dispersion
    for (int k = 0; k < FLOCK_SIZE; k++) {
        if (k != robot_id) {
            dist = sqrt(pow(loc[robot_id][0] - loc[k][0], 2) + pow(loc[robot_id][1] - loc[k][1], 2));
            if (dist < RULE2_THRESHOLD) {
                for (int j = 0; j < 2; j++) {
                    dispersion[j] += 1 / (loc[robot_id][j] - loc[k][j] + 1e-5); // Avoid divide by zero
                }
            }
        }
    }

    // Alignment
    for (int j = 0; j < 2; j++) {
        alignment[j] = avg_speed[j] - speed[robot_id][j];
    }

    // Combine rules into speed
    for (int j = 0; j < 2; j++) {
        speed[robot_id][j] = RULE1_WEIGHT * cohesion[j]
                           + RULE2_WEIGHT * dispersion[j]
                           + RULE3_WEIGHT * alignment[j];

        #ifdef MIGRATORY_URGE
        speed[robot_id][j] += MIGRATION_WEIGHT * (migr[j] - loc[robot_id][j]);
        #endif
    }

    printf("Robot %d - Cohesion: [%f, %f], Dispersion: [%f, %f], Alignment: [%f, %f], Speed: [%f, %f]\n",
           robot_id, cohesion[0], cohesion[1], dispersion[0], dispersion[1], alignment[0], alignment[1],
           speed[robot_id][0], speed[robot_id][1]);
}



void compute_wheel_speeds(int *msl, int *msr) {
    float x = speed[robot_id][0] * cosf(loc[robot_id][2]) + speed[robot_id][1] * sinf(loc[robot_id][2]);
    float y = -speed[robot_id][0] * sinf(loc[robot_id][2]) + speed[robot_id][1] * cosf(loc[robot_id][2]);
    float Ku = 0.2, Kw = 0.5, range = sqrtf(x * x + y * y), bearing = atan2(y, x);
    float u = Ku * range * cosf(bearing), w = Kw * bearing;

    *msl = (u - AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
    *msr = (u + AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);

    *msl = (*msl > MAX_SPEED) ? MAX_SPEED : ((*msl < -MAX_SPEED) ? -MAX_SPEED : *msl);
    *msr = (*msr > MAX_SPEED) ? MAX_SPEED : ((*msr < -MAX_SPEED) ? -MAX_SPEED : *msr);
}

void update_self_motion(int msl, int msr) {
    float theta = loc[robot_id][2], dr = msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T, dl = msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    float du = (dr + dl) / 2.0, dtheta = (dr - dl) / AXLE_LENGTH, dx = du * cosf(theta), dy = du * sinf(theta);

    loc[robot_id][0] += dx;
    loc[robot_id][1] += dy;
    loc[robot_id][2] += dtheta;

    loc[robot_id][2] = fmod(loc[robot_id][2], 2 * M_PI);
    if (loc[robot_id][2] < 0) loc[robot_id][2] += 2 * M_PI;
}

void broadcast_position() {
    char outbuffer[255];
    
    // Format the message with robot ID, position, and orientation
    sprintf(outbuffer, "%d#%f#%f#%f", robot_id, loc[robot_id][0], loc[robot_id][1], loc[robot_id][2]);

    // Send the message through the emitter
    wb_emitter_send(emitter, outbuffer, strlen(outbuffer) + 1);

    // Debugging output (optional)
    printf("Robot %d broadcasting position: %s\n", robot_id, outbuffer);
}


int main() {
    int msl = 0, msr = 0;

    reset();
    initial_pos();

    while (true) {
        // Broadcast position to other robots
        broadcast_position();

        // Update Reynolds' rules
        reynolds_rules();

        // Compute wheel speeds
        compute_wheel_speeds(&msl, &msr);

        // Set wheel speeds
        wb_motor_set_velocity(left_motor, msl * MAX_SPEED_WEB / 1000.0);
        wb_motor_set_velocity(right_motor, msr * MAX_SPEED_WEB / 1000.0);
     
	    // Update position from GPS and gyro
        update_position();
		wb_robot_step(TIME_STEP); // Wait for sensors to update
    }

    return 0;
}

