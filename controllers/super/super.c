//******************************************************************************
//  Name:   super.c
//  Author: -
//  Date: -
//  Rev: -
//******************************************************************************

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE 5        // Number of robots in flock
#define TIME_STEP 64        // [ms] Length of time step
#define RULE1_THRESHOLD 0.2
#define V_MAX 0.1           // Maximum speed of a robot
#define D_MAX 0.5           // Maximum distance per timestep

WbNodeRef robs[FLOCK_SIZE];      // Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE]; // Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE]; // Robots rotation fields
WbDeviceTag emitter;             // Single emitter

float loc[FLOCK_SIZE][3];        // Location of everybody in the flock
float prev_loc[FLOCK_SIZE][2];   // Previous locations to calculate velocity
float migrx = 0.8, migry = 1.6;  // Migration vector
float orient_migr;               // Migration orientation
FILE *csv_file;                  // Pointer for CSV file

// Sign function
double sign(double x) {
    return (x > 0) - (x < 0);
}

/*
 * Initialize flock position and devices
 */
void reset(void) {
    wb_robot_init();

    emitter = wb_robot_get_device("emitter");
    if (emitter == 0) printf("missing emitter\n");

    char rob[7] = "epuck0";
    for (int i = 0; i < FLOCK_SIZE; i++) {
        sprintf(rob, "epuck%d", i);
        robs[i] = wb_supervisor_node_get_from_def(rob);
        robs_trans[i] = wb_supervisor_node_get_field(robs[i], "translation");
        robs_rotation[i] = wb_supervisor_node_get_field(robs[i], "rotation");
    }

    // Open CSV file for writing
    csv_file = fopen("flocking_data.csv", "w");
    if (!csv_file) {
        printf("Error opening CSV file!\n");
        exit(1);
    }

    // Write CSV header
    fprintf(csv_file, "Time,o[t],d[t],v[t],M_fl[t],x_0,y_0,x_1,y_1,x_2,y_2,x_3,y_3,x_4,y_4\n");
}

/*
 * Calculate orientation alignment metric o[t]
 */
float calculate_orientation() {
    float o_t_real = 0, o_t_imag = 0;
    for (int i = 0; i < FLOCK_SIZE; i++) {
        float angle = loc[i][2];
        o_t_real += cos(angle);
        o_t_imag += sin(angle);
    }
    return sqrt(o_t_real * o_t_real + o_t_imag * o_t_imag) / FLOCK_SIZE;
}

/*
 * Calculate distance metric d[t]
 */
float calculate_distance() {
    float d_t = 0;
    for (int i = 0; i < FLOCK_SIZE; i++) {
        for (int j = 0; j < FLOCK_SIZE; j++) {
            if (i != j) {
                float dist = sqrtf(powf(loc[i][0] - loc[j][0], 2) + powf(loc[i][1] - loc[j][1], 2));
                d_t += fabs(dist - RULE1_THRESHOLD);
            }
        }
    }
    return 1 / (1 + d_t / (FLOCK_SIZE * (FLOCK_SIZE - 1)));
}

/*
 * Calculate velocity metric v[t]
 */
float calculate_velocity() {
    float v_t = 0;
    for (int i = 0; i < FLOCK_SIZE; i++) {
        // Compute velocity projection along migration direction
        float vx = (loc[i][0] - prev_loc[i][0]) / (TIME_STEP / 1000.0); // Velocity X
        float vy = (loc[i][1] - prev_loc[i][1]) / (TIME_STEP / 1000.0); // Velocity Y
        float proj_migr = (vx * migrx + vy * migry) / sqrtf(migrx * migrx + migry * migry);
        v_t += fmax(proj_migr, 0);
    }
    return v_t / V_MAX;
}

/*
 * Update previous positions
 */
void update_previous_positions() {
    for (int i = 0; i < FLOCK_SIZE; i++) {
        prev_loc[i][0] = loc[i][0];
        prev_loc[i][1] = loc[i][1];
    }
}

/*
 * Compute performance metric and write to CSV
 */
void log_metrics(int time) {
    float o_t = calculate_orientation();
    float d_t = calculate_distance();
    float v_t = calculate_velocity();
    float m_fl_t = o_t * d_t * v_t;

    // Start with time, metrics, and m_fl_t
    fprintf(csv_file, "%d,%f,%f,%f,%f", time, o_t, d_t, v_t, m_fl_t);

    // Append positions of all robots
    for (int i = 0; i < FLOCK_SIZE; i++) {
        fprintf(csv_file, ",%f,%f", loc[i][0], loc[i][1]);
    }

    // End the line
    fprintf(csv_file, "\n");
    fflush(csv_file); // Ensure data is written to the file immediately
}

/*
 * Send initial poses to the robots
 */
void send_init_poses(void) {
    char buffer[255];    // Buffer for sending data

    for (int i = 0; i < FLOCK_SIZE; i++) {
        // Get initial data
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
        loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[1]; // Y
        loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3] *
                    sign(wb_supervisor_field_get_sf_rotation(robs_rotation[i])[2]); // THETA

        // Initialize previous locations
        prev_loc[i][0] = loc[i][0];
        prev_loc[i][1] = loc[i][1];

        // Send initial position and migration vector to robots
        sprintf(buffer, "%1d#%f#%f#%f##%f#%f", i, loc[i][0], loc[i][1], loc[i][2], migrx, migry);
        wb_emitter_send(emitter, buffer, strlen(buffer));
    }

    wb_robot_step(TIME_STEP); // Let the robots process initial data
}

/*
 * Main function
 */
int main(int argc, char *args[]) {
    orient_migr = atan2f(migry, migrx);
    if (orient_migr < 0) {
        orient_migr += 2 * M_PI; // Keep value within 0, 2pi
    }

    reset();
    send_init_poses();

    int time = 0;
    while (wb_robot_step(TIME_STEP) != -1) {
        // Update flock positions
        for (int i = 0; i < FLOCK_SIZE; i++) {
            loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
            loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[1]; // Y
            loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3] *
                        sign(wb_supervisor_field_get_sf_rotation(robs_rotation[i])[2]); // THETA
        }

        // Compute and log metrics every 10 steps
        if (time % 10 == 0) {
            log_metrics(time);
        }

        // Update previous positions
        update_previous_positions();

        time += TIME_STEP;
    }

    // Close the CSV file
    fclose(csv_file);

    return 0;
}
