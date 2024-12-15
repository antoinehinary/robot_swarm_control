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
#define DELTA_T 0.064   // Timestep (seconds)
#define RULE1_THRESHOLD 0.2
#define V_MAX 0.1288          // Maximum speed of a robot
#define D_MAX 0.5           // Maximum distance per timestep

#define INIT_MESSAGE 0

WbNodeRef robs[FLOCK_SIZE];      // Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE]; // Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE]; // Robots rotation fields
WbDeviceTag emitter;             // Single emitter

float loc[FLOCK_SIZE][3];        // Location of everybody in the flock
float prev_loc[FLOCK_SIZE][2];   // Previous locations to calculate velocity
float migrx = 0.8, migry = 1.6;  // Migration vector
float orient_migr;               // Migration orientation
FILE *csv_file;                  // Pointer for CSV file
FILE *Reynold1;                  // Pointer for CSV file
FILE *Reynold2;                  // Pointer for CSV file
FILE *Laplace;                  // Pointer for CSV file

int state = 0;                   // State of the flock 0 = Reynold / 1 = Laplace
int switched = 0;                   // State of the flock 0 = Reynold / 1 = Laplace

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
    csv_file = fopen("full_data.csv", "w");
    if (!csv_file) {
        printf("Error full data opening CSV file!\n");
        exit(1);
    } 

    // Open CSV file for writing
    Reynold1 = fopen("Reynold1.csv", "w");
    if (!Reynold1) {
        printf("Error Reynold1 opening CSV file!\n");
        exit(1);
    } 

    // Open CSV file for writing
    Laplace = fopen("Laplace.csv", "w");
    if (!Laplace) {
        printf("Error Laplace opening CSV file!\n");
        exit(1);
    } 

    // Open CSV file for writing
    Reynold2 = fopen("Reynold2.csv", "w");
    if (!Reynold2) {
        printf("Error Reynold2 opening CSV file!\n");
        exit(1);
    }

    // Write CSV header
    fprintf(csv_file, "Time,o[t],d[t],v[t],M_fl[t],x_0,y_0,x_1,y_1,x_2,y_2,x_3,y_3,x_4,y_4");
    fprintf(Reynold1, "Time,o[t],d[t],v[t],M_fl[t],x_0,y_0,x_1,y_1,x_2,y_2,x_3,y_3,x_4,y_4");
    fprintf(Laplace,  "Time,d[t],v[t],M_fl[t],x_0,y_0,x_1,y_1,x_2,y_2,x_3,y_3,x_4,y_4");
    fprintf(Reynold2, "Time,o[t],d[t],v[t],M_fl[t],x_0,y_0,x_1,y_1,x_2,y_2,x_3,y_3,x_4,y_4");
    
    for (int i = 0; i < FLOCK_SIZE; i++) {
        fprintf(csv_file, ",v_%d", i); // Add headers for individual velocities
        fprintf(Reynold1, ",v_%d", i); // Add headers for individual velocities
        fprintf(Laplace,  ",v_%d", i); // Add headers for individual velocities
        fprintf(Reynold2, ",v_%d", i); // Add headers for individual velocities
    }

    fprintf(csv_file, "\n");
    fprintf(Reynold1, "\n");
    fprintf(Laplace,  "\n");
    fprintf(Reynold2, "\n");
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
    float com_x = 0.0, com_y = 0.0;
    float d_t = 0.0;

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
    return 1.0 / (1.0 + (d_t / FLOCK_SIZE));
}


/*
 * Calculate velocity metric v[t]
 */
float calculate_velocity() {
    float com_x = 0.0, com_y = 0.0;
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
    float flock_migrx = migrx - com_x;
    float flock_migry = migry - com_y;
    float migration_magnitude = sqrtf(flock_migrx * flock_migrx + flock_migry * flock_migry); // Magnitude of migration vector
    float proj_migr = (vel_x * flock_migrx + vel_y * flock_migry) / migration_magnitude;

    // Step 5: Normalize the projection and ensure it is non-negative
    return fmax(proj_migr, 0) / V_MAX;
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
        sprintf(buffer, "%1d#%f#%f#%f##%f#%f#%1d", i, loc[i][0], loc[i][1], loc[i][2], migrx, migry, INIT_MESSAGE);
        wb_emitter_send(emitter, buffer, strlen(buffer));
    }

    wb_robot_step(TIME_STEP); // Let the robots process initial data
}

/*
 * Compute distance metric d[t] for Laplace
 */
float calculate_distance_laplace() {
    float d_t = 0.0;

    // Compute the sum of the inverse of the distances to the goal
    for (int k = 0; k < FLOCK_SIZE; k++) {
        float g_k_x = migrx; // Target position x (Laplace mode assumes single goal)
        float g_k_y = migry; // Target position y

        float dist = sqrtf(powf(loc[k][0] - g_k_x, 2) + powf(loc[k][1] - g_k_y, 2));
        d_t += 1.0 / dist; // Inverse distance
    }

    // Normalize and compute the distance metric
    return 1.0 / (1.0 + (d_t / FLOCK_SIZE));
}

/*
 * Compute velocity metric v[t] for Laplace
 */
float calculate_velocity_laplace() {
    float v_t = 0.0;

    // Compute the velocity of each robot normalized by D_MAX
    for (int k = 0; k < FLOCK_SIZE; k++) {
        float vel_x = (loc[k][0] - prev_loc[k][0]) / DELTA_T; // Velocity in x
        float vel_y = (loc[k][1] - prev_loc[k][1]) / DELTA_T; // Velocity in y

        float vel_magnitude = sqrtf(vel_x * vel_x + vel_y * vel_y); // Magnitude of velocity
        v_t += vel_magnitude / D_MAX; // Normalize by D_MAX
    }

    // Average the velocities across the flock
    return v_t / FLOCK_SIZE;
}

/*
 * Compute performance metric and write to CSV
 */
void log_metrics(int time) {
    float o_t = 0.0; // Orientation alignment (only for Reynold)
    float d_t, v_t, m_fl_t;

    // Compute the metrics based on the current state
    if (state == 0 || state == 2) { // Reynold mode
        o_t = calculate_orientation();
        d_t = calculate_distance(); // Original Reynold distance metric
        v_t = calculate_velocity(); // Original Reynold velocity metric
        m_fl_t = o_t * d_t * v_t;   // Reynold's metric includes orientation
    } else { // Laplace mode
        d_t = calculate_distance_laplace(); // New Laplace distance metric
        v_t = calculate_velocity_laplace(); // New Laplace velocity metric
        m_fl_t = d_t * v_t;                 // Laplace's metric excludes orientation
    }

    // Write to the appropriate file based on the state
    if (state == 1) {
        fprintf(Laplace, "%d,%f,%f,%f", time, d_t, v_t, m_fl_t);
        fprintf(csv_file, "%d,%f,%f,%f,%f", time, 0.0, d_t, v_t, m_fl_t);
        // Append positions of all robots
        for (int i = 0; i < FLOCK_SIZE; i++) {
            fprintf(Laplace, ",%f,%f", loc[i][0], loc[i][1]);
            fprintf(csv_file, ",%f,%f", loc[i][0], loc[i][1]);
        }
        // Compute and append velocities of all robots
        for (int i = 0; i < FLOCK_SIZE; i++) {
            float vel_x = (loc[i][0] - prev_loc[i][0]) / DELTA_T;
            float vel_y = (loc[i][1] - prev_loc[i][1]) / DELTA_T;
            float velocity = sqrtf(vel_x * vel_x + vel_y * vel_y);
            fprintf(Laplace, ",%f", velocity);
            fprintf(csv_file, ",%f", velocity);
        }
        fprintf(Laplace, "\n");
        fprintf(csv_file, "\n");
        fflush(Laplace); // Ensure the data is written to the file
        fflush(csv_file); // Ensure the data is written to the file
    } else if (state == 0) {
        fprintf(Reynold1, "%d,%f,%f,%f,%f", time, o_t, d_t, v_t, m_fl_t);
        fprintf(csv_file, "%d,%f,%f,%f,%f", time, o_t, d_t, v_t, m_fl_t);
        // Append positions of all robots
        for (int i = 0; i < FLOCK_SIZE; i++) {
            fprintf(Reynold1, ",%f,%f", loc[i][0], loc[i][1]);
            fprintf(csv_file, ",%f,%f", loc[i][0], loc[i][1]);
        }
        // Compute and append velocities of all robots
        for (int i = 0; i < FLOCK_SIZE; i++) {
            float vel_x = (loc[i][0] - prev_loc[i][0]) / DELTA_T;
            float vel_y = (loc[i][1] - prev_loc[i][1]) / DELTA_T;
            float velocity = sqrtf(vel_x * vel_x + vel_y * vel_y);
            fprintf(Reynold1, ",%f", velocity);
            fprintf(csv_file, ",%f", velocity);
        }
        fprintf(Reynold1, "\n");
        fprintf(csv_file, "\n");
        fflush(Reynold1); // Ensure the data is written to the file
        fflush(csv_file); // Ensure the data is written to the file
    } else if (state == 2) {
        fprintf(csv_file, "%d,%f,%f,%f,%f", time, o_t, d_t, v_t, m_fl_t);
        fprintf(Reynold2, "%d,%f,%f,%f,%f", time, o_t, d_t, v_t, m_fl_t);
        // Append positions of all robots
        for (int i = 0; i < FLOCK_SIZE; i++) {
            fprintf(Reynold2, ",%f,%f", loc[i][0], loc[i][1]);
            fprintf(csv_file, ",%f,%f", loc[i][0], loc[i][1]);
        }
        // Compute and append velocities of all robots
        for (int i = 0; i < FLOCK_SIZE; i++) {
            float vel_x = (loc[i][0] - prev_loc[i][0]) / DELTA_T;
            float vel_y = (loc[i][1] - prev_loc[i][1]) / DELTA_T;
            float velocity = sqrtf(vel_x * vel_x + vel_y * vel_y);
            fprintf(csv_file, ",%f", velocity);
            fprintf(Reynold2, ",%f", velocity);
        }
        fprintf(Reynold2, "\n");
        fprintf(csv_file, "\n");
        fflush(Reynold2); // Ensure the data is written to the file
        fflush(csv_file); // Ensure the data is written to the file
    }
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

        // Detect mode change and adjust metrics accordingly
        if (state == 0) { // Reynold mode
            int arrived_count = 0;
            for (int i = 0; i < FLOCK_SIZE; i++) {
                if ((loc[i][0] > 0.3) && (loc[i][0] < 1.5)) {
                    arrived_count++;
                }
            }
            if (arrived_count == FLOCK_SIZE) {
                state = 1; // Switch to Laplace mode
                printf("Switched to Laplace mode at Time: %d, state : %d\n", time, state);
            }
        } else if (state == 1) { // Laplace mode
            int exited_count = 0;
            for (int i = 0; i < FLOCK_SIZE; i++) {
                if (loc[i][0] > 2.5) {
                    exited_count++;
                }
            }
            if (exited_count == FLOCK_SIZE) {
                state = 2; // Switch back to Reynold mode
                printf("Switched to Reynold2 mode at Time: %d, state : %d\n", time, state);
            }
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
    fclose(Reynold1);
    fclose(Reynold2);
    fclose(Laplace);
    fclose(csv_file);

    return 0;
}
