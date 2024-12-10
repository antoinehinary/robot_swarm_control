//******************************************************************************
//  Name:   flocking_formation_controller.c
//  Author: -
//  Date:   -
//  Rev:    -
//******************************************************************************


#include <stdio.h>
#include <math.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdlib.h>
#include <stdbool.h>

#define FLOCK_SIZE	  5	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step



WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot

WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID

char Controller[7] = "reynold";
float loc[FLOCK_SIZE][3];	// X, Y, Theta of all robots
float prev_loc[FLOCK_SIZE][3];	// Previous X, Y, Theta values
int arrived[FLOCK_SIZE] = {0,0,0,0};
int t;

static void reset() {
	
	wb_robot_init();
                                                                                                                                                
	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	
	//get motors
	// left_motor = wb_robot_get_device("left wheel motor");
	// right_motor = wb_robot_get_device("right wheel motor");
	// wb_motor_set_position(left_motor, INFINITY);
	// wb_motor_set_position(right_motor, INFINITY);

	// int i;
	// char s[4]="ps0";
	// for(i=0; i<NB_SENSORS;i++) {
	// 	ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
	// 	s[2]++;				// increases the device number
	// }
	char* robot_name; 
	robot_name=(char*) wb_robot_get_name(); 

	// for(i=0;i<NB_SENSORS;i++) {
	// 	wb_distance_sensor_enable(ds[i],64);
	// }
	wb_receiver_enable(receiver,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

	// for(i=0; i<FLOCK_SIZE; i++) {
	// 	initialized[i] = 0; 		  // Set initialization to 0 (= not yet initialized)
	// }

	printf("Reset: robot %d\n",robot_id_u);
}

void laplacian_rules(){
	printf("laplace");


	#define THRESHOLD 0.001

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

// double max_difference(double X[FLOCK_SIZE][2], double X_next[FLOCK_SIZE][2]) {
//     double max_diff = 0.0;
//     for (int i = 0; i < FLOCK_SIZE; i++) {
//         for (int j = 0; j < 2; j++) {
//             double diff = fabs(X_next[i][j] - X[i][j]);
//             if (diff > max_diff) {
//                 max_diff = diff;
//             }
//         }
//     }
//     return max_diff;
// }


    // Goal velocities
    double VGx = 2.5 , VGy = 1.6;

    // Initial positions (X and Y)
    double X[FLOCK_SIZE][2] = loc
    double b[FLOCK_SIZE][2] = {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}};
    double X_next[FLOCK_SIZE][2];

    // Laplace matrix
    double L[FLOCK_SIZE][FLOCK_SIZE] = {
        { 1, -1,  0,  0,  0},
        {-1,  2, -1,  0,  0},
        { 0, -1,  2, -1,  0},
        { 0,  0, -1,  2, -1},
        { 0,  0,  0, -1,  1}
    };




	// Save current positions
	for (int i = 0; i < FLOCK_SIZE; i++) {
		for (int j = 0; j < 2; j++) {
			X_next[i][j] = X[i][j];
		}
	}

	// Apply Laplacian feedback control
	double LX[FLOCK_SIZE][2];
	multiply_matrix_vector(L, X, LX);

	for (int i = 0; i < FLOCK_SIZE; i++) {
		for (int j = 0; j < 2; j++) {
			X_next[i][j] += -TIME_STEP  * (LX[i][j] - b[i][j]);
		}
	}

	// Apply constant velocity
	for (int i = 0; i < FLOCK_SIZE; i++) {
		X_next[i][0] += TIME_STEP  * VGx;
		X_next[i][1] += TIME_STEP  * VGy;
	}

	// Update positions for the next iteration
	for (int i = 0; i < FLOCK_SIZE; i++) {
		for (int j = 0; j < 2; j++) {
			X[i][j] = X_next[i][j];
		}
	}

}



int main(int argc, char *argv[]) {
	
	int rob_nb;			// Robot number
	float rob_x, rob_y, rob_theta;  // Robot position and orientation
	int isthere;
	char *inbuffer;			// Buffer for the receiver node
	char outbuffer[255];
	reset();
	// int i;

	// Forever
	for(;;){

		if (t % 10 == 0) {
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[1]; // Y
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]*sign(wb_supervisor_field_get_sf_rotation(robs_rotation[i])[2]);; // THETA			
			}
			
			print('envoie')
			for (int i=0;i<FLOCK_SIZE;i++) {
			// printf("Robot %d: x=%f, y=%f, theta=%f\n", i, loc[i][0], loc[i][1], loc[i][2]);
				sprintf(outbuffer,"%1d#%f#%f#%f#%d",robot_id,loc[robot_id][0],loc[robot_id][1], loc[robot_id][2], arrived[robot_id]);
				wb_emitter_send(emitter,outbuffer,strlen(outbuffer));
			}
				
		}
		t += TIME_STEP;



		while (wb_receiver_get_queue_length(receiver) > 0) 
		{ 
			print('entre dans buffer')
			/* Get information */
			inbuffer = (char*) wb_receiver_get_data(receiver);
			sscanf(inbuffer,"%d#%f#%f#%f#%d",&rob_nb,&rob_x,&rob_y,&rob_theta, &isthere);
			
// 			if ((int) rob_nb/FLOCK_SIZE == (int) robot_id/FLOCK_SIZE) {
// 				rob_nb %= FLOCK_SIZE;
// 				if (initialized[rob_nb] == 0) {
// 				// Get initial positions
// 				loc[rob_nb][0] = rob_x; //x-position
// 				loc[rob_nb][1] = rob_y; //y-position
// 				loc[rob_nb][2] = rob_theta; //theta
// 				prev_loc[rob_nb][0] = loc[rob_nb][0];
// 				prev_loc[rob_nb][1] = loc[rob_nb][1];
// 				initialized[rob_nb] = 1;
// 			} else {
// 				// Get position update
// //				printf("\n got update robot[%d] = (%f,%f) \n",rob_nb,loc[rob_nb][0],loc[rob_nb][1]);
// 				prev_loc[rob_nb][0] = loc[rob_nb][0];
// 				prev_loc[rob_nb][1] = loc[rob_nb][1];
// 				loc[rob_nb][0] = rob_x; //x-position
// 				loc[rob_nb][1] = rob_y; //y-position
// 				loc[rob_nb][2] = rob_theta; //theta
// 			}
			
// 			speed[rob_nb][0] = (1/TIME_STEP )*(loc[rob_nb][0]-prev_loc[rob_nb][0]);
// 			speed[rob_nb][1] = (1/TIME_STEP )*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
// 			count++;
// 			}
			arrived[rob_nb] = isthere;

			wb_receiver_next_packet(receiver);
		}

		for (int i=0;i<FLOCK_SIZE;i++) {
			printf("Robot %d: x=%f, y=%f, theta=%f\n", i, loc[i][0], loc[i][1], loc[i][2], arrived[i]);
		}
		if (loc[robot_id][0] > 0.4){
			arrived[robot_id] = 1;
		}

		int cout = 0;
		for(int i=0;i<FLOCK_SIZE;i++) {
			if (arrived[i] == 1){
				cout++;
				if (cout == FLOCK_SIZE){
					strcpy(Controller, "laplace");
				}
			}else{
				break;
			}
		}

		// Change the content to "laplace"
    	if (strcmp(Controller, "reynold") == 0){
			// reynolds_rules();
			printf("reynold");
		}else if (strcmp(Controller, "laplace")==0){
			laplacian_rules();
		}
		

		
		

		// Continue one step
		wb_robot_step(TIME_STEP);

	}
}



