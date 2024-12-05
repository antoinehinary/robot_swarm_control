//******************************************************************************
//  Name:   super.c
//  Author: -
//  Date: -
//  Rev: -
//******************************************************************************


#include <stdio.h>
// #include <string.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>


#define TIME_STEP	64		// [ms] Length of time step
#define VERBOSE 1
#define FLOCK_SIZE	4		// Number of robots in flock

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbDeviceTag emitter;			// Single emitter

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

int t = 0;


// sign function 
double sign(double x) {

    double y;
     
    if (x>0) y = 1;
    if (x<0) y = -1;
    
    return y;

}

void reset(void) {

	wb_robot_init();

	emitter = wb_robot_get_device("emitter");
	if (emitter==0) printf("missing emitter\n");
	
	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
}


int main(int argc, char *argv[]) {

  	// controller initialization
  	reset(); 

  	int i;

	for(;;) { // Main endless control loop
		wb_robot_step(TIME_STEP);
		
		// if (t % 10 == 0) {
		// 	for (i=0;i<FLOCK_SIZE;i++) {
		// 		// Get data
		// 		loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
		// 		loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[1]; // Y
		// 		loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]*sign(wb_supervisor_field_get_sf_rotation(robs_rotation[i])[2]);; // THETA			
		// 	}
			
			// if (VERBOSE) {
			// 	for (i=0;i<FLOCK_SIZE;i++) {
			// 	// printf("Robot %d: x=%f, y=%f, theta=%f\n", i, loc[i][0], loc[i][1], loc[i][2]);

			// 	sprintf(buffer,"%1d#%f#%f#%f",i+offset,loc[i][0],loc[i][1],loc[i][2]);
            // 	wb_emitter_send(emitter,buffer,strlen(buffer));
			// 	}
				
			// }			
		// }
		t += TIME_STEP;
	}

  return 0;
}




