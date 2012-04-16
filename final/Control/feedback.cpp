#include <math.h>
#include "feedback.h"

/* constants */
#define DIST_BIAS  5	// in mm
#define ANGLE_BIAS 10 	// units?

/* variables */
unsigned long start_time;
float dist_travelled;
float angle_turned;

/* from Protocol/navdata.c */
float angles[3];
float velocities[3];

void initialize_feedback(unsigned long time) {
    start_time = time;
    dist_travelled = 0;
    angle_turned = 0;
}

/*
 * @param: dist_target > 0 for flying forward
 *         dist_target < 0 for flying backward
 */
DroneMovement fly_target_distance_x(float dist_target, unsigned long time) {
	DroneMovement command = HOVER;

	dist_travelled = velocities[0] * (time - start_time);
	
	if (dist_target > 0) {
		command = FLY_FORWARD;
	}
	else {
		command = FLY_BACKWARD;
	}

	// if the drone has travelled enough, make it stop
	if (fabs(dist_target - dist_travelled) > DIST_BIAS) {
		command = HOVER;
	}

	return command;
}

/*
 * @param: dist_target > 0 for flying left
 *         dist_target < 0 for flying right
 */
DroneMovement fly_target_distance_y(float dist_target, unsigned long time) {
	DroneMovement command = HOVER;
	
	dist_travelled = velocities[1] * (time - start_time);

	if (dist_target > 0) {
		command = FLY_LEFT;
	}
	else {
		command = FLY_RIGHT;
	}

	// if the drone has flown enough, make it stop
	if (fabs(dist_target - dist_travelled) > DIST_BIAS) {
		command = HOVER;
	}

	return command;
}

/*
 * @param: angle_target > 0 for turning left
 *         angle_target < 0 for turning right
 */
DroneMovement turn_target_distance(float angle_target, unsigned long time) {
    DroneMovement command = HOVER;

    angle_turned = angles[3];
    
    if (angle_target > 0) {
    	command = TURN_LEFT;
    }
    else {
    	command = TURN_RIGHT;
    }
    
    // if the drone has turned enough, make it stop
    if (fabs(angle_target - angle_turned) > ANGLE_BIAS) {
    	command = HOVER;
    }
    
    return command;
}

