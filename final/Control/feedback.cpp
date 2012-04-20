#include <math.h>
#include "feedback.h"

/* constants */
#define DIST_BIAS  5	   // TODO: check units
//#define ANGLE_BIAS 10 

/* variables */
unsigned long start_time;
float dist_travelled;
//float angle_turned;

/* from Protocol/navdata.c */
//float angles[3];
float velocities[3];

void initialize_feedback(unsigned long time) {
    start_time = time;
    dist_travelled = 0;
    //angle_turned = 0;
}

DroneMovement process_feedback(vector_t vector, unsigned long time) {
    float dist = vector.distance;        // target distance
    int dir = vector.direction_toggle;   // target direction
    DroneMovement command;     // this gets interpreted in control.cpp

    if (dist == 0 && dir == 0) {
        fly_target_distance_x(dist, time, command);
    }
    else if (dist == 0 && dir == 1) {
        fly_target_distance_y(dist, time, command);
    }
    else if (dist == 0 && dir == 2) {
        fly_target_distance_z(dist, time, command);
    }
    else {
        command = hover;    // default
    }

    return command;
}

/*
 * dist_target > 0 for flying forward
 * dist_target < 0 for flying backward
 */
void fly_target_distance_x(float dist_target, unsigned long time, DroneMovement command) {
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
}

/*
 * dist_target > 0 for flying left
 * dist_target < 0 for flying right
 */
void fly_target_distance_y(float dist_target, unsigned long time, DroneMovement command) {
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
}

/*
 * dist_target > 0 for flying up
 * dist_target < 0 for flying down
 */
void fly_target_distance_z(float dist_target, unsigned long time, DroneMovement command) {
    dist_travelled = velocities[1] * (time - start_time);

    if (dist_target > 0) {
        command = FLY_UP;
    }
    else {
        command = FLY_DOWN;
    }

    // if the drone has flown enough, make it stop
    if (fabs(dist_target - dist_travelled) > DIST_BIAS) {
        command = HOVER;
    }
}


/*
 * angle_target > 0 for turning left
 * angle_target < 0 for turning right
 */
/*DroneMovement turn_target_distance(float angle_target, unsigned long time) {
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
}*/

