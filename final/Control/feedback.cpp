#include <stdio.h>
#include <math.h>
#include <algorithm>
#include "feedback.h"
#include "control.h"

/* constants */
#define DISTANCE_BIAS  5      // in mm
#define ANGLE_BIAS     0.1    // in radians

/* variables */
double start_time = 0.0;
float distance_travelled = 0.0f;
float angle_turned = 0.0f;

/* from Protocol/navdata.c */

void initialize_feedback(double time) {
    start_time = time;
    distance_travelled = 0;
    angle_turned = 0;
}

/*
 * distance >= 0 
 */
float do_feedback_forward(double distance_target, double time) {
    distance_travelled = fabs(vx) * (time - start_time);
    printf("vx: %f, vy: %f, distance_traveled: %f\n", vx, vy, distance_travelled);
    start_time = time;
    
    // if we haven't reached the target distance, keep going
    // and proportionally adjust the rotor velocity
    if (distance_target - distance_travelled > DISTANCE_BIAS) {
        return std::min(0.05 * (1 - distance_travelled / distance_target), 0.05);   // might need another scale
    }
    else {
        return 0;
    }
}

/*
 * angle_target > 0 for turning left
 * angle_target < 0 for turning right
 */
float do_feedback_turn(double phi_target, double time) {
    angle_turned = fabs(gyroz) * (time - start_time);   //TODO: check this
    start_time = time;
    
    // if we haven't reached the target angle, keep going
    // and proportionally adjust the rotor velocity
    if (phi_target - angle_turned > ANGLE_BIAS) {
        return 1 - angle_turned - phi_target;   // might need another scale
    }
    else {
        return 0;
    }
}

