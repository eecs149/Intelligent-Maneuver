#include <math.h>
#include "feedback.h"

/* constants */
#define DISTANCE_BIAS  5      // in mm
#define ANGLE_BIAS     0.1    // in radians

/* variables */
unsigned long start_time;
float distance_travelled;
float angle_turned;

/* from Protocol/navdata.c */
float angles[3];
float velocities[3];

void initialize_feedback(unsigned long time) {
    start_time = time;
    distance_travelled = 0;
    angle_turned = 0;
}

/*
 * distance >= 0 
 */
float do_feedback_forward(double distance_target, unsigned long time) {
    distance_travelled = velocities[0] * (time - start_time);
    
    // if we haven't reached the target distance, keep going
    // and proportionally adjust the rotor velocity
    if (distance_target - distance_travelled > DISTANCE_BIAS) {
        return 1 - distance_travelled / distance_target;   // might need another scale
    }
    else {
        return 0;
    }
}

/*
 * angle_target > 0 for turning left
 * angle_target < 0 for turning right
 */
float do_feedback_turn(double phi_target, unsigned long time) {
    angle_turned = angles[3];   //TODO: check this
    
    // if we haven't reached the target angle, keep going
    // and proportionally adjust the rotor velocity
    if (angle_target - angle_turned > ANGLE_BIAS) {
        return 1 - angle_turned - angle_target;   // might need another scale
    }
    else {
        return 0;
    }
}
