#include <stdio.h>
#include <math.h>
#include <algorithm>
#include "feedback.h"
#include "control.h"

/* constants */
#define DISTANCE_BIAS  5      // in mm
#define ANGLE_BIAS     5    // in degrees

/* variables */
float distance_travelled = 0.0f;
float angle_turned = 0.0f;

/* from Protocol/navdata.c */
void initialize_feedback() {
    distance_travelled = 0;
    angle_turned = 0;
}

/*
 * distance >= 0 
 */
float do_feedback_forward(double distance_target, double dt) {
    distance_travelled = fabs(vx) * dt;
    printf("vx: %f, vy: %f, distance_traveled: %f\n", vx, vy, distance_travelled);
    
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
float do_feedback_turn(double phi_target, double dt) {
    angle_turned = fabs(gyroz) * dt;
    printf("gyrox: %f, gyroy: %f, gyroz: %f, angle_turned: %f\n",
           gyrox, gyroy, gyroz, angle_turned);

    // if we haven't reached the target angle, keep going
    // and proportionally adjust the rotor velocity
    
    // turn left
    if (phi_target > 0 && phi_target - angle_turned > ANGLE_BIAS) {
        return angle_turned / phi_target - 1;
    }
    
    // turn right
    else if (phi_target < 0 && phi_target - angle_turned > ANGLE_BIAS) {
        return 1 - angle_turned / fabs(phi_target);
    }

    else {
        return 0;
    }
}

