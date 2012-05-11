#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include "feedback.h"
#include "control.h"

/* constants */
#define DISTANCE_BIAS  70  // in mm
#define ANGLE_BIAS     2   // in degrees

/* variables */
float start_position = 0.0f;
float start_angle = 0.0f;

/* from Protocol/navdata.c */
void initialize_feedback() {
    start_position = accumDist;
    start_angle = gyroz;
}

/*
 * distance >= 0 
 */
float do_feedback_forward(double distance_target) {
    float distance_travelled = accumDist - start_position;
    printf("accumX: %f, accumY: %f, accumDist: %f, start_position: %f, distance_traveled: %f\n",
           accumX, accumY, accumDist, start_position, distance_travelled);

    // if we haven't reached the target distance, keep going
    // and proportionally adjust the rotor velocity
    if (fabs(distance_target - distance_travelled) > DISTANCE_BIAS) {
        return (distance_target - distance_travelled)/10000.0 * -1;
    }
    else {
        return 0.0f;
    }
}

/*
 * angle_target > 0 for turning left
 * angle_target < 0 for turning right
 */
float do_feedback_turn(double phi_target) {
    float angle_turned = (gyroz - start_angle) * -1;
    printf("gyroz: %f, angle_turned: %f\n", gyroz, angle_turned);

    // if we haven't reached the target angle, keep going
    // and proportionally adjust the rotor velocity
    if (fabs(phi_target - angle_turned) > ANGLE_BIAS) {
        return (phi_target - angle_turned)/250 * -1;
    }

    else {
        return 0.0f;
    }
}

