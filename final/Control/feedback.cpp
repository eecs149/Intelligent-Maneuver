#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include "feedback.h"
#include "control.h"

/* constants */
#define DISTANCE_BIAS  100  // in mm
#define ANGLE_BIAS     5    // in degrees

/* variables */
float start_position = 0.0f;
float start_angle = 0.0f;

/* from Protocol/navdata.c */
void initialize_feedback() {
    start_position = accumX;
    start_angle = gyroz;
}

/*
 * distance >= 0 
 */
float do_feedback_forward(double distance_target) {
    //distance_travelled += fabs(vx) * dt;
    float distance_travelled = accumX - start_position;
    printf("accumX: %f, distance_traveled: %f\n", accumX, distance_travelled);
    // if we haven't reached the target distance, keep going
    // and proportionally adjust the rotor velocity
    if (fabs(distance_target - distance_travelled) > DISTANCE_BIAS) {
        puts("going forward");
        return (distance_target - distance_travelled)/20000 * -1;
    }
    else {
        puts("done moving forward");
        return 0.0f;
    }
}

/*
 * angle_target > 0 for turning left
 * angle_target < 0 for turning right
 */
float do_feedback_turn(double phi_target) {
    float angle_turned = gyroz - start_angle;
    printf("gyrox: %f, gyroy: %f, gyroz: %f, angle_turned: %f\n",
           gyrox, gyroy, gyroz, angle_turned);

    // if we haven't reached the target angle, keep going
    // and proportionally adjust the rotor velocity
    
    // turn left
    if (phi_target > 0 && phi_target - angle_turned > ANGLE_BIAS) {
        puts("turning left");
        return (phi_target - angle_turned)/300;  //1 - angle_turned / phi_target
    }
    
    // turn right
    else if (phi_target < 0 && fabs(phi_target) - angle_turned > ANGLE_BIAS) {
        puts("turning right");        
        return (fabs(phi_target) - angle_turned)/300;   //angle_turned / fabs(phi_target) - 1;
    }

    else {
        puts("done turning.");
        return 0.0f;
    }
}

