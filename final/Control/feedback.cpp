#include <math.h>
#include "feedback.h"


/* prototype code for flying a desired distance or a desired angle */
/* flying direction states - TODO: define these */
typedef enum{
	FLY_RIGHT,      // positive x
	FLY_LEFT,       // negative x
	FLY_FORWARD,    // positive y
	FLY_BACKWARD    // negative y
} flyDirectionState_t;

/* turning direction states - TODO: define these */
typedef enum{
	TURN_RIGHT,     // positive angle
	TURN_LEFT       // negative angle
} turnDirectionState_t;


/* constants */
#define DIST_BIAS  5	// in mm
#define ANGLE_BIAS 10 	// units?
#define TIME_STEP  2	// in ms; set to 1 or delete if this becomes a waste of space in memory

/* variables */
unsigned long start_time;

float target_x;           // TODO: set this somewhere
float target_y;           // TODO: set this somewhere
float target_angle;       // TODO: set this somewhere (yaw = psi = angles[3])

/* from Protocol/navdata.c */
float angles[3];
float velocities[3];


float dist_travelled;
float angle_turned;


void initialize_feedback(unsigned long time) {
    start_time = time;
    dist_travelled = 0;
    angle_turned = 0;
}

/*
 * @param: dist_target should be "target_x" or "target_y"
 */
void fly_target_distance(float dist_target, unsigned long time) {
    // TODO: make drone fly in a direction based on flyDirectionState_t

    do {
        if (time % TIME_STEP == 0) {
            if (FLY_RIGHT || FLY_LEFT) {
                dist_travelled = fabs(velocities[0]) * (time - start_time);
            }
            else {
                dist_travelled = fabs(velocities[1]) * (time - start_time);
            }
        }

        // TODO: make drone stop and hover here
    } while (fabs(fabs(dist_target) - dist_travelled) < DIST_BIAS);
}

/*
 * @param: angle_target should be "target_angle"
 */
void turn_target_distance(float angle_target, unsigned long time) {
    // TODO: make drone turn in a direction based on turnDirectionState_t

    do {
        if (time % TIME_STEP == 0) {
            angle_turned = angles[3];
        }
    } while (fabs(angle_target - angle_turned) < ANGLE_BIAS);;

    // TODO: make drone stop and hover here
}

