#ifndef FEEDBACK_H
#define FEEDBACK_H

enum DroneMovement {
	FLY_FORWARD,    // positive x
	FLY_BACKWARD,   // negative x
	FLY_LEFT,       // positive y
	FLY_RIGHT,      // negative y
	TURN_LEFT,      // negative angle
	TURN_RIGHT,     // positive angle
	HOVER
};

void initialize_feedback();

/*
 * @param: dist_target > 0 for flying forward
 *         dist_target < 0 for flying backward
 */
DroneMovement fly_target_distance_x(float dist_target, unsigned long time);

/*
 * @param: dist_target > 0 for flying left
 *         dist_target < 0 for flying right
 */
DroneMovement fly_target_distance_y(float dist_target, unsigned long time);

/*
 * @param: angle_target > 0 for turning left
 *         angle_target < 0 for turning right
 */
DroneMovement turn_target_distance(float angle_target);

#endif

