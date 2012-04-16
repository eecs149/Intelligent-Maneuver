#ifndef FEEDBACK_H
#define FEEDBACK_H


void initialize_feedback();

/*
 * @param: dist_target should be "target_x" or "target_y"
 */
void fly_target_distance(float dist_target);

/*
 * @param: angle_target should be "target_angle"
 */
void turn_target_distance(float angle_target);

#endif

