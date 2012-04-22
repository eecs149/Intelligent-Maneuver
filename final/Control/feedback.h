#ifndef FEEDBACK_H
#define FEEDBACK_H

enum DroneMovement
{
    FLY_FORWARD,    // positive x
    FLY_BACKWARD,   // negative x
    
    FLY_LEFT,       // positive y
    FLY_RIGHT,      // negative y
    
    FLY_UP,         // positive z
    FLY_DOWN,       // negative z
    
    TURN_LEFT,      // positive angle
    TURN_RIGHT,     // negative angle
    
    HOVER
};

struct vector_t
{
    float distance;
    int direction_toggle;   // 0 for x, 1 for y, 2 for z (this is sort of ghetto)
};

void initialize_feedback();

DroneMovement process_feedback(vector_t vector, unsigned long time);

/*
 * dist_target > 0 for flying forward
 * dist_target < 0 for flying backward
 */
DroneMovement fly_target_distance_x(float dist_target, unsigned long time);

/*
 * dist_target > 0 for flying left
 * dist_target < 0 for flying right
 */
DroneMovement fly_target_distance_y(float dist_target, unsigned long time);

/*
 * dist_target > 0 for flying up
 * dist_target < 0 for flying down
 */
DroneMovement fly_target_distance_z(float dist_target, unsigned long time);

/* TODO: probably don't need this, since we have fly x-distance & fly y-distance
 * angle_target > 0 for turning left
 * angle_target < 0 for turning right
 */
//DroneMovement turn_target_distance(float angle_target);

#endif

