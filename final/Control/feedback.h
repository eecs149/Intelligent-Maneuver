#ifndef FEEDBACK_H
#define FEEDBACK_H

void initialize_feedback(double time);

float do_feedback_forward(double distance, double time);
float do_feedback_turn(double phi, double time);

#endif

