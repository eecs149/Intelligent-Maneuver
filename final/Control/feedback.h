#ifndef FEEDBACK_H
#define FEEDBACK_H

void initialize_feedback();

float do_feedback_forward(double distance, double dt);
float do_feedback_turn(double phi, double dt);

#endif

