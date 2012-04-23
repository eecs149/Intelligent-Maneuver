#ifndef FEEDBACK_H
#define FEEDBACK_H

void initialize_feedback();

float do_feedback_forward(double distance, unsigned long time);
float do_feedback_turn(double phi, unsigned long time);

#endif

