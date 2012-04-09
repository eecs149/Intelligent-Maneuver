#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <pthread.h>

#include <mrpt/utils.h>

#include "app.h" 

extern float32_t angles[3];
extern float32_t velocities[3];

extern pthread_mutex_t mutex;

int main( int argc, char* argv[] )
{
   appInit();


   while (1) {
   }

   return 0;
}

