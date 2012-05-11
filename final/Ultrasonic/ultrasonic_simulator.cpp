#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include "../memdb/memdb.h"


int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        fprintf(stderr, "USAGE: <file> <db port>");
        return -1;
    }

    db_t db = db_connect(argv[2]);
    if (db == -1)
    {
        fprintf(stderr, "simulatlr failed to connect to memdb.");
        return -1;
    }

    FILE *file = fopen(argv[1], "r");

    timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 100000000;

    float distances[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int count = 0;
    for (;;)
    {
        char line[32] = "";
        fgets(line, sizeof(line), file);

        char sensorId = line[0];
        float distance = atoi(line+1) / 5510.0f;

        if (distances[sensorId - 'A'] == 0.0f)
            count++;
        distances[sensorId - 'A'] = distance;

        // send to memdb
        if (count == 4)
        {
            db_printf(db, "ultrasonic", "%f,%f,%f,%f", 
                      distances[0], distances[1], distances[2], distances[3]);
            count = 0;
            memset(distances, 0, sizeof(distances));
        }

        nanosleep(&ts, NULL);
    }
}

