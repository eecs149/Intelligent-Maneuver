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
        fprintf(stderr, "USAGE: <tty> <db port>");
        return -1;
    }

    db_t db = db_connect(argv[2]);
    if (db == -1)
    {
        fprintf(stderr, "readultrasonic failed to connect to memdb.");
        return -1;
    }

    struct termios attrib; int fd = open(argv[1], O_RDONLY);
    if (tcgetattr(fd, &attrib) < 0)
    {
        close(fd);
        fprintf(stderr, "Failed to get attributes");
        return -1;
    }
    if (cfsetispeed(&attrib, B9600) < 0 || cfsetospeed(&attrib, B9600) < 0)
    {
        close(fd);
        fprintf(stderr, "Failed to set speed");
        return -1;
    }
    if (tcsetattr(fd, TCSAFLUSH, &attrib) < 0)
    {
        close(fd);
        fprintf(stderr, "Failed to apply config");
        return -1;
    }

//    FILE *dump = fopen("foo.dump", "w");

    float distances[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int count = 0;
    for (;;)
    {
        char line[32] = "";
        int readsize = read(fd, line, sizeof(line));
        if (readsize <= 2)
            continue;
        if (!isalpha(line[0]))
            continue;

        char sensorId = line[0];
        float distance = atoi(line+1) / 5510.0f;

        if (distances[sensorId - 'A'] == 0.0f)
            count++;
        distances[sensorId - 'A'] = distance;
//        fprintf(dump, "%s", line); fflush(dump);

        // send to memdb
        if (count == 4)
        {
            db_printf(db, "ultrasonic", "%f,%f,%f,%f", 
                      distances[0], distances[1], distances[2], distances[3]);
            count = 0;
            memset(distances, 0, sizeof(distances));
        }
    }
}

