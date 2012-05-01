#include <stdio.h>
#include "../memdb/memdb.h"

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        fprintf(stderr, "incorrect number of arguments.");
        return -1;
    }

    FILE *lidarFile = fopen(argv[1], "rb");
    if (!lidarFile)
    {
        fprintf(stderr, "failed to open port for lidar readings: %s\n", 
                argv[1]);
        return -1;
    }

    db_t db = db_connect(argv[2]);
    if (db == -1)
    {
        fprintf(stderr, "failed to connect to memdb.");
        return -1;
    }

    for (;;)
    {
        // loop until it has read 0xFA
        while (fgetc(lidarFile) != 0xFA);

        // read the whole packet
        unsigned char lidarPacket[22];
        lidarPacket[0] = 0xFA;
        fread(lidarPacket + 1, sizeof(lidarPacket) - 1, 1, lidarFile);

        // compute the checksum
        

        // send to memdb
        unsigned *p = reinterpret_cast<unsigned*>(&lidarPacket);
        db_printf(db, "lidar", "%x,%x,%x,%x,%x", p[0], p[1], p[2], p[3], p[4]);
    }
}

