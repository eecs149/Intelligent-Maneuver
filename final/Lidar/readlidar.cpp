#include <stdio.h>
#include "../memdb/memdb.h"


struct LidarPacket
{
    unsigned char FA;
    unsigned char sequence;
    unsigned short speed;
    struct
    {
        unsigned short distance;
        unsigned short surface;
    } measurements[4];
};


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

        // compute and validate checksum
        unsigned int data_list[10];
        for (int i = 0; i < 10; ++i)
            data_list[i] = lidarPacket[i*2] + (lidarPacket[i*2+1] << 8);
        unsigned int chk32 = 0;
        for (int i = 0; i < 10; ++i)
            chk32 = (chk32 << 1) + data_list[i];
        unsigned checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
        checksum &= 0x7FFF;
        unsigned short expectedChecksum = *((unsigned short*)&lidarPacket[20]);
        if (checksum != expectedChecksum)
            continue;

        LidarPacket packet;
        memcpy(&packet, lidarPacket, sizeof(packet));

        if (packet.sequence == 0xa0) {
        printf("%x, %x\n%x\t%x\n%x\t%x\n%x\t%x\n%x\t%x\n",
               packet.sequence, packet.speed,
               packet.measurements[0].distance, packet.measurements[0].surface,
               packet.measurements[1].distance, packet.measurements[1].surface,
               packet.measurements[2].distance, packet.measurements[2].surface,
               packet.measurements[3].distance, packet.measurements[3].surface);
        puts("=========================");
        }

        // send to memdb
        unsigned *p = reinterpret_cast<unsigned*>(&lidarPacket);
        db_printf(db, "lidar", "%x,%x,%x,%x,%x", p[0], p[1], p[2], p[3], p[4]);
    }
}

