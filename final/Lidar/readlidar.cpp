#include <stdio.h>
#include "../memdb/memdb.h"
#include <termios.h>
#include <fcntl.h>


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
        fprintf(stderr, "USAGE: <lidar tty> <db port>");
        return -1;
    }

    struct termios attrib;
    int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);
    if (tcgetattr(fd, &attrib) < 0) {
        close(fd);
        fprintf(stderr, "Failed to get attributes");
        return -1;
    }
    if (cfsetispeed(&attrib, B115200) < 0 || cfsetospeed(&attrib, B115200) < 0) {
        close(fd);
        fprintf(stderr, "Failed to set speed");
        return -1;
    }
    if (tcsetattr(fd, TCSAFLUSH, &attrib) < 0) {
        close(fd);
        fprintf(stderr, "Failed to apply config");
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
        unsigned char val = 0;
        while (val != 0xFA) {
            read(fd, &val, 1);
        }

        // read the whole packet
        unsigned char lidarPacket[22];
        lidarPacket[0] = 0xFA;
        read(fd, lidarPacket+1, sizeof(lidarPacket)-1);

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

        printf("%x, %x\n%x\t%x\n%x\t%x\n%x\t%x\n%x\t%x\n",
               packet.sequence, packet.speed,
               packet.measurements[0].distance, packet.measurements[0].surface,
               packet.measurements[1].distance, packet.measurements[1].surface,
               packet.measurements[2].distance, packet.measurements[2].surface,
               packet.measurements[3].distance, packet.measurements[3].surface);
        puts("=========================");

        // send to memdb
        unsigned *p = reinterpret_cast<unsigned*>(&lidarPacket);
        db_printf(db, "lidar", "%x,%x,%x,%x,%x", p[0], p[1], p[2], p[3], p[4]);
    }
}

