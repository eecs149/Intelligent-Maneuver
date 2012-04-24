#include <ardrone_tool/Navdata/ardrone_navdata_client.h>

#include <Navdata/navdata.h>
#include <memdb.h>
#include <stdio.h>

int db;
/* Initialization local variables before event loop  */
inline C_RESULT demo_navdata_client_init( void* data )
{
    db = db_connect("8765");
    return C_OK;
}

/* Receving navdata during the event loop */
inline C_RESULT demo_navdata_client_process( const navdata_unpacked_t* const navdata )
{
    const navdata_demo_t*nd = &navdata->navdata_demo;
    const navdata_time_t*nd_time = &navdata->navdata_time;
    const navdata_phys_measures_t* nfr = &navdata->navdata_phys_measures;
    char buffer[1024];
    int hover = 0; // boolean indicating whether it's hovering or not
    float phi; // left/right angle
    float theta; // front/back angle
    float gaz; // verticle speed
    float yaw; // angular speed

    // stream to memdb
    db_printf(db, "navdata", "%u,%f,%f,%f,%d,%f,%f,%f,%f,%f",
              nd_time->time,
              nd->vx, nd->vy, nd->vz,
              nfr->phys_accs[ACC_X], nfr->phys_accs[ACC_Y], nfr->phys_accs[ACC_Z],
              nfr->phys_gyros[GYRO_X], nfr->phys_gyros[GYRO_Y], nfr->phys_gyros[GYRO_Z]);

    // read from memdb and send to ardrone
    while (db_tryget(db, "drone_command", buffer, sizeof(buffer)) != -1)
    {
        sscanf(buffer, "%d,%f,%f,%f,%f", &hover, &phi, &theta, &gaz, &yaw);
        if (hover)
            ardrone_at_set_progress_cmd(1, phi, theta, gaz, yaw);
        else
            ardrone_at_set_progress_cmd(0, 0, 0, 0, 0);
    }


    return C_OK;
}

/* Relinquish the local resources after the event loop exit */
inline C_RESULT demo_navdata_client_release( void )
{
    db_close(db);
    return C_OK;
}

/* Registering to navdata client */
BEGIN_NAVDATA_HANDLER_TABLE
  NAVDATA_HANDLER_TABLE_ENTRY(demo_navdata_client_init, demo_navdata_client_process, demo_navdata_client_release, NULL)
END_NAVDATA_HANDLER_TABLE

