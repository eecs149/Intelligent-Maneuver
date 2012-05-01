#include <stdio.h>

#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <Navdata/navdata.h>

#include "../../../memdb/memdb.h"

#include <time.h>

int db;
/* Initialization local variables before event loop  */
inline C_RESULT demo_navdata_client_init( void* data )
{
    db = db_connect("8765");
    return C_OK;
}

unsigned prevMicroseconds = 0;

/* Receving navdata during the event loop */
inline C_RESULT demo_navdata_client_process( const navdata_unpacked_t* const navdata )
{
    const navdata_demo_t*nd = &navdata->navdata_demo;
    const navdata_time_t*nd_time = &navdata->navdata_time;
    const navdata_phys_measures_t* nfr = &navdata->navdata_phys_measures;
    char buffer[1024];

    unsigned seconds = nd_time->time >> 21;
    unsigned microseconds = nd_time->time & 0x1FFFFF;
    microseconds += seconds * 1e6;

    unsigned dt = 0;
    if (prevMicroseconds != 0)
        dt = microseconds - prevMicroseconds;
    prevMicroseconds = microseconds;

    // stream to memdb
    db_printf(db, "navdata", "%u,%4.3f,%4.3f,%4.3f,%f,%f,%f",
              dt,
              nd->vx, nd->vy, nd->vz,
              nd->theta, nd->phi, nd->psi);
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

