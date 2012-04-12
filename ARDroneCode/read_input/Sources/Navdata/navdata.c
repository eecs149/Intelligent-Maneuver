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

        // stream to memdb
        db_printf(db, "navdata", "%u,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                  nd_time->time,
                  nd->vx, nd->vy, nd->vz,
                  nfr->phys_accs[ACC_X], nfr->phys_accs[ACC_Y], nfr->phys_accs[ACC_Z],
                  nfr->phys_gyros[GYRO_X], nfr->phys_gyros[GYRO_Y], nfr->phys_gyros[GYRO_Z]);

        
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

