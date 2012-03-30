#include <ardrone_tool/Navdata/ardrone_navdata_client.h>

#include <Navdata/navdata.h>
#include <stdio.h>

volatile float32_t accels[3];
volatile float32_t vels[3];
volatile float32_t angles[3];

/* Initialization local variables before event loop  */
inline C_RESULT demo_navdata_client_init( void* data )
{
  return C_OK;
}

/* Receving navdata during the event loop */
inline C_RESULT demo_navdata_client_process( const navdata_unpacked_t* const navdata )
{
	const navdata_demo_t*nd = &navdata->navdata_demo;
        const navdata_phys_measures_t* const nfr = &navdata->navdata_phys_measures;
        //accels[0] = nfr->phys_accs[0];
        //accels[1] = nfr->phys_accs[1];
        //accels[2] = nfr->phys_accs[2];
        //angles[0] = nd->theta;
        //angles[1] = nd->phi;
        //angles[2] = nd->psi;
        //vels[0] = nd->vx;
        //vels[1] = nd->vy;
        //vels[2] = nd->vz;
        
        FILE *file;
        file = fopen("sensor_data_test.txt","a+");

	//printf("=====================\nNavdata for flight demonstrations =====================\n\n");

	//printf("Control state : %i\n",nd->ctrl_state);
	//printf("Battery level : %i mV\n",nd->vbat_flying_percentage);
	//printf("Orientation   : [Theta] %4.3f  [Phi] %4.3f  [Psi] %4.3f\n",nd->theta,nd->phi,nd->psi);
	//printf("Altitude      : %i\n",nd->altitude);
	//printf("Speed         : [vX] %4.3f  [vY] %4.3f  [vZPsi] %4.3f\n",nd->theta,nd->phi,nd->psi);
        //printf("Accels        : %4.3f, %4.3f, %4.3f\n", nfr->phys_accs[0], nfr->phys_accs[1], nfr->phys_accs[2]);
        //printf("Gyros         : %4.3f, %4.3f\n", nfr->phys_gyros[0], nfr->phys_gyros[1]);
	fprintf(file, "Orientation   : [Theta] %4.3f  [Phi] %4.3f  [Psi] %4.3f\n",nd->theta,nd->phi,nd->psi);
	fprintf(file, "Altitude      : %i\n",nd->altitude);
	fprintf(file, "Speed         : [vX] %4.3f  [vY] %4.3f  [vZ] %4.3f\n",nd->vx,nd->vy,nd->vz);
        fprintf(file, "Accels        : %4.3f, %4.3f, %4.3f\n", nfr->phys_accs[0], nfr->phys_accs[1], nfr->phys_accs[2]);
        fprintf(file, "Gyros         : %4.3f, %4.3f\n", nfr->phys_gyros[0], nfr->phys_gyros[1]);

        fclose(file);

	//printf("\033[8A");
        
          return C_OK;
}

/* Relinquish the local resources after the event loop exit */
inline C_RESULT demo_navdata_client_release( void )
{
  return C_OK;
}

/* Registering to navdata client */
BEGIN_NAVDATA_HANDLER_TABLE
  NAVDATA_HANDLER_TABLE_ENTRY(demo_navdata_client_init, demo_navdata_client_process, demo_navdata_client_release, NULL)
END_NAVDATA_HANDLER_TABLE

