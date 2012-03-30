/**
 * @file main.c
 * @author sylvain.gaeremynck@parrot.com
 * @date 2009/07/01
 */
#include <ardrone_testing_tool.h>

//ARDroneLib
#include <ardrone_tool/ardrone_time.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/Control/ardrone_control.h>
#include <ardrone_tool/UI/ardrone_input.h>

//Common
#include <config.h>
#include <ardrone_api.h>

//VP_SDK
#include <ATcodec/ATcodec_api.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Os/vp_os_signal.h>

static int32_t exit_ihm_program = 1;

extern volatile float32_t accels[3];
extern volatile float32_t vels[3]; 
extern volatile float32_t angles[3]; 

int count = 0;

/* Implementing Custom methods for the main function of an ARDrone application */

/* The delegate object calls this method during initialization of an ARDrone application */
C_RESULT ardrone_tool_init_custom(int argc, char **argv)
{
    C_RESULT res;
    res = ardrone_tool_set_ui_pad_start(1);
    return res;
}

/* The delegate object calls this method when the event loop exit */
C_RESULT ardrone_tool_shutdown_custom()
{
    return C_OK;
}

/* The event loop calls this method for the exit condition */
bool_t ardrone_tool_exit()
{
  return exit_ihm_program == 0;
}

C_RESULT signal_exit()
{
  exit_ihm_program = 0;

  return C_OK;
}

C_RESULT ardrone_tool_update_custom() {
    count++;
    if (count % 50 == 0) {
        printf("Cycle %d\n", count);
    }
    if (count == 1500) {
        return ardrone_tool_set_ui_pad_start(0);
    }
    if (count >= 1550) {
        return signal_exit();
    }
    //printf("Orientation   : [Theta] %4.3f  [Phi] %4.3f  [Psi] %4.3f\n",angles[0],angles[1],angles[2]);
    //printf("Speed         : [vX] %4.3f  [vY] %4.3f  [vZPsi] %4.3f\n",vels[0],vels[1],vels[2]);
    //printf("Accels        : %4.3f, %4.3f, %4.3f\n",accels[0],accels[1],accels[2]);
    return C_OK;
}

/* Implementing thread table in which you add routines of your application and those provided by the SDK */
BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY( ardrone_control, 20 )
  THREAD_TABLE_ENTRY( navdata_update, 20 )
END_THREAD_TABLE

