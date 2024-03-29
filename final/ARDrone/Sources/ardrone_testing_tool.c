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

#include "../../memdb/memdb.h"

#include <time.h>

static int32_t exit_ihm_program = 1;
extern int db;

/* The delegate object calls this method during initialization of an ARDrone application */
C_RESULT ardrone_tool_init_custom(int argc, char **argv)
{
    C_RESULT res;
    // Set drone to "takeoff" mode
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

/* This function is called in the update function, at a refresh rate of 20 ms */
C_RESULT ardrone_tool_update_custom() {
    // read from memdb and send to ardrone
    char buffer[1024];
    int hover = 0; // boolean indicating whether it's hovering or not
    float phi; // left/right angle
    float theta; // front/back angle
    float gaz; // verticle speed
    float yaw; // angular speed

    while (db_tryget(db, "drone_command", buffer, sizeof(buffer)) != -1)
    {
        sscanf(buffer, "%d,%f,%f,%f,%f", &hover, &phi, &theta, &gaz, &yaw);
        if (hover == 1) // hover
        {
            ardrone_at_set_progress_cmd(0, 0, 0, 0, 0);
        }
        else if (hover == -1) // land
        {
            ardrone_tool_set_ui_pad_start(0);
        }
        else // do stuff...
        {
            ardrone_at_set_progress_cmd(1, phi, theta, gaz, yaw);
        }
    }

    return C_OK;
}

/* Implementing thread table in which you add routines of your application and those provided by the SDK */
BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY( ardrone_control, 20 )
  THREAD_TABLE_ENTRY( navdata_update, 20 )
END_THREAD_TABLE

