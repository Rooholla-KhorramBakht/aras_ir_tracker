#include <ctype.h>
#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "interface/vcos/vcos.h"
#include "bcm_host.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_connection.h"

#include "RaspiCLI.h"

#include <sys/ioctl.h>

#include "raw_header.h"

#include <pthread.h>
#include <wiringPi.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <math.h>

typedef struct {
	int threshold_level;
	int clustering_threshold_level;
	double kp;
	double kd;
	double ki;
  double camera_fps;
}MARKER_EXTRACTOR_PARAMS_T;

extern MARKER_EXTRACTOR_PARAMS_T marker_extractor_params;

double feed_forward_term;
double fps_error,fps_error_old,fps_error_integral;
unsigned long long frame_timestamp=0,frame_timestamp_old=0; //A variable to hold the timestamp of the captured image
unsigned long long trigger_timestamp=0;
unsigned long long trigger_timestamp_old=0;
int sync_pll_cnt=0;
void set_fps_ov5647(double fps);

void initilize_pll_controller(double init_ff_fps)
{
  feed_forward_term=init_ff_fps;
}

double saturate_fps_cmd(double numinal_val,double fps)
{
	if(fps>numinal_val+5)
	  	return numinal_val+5;
	else if(fps<numinal_val-5)
		return numinal_val-5;
	else
		return fps;
}

void trigger_isr(void) {
	sync_pll_cnt++;
	trigger_timestamp_old=trigger_timestamp;
	trigger_timestamp=vcos_getmicrosecs();
	//Low Pass filter on the feed forward term (the computer frequrency using the intrrupts is noisy)
	feed_forward_term=0.95*feed_forward_term+0.1*1000000.0/(double)(trigger_timestamp-trigger_timestamp_old)/2;
	// vcos_log_error("Freq= %3.2f",feed_forward_term);

	//Compute the phase difference of frames with respect to the sync
  // VSYNC: -------|-----------|-----------|-----------
	// Trigger: --------|-----------|-----------|-----------
	// Phase error: -0+
	//When the error is positive, frames are coming prior to triggers
	double a = abs(trigger_timestamp-(double)frame_timestamp-(1000000.0/marker_extractor_params.camera_fps));
	double b = abs(trigger_timestamp-(double)frame_timestamp);
	double fps_error;
	static double fps_error_sum;
	if(a<b)
		 fps_error = (trigger_timestamp-(double)frame_timestamp-(1000000.0/marker_extractor_params.camera_fps))/1000000.0;
	else
		 fps_error = (trigger_timestamp-(double)frame_timestamp)/1000000.0;



	//Run the controller once every second
	 if(sync_pll_cnt>marker_extractor_params.camera_fps/10) //Run the loop once every second
	 {
		 fps_error=fps_error_sum/(double)sync_pll_cnt;
		 sync_pll_cnt=0;
		 fps_error_sum=0;
		 double fps_error_d=(fps_error-fps_error_old)*marker_extractor_params.camera_fps;
		 fps_error_old=fps_error;
		 fps_error_integral+=fps_error;
		 double cmd_delta=fps_error*marker_extractor_params.kp+
			 													 fps_error_d*marker_extractor_params.kd+
																 fps_error_integral*marker_extractor_params.ki;
		 //The negative sign for the error delta indicates that when frames come prior to triggers (positive error)
		 //,frames should slow down for the two to catch up
		 double cmd=saturate_fps_cmd(feed_forward_term,feed_forward_term-cmd_delta);
		 set_fps_ov5647(cmd);
		 vcos_log_error("PLL Controller Triggered, error= %3.2f ms\t cmd=%3.2f",fps_error*1000,cmd);
	 }
	 else
	 	fps_error_sum=fps_error+fps_error_sum;
 }
