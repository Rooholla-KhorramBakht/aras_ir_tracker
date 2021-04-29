#ifndef __PLL_CONTROLLER__
#define __PLL_CONTROLLER__

extern unsigned long long frame_timestamp,frame_timestamp_old; //A variable to hold the timestamp of the captured image
extern unsigned long long trigger_timestamp;
extern unsigned long long trigger_timestamp_old;

double saturate_fps_cmd(double numinal_val,double fps);
void trigger_isr(void);
void initilize_pll_controller(double init_ff_fps);

#endif
