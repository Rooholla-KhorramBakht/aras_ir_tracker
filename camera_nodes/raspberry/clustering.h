#ifndef __CLUSTERING__
#define __CLUSTERING__

float distanceCalc(float Px,float Py,float cx,float cy);
void itterativeMean(float oldMeanX,float oldMeanY,float x,float y,int i,int *cnt, float *outx,float *outy);
float myabs(float in);
int ProcessData(float *cx,float *cy,float *Rx,float *Ry ,float Px ,float Py ,int nCusters,int *cnt,int Threshol);

#endif
