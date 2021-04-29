#include <math.h>

float distanceCalc(float Px,float Py,float cx,float cy)
{
 float dx=Px-cx;
 float dy=Py-cy;
 return sqrt(dx*dx+dy*dy);
}

void itterativeMean(float oldMeanX,float oldMeanY,float x,float y,int i,int *cnt, float *outx,float *outy)
{
	*outx=((float)cnt[i]*oldMeanX+x)/((float)cnt[i]+1);
	*outy=((float)cnt[i]*oldMeanY+y)/((float)cnt[i]+1);
     cnt[i]=cnt[i]+1;
}
float myabs(float in)
{
	if(in<0)
		return -in;
	else
		return in;
}
int ProcessData(float *cx,float *cy,float *Rx,float *Ry ,float Px ,float Py ,int nCusters,int *cnt,int Threshol)
{
	float distance,x,y;
	int assignedFlag=0,index;

	for(int i=0;i<nCusters;i++)
	{
		if(cx[i]!=-1)
		{
			distance=distanceCalc(Px,Py,cx[i],cy[i]);
			if(distance<Threshol)
			{
				float Rx_new=myabs(Px-cx[i]);
				float Ry_new=myabs(Py-cy[i]);

				itterativeMean(cx[i],cy[i],Px,Py,i,cnt, &x,&y);
				cx[i]=x;
				cy[i]=y;
				index=i;
				if(Rx[i]<Rx_new)
					Rx[i]=Rx_new;
				if(Ry[i]<Ry_new)
					Ry[i]=Ry_new;
				assignedFlag=1;
			}
		}
	}

	if(assignedFlag==0)
	{
		for(int j=0;j<nCusters;j++)
		{
			if(cx[j]==-1)
			{
				cx[j]=Px;
				cy[j]=Py;
				index=j;
				assignedFlag=1;
				break;
			}
		}
	}
	if(assignedFlag==1)
		return index;
	else
		return -1;
}
