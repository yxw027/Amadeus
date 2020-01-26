
#include "typedefine.h"
#include "define.h"
#include "coordinate.h"
#include "constdef.h"
#include <math.h>


void CordTrans_Using7Para(double * x, double * y,double * z, CordTrans7Para* Para)//
{
	//七参数转换通用模型
	//	[x2]		[dx]				[	1		dOmg	-dPhi	][x1]
	//	[y2]	=	[dy]	+	(1 + dS)[	-dOmg	1		dEps	][y1]
	//	[z2]		[dz]				[	dPhi	-dEps	1		][z1]

	double x1 = *x;
	double y1 = *y;
	double z1 = *z;

	double dx = Para->dx;
	double dy = Para->dy;
	double dz = Para->dz;
	double dEps = Para->dEps;
	double dPhi = Para->dPhi;
	double dOmg = Para->dOmg;
	double dS = Para->dS;

	*x = dx + (1+dS) * (		 x1	+ dOmg * y1 - dPhi * z1 );
	*y = dy + (1+dS) * ( -dOmg * x1 +	     y1	+ dEps * z1);
	*z = dz + (1+dS) * (dPhi   * x1	- dEps * y1 +	     z1);
}


int32 WGS2ECEF(WGS *src, ECEF *des)
{
	double n;

 	n = (SEMIMAJOR_AXIS*SEMIMAJOR_AXIS)/sqrt(
		  SEMIMAJOR_AXIS*SEMIMAJOR_AXIS*cos(src->lat)*cos(src->lat) + 
	  		SEMIMINOR_AXIS*SEMIMINOR_AXIS*sin(src->lat)*sin(src->lat) );

	des->x = (n + src->alt)*cos(src->lat)*cos(src->lon);
	des->y = (n + src->alt)*cos(src->lat)*sin(src->lon);
	des->z = ((SEMIMINOR_AXIS*SEMIMINOR_AXIS)/(SEMIMAJOR_AXIS*SEMIMAJOR_AXIS)*n 
	  			+ src->alt ) * sin(src->lat);

	return 0;
}


int32 ECEF2WGS(ECEF *src, WGS *des)
{
	double p=0.0,slat=0.0,N=0.0,htold=0.0,latold=0.0;
	const double eccSq = 0.00669438002290;
	int32  i=0;
	double tmp=0.0;

	//Check postion valiable
	tmp = sqrt(src->x*src->x + src->y*src->y + src->z * src->z);
	if (f_abs(tmp) < SEMIMAJOR_AXIS*0.5)
		goto __err_exit;

	//Check ECEF2WGS change range
	p = sqrt(src->x*src->x + src->y*src->y);
	if(p < MICRO_NUM)
		goto __err_exit;
		
    des->lat = atan2(src->z, p*(1.0-eccSq));
    des->alt = 0;

    for(i=0; i<5; i++)
	{
		slat = sin(des->lat);
		N = SEMIMAJOR_AXIS / sqrt(1.0 - eccSq*slat*slat);
		htold = des->alt;
        	des->alt = p/cos(des->lat) - N;
         
		latold = des->lat;
		des->lat = atan2(src->z, p*(1.0-eccSq*(N/(N+ des->alt ))));
		if(f_abs(des->lat - latold) < 1.0e-9 && 
			f_abs(des->alt - htold) < (1.0e-9 * SEMIMAJOR_AXIS) )
			 break;
     }

	des->lon = atan2(src->y,src->x);
	return 0;

__err_exit:
	des->lon = 0.0;
	des->lat = 0.0;
	des->alt = 0.0;
	return -1;
}

int32 ECEF2NEH(ECEF *src, ECEF *ref, NEH *des)
{
	double lon,lat;
	double dtx,dty,dtz;
	WGS p;
	double dis2EC;

	dis2EC = sqrt(src->x*src->x + src->y*src->y + src->z * src->z);
	if ( (f_abs(dis2EC) < SEMIMAJOR_AXIS*0.5) && (f_abs(dis2EC) > MICRO_NUM) )
		goto __err_exit;

	dis2EC = sqrt(ref->x*ref->x + ref->y*ref->y + ref->z * ref->z);
	if (f_abs(dis2EC) < SEMIMAJOR_AXIS*0.5)
		goto __err_exit;	

	if( ECEF2WGS(ref,&p) != 0 )
		goto __err_exit;

	lon = p.lon;
	lat = p.lat;
	
	dtx = src->x - ref->x;
	dty = src->y - ref->y;
	dtz = src->z - ref->z;

	des->north = -sin(lat)*cos(lon)*dtx - sin(lon)*sin(lat)*dty + cos(lat)*dtz;
	des->east = -sin(lon)*dtx + cos(lon)*dty;
	des->head = cos(lat)*cos(lon)*dtx + cos(lat)*sin(lon)*dty + sin(lat)*dtz;

	return 0;
	
__err_exit:
	des->north = 0.0;
	des->east = 0.0;
	des->head = 0.0;
	return -1;
}

/**
*	@brief	transfer velocity from ECEF to speed & heading.
*	@param	pPos: receiver position pointer
*			pVel: receiver velocity pointer
*			pSpeed: speed pointer
*			pHeading: heading pointer
*	@return  TRUE: transfer success, FALSE: transfer FALSE
*	@author	Juan.gou
*	@date	Aug 17, 2015
*/
boolean ECEFVel2SpeedHeading(ECEF* pPos, ECEF* pVel, double* pSpeed, double* pHeading)
{
	double phi, lambda, Vnorth, Veast;
	WGS wgspos;
	
	if(ECEF2WGS(pPos, &wgspos) != 0)
		return FALSE;

	phi = wgspos.lat;
	lambda = wgspos.lon;

	Vnorth = -pVel->x*sin(phi)*cos(lambda) - pVel->y*sin(phi)*sin(lambda) + pVel->z*cos(phi);
	Veast = -pVel->x*sin(lambda) + pVel->y*cos(lambda);
	
	*pSpeed = sqrt(Vnorth*Vnorth + Veast*Veast);
	*pHeading = atan2(Veast, Vnorth);
	
	return TRUE;
}


boolean ECEFVel2ENU(ECEF* pPos, ECEF* pVel, NEH* pNEHVel)
{
	double phi, lambda;
	double sinlambda=0.0, coslambda=0.0, sinphi=0.0, cosphi=0.0;
	WGS wgspos;

	if(ECEF2WGS(pPos, &wgspos) != 0)
		return FALSE;

	phi = wgspos.lat;
	lambda = wgspos.lon;

	sinlambda = sin(lambda);
	coslambda = cos(lambda);
	sinphi = sin(phi);
	cosphi = cos(phi);

	pNEHVel->east = -pVel->x*sinlambda + pVel->y*coslambda;
	pNEHVel->north = -pVel->x*sinphi*coslambda - pVel->y*sinphi*sinlambda + pVel->z*cosphi;
	pNEHVel->head =  pVel->x*cosphi*coslambda + pVel->y*cosphi*sinlambda + pVel->z*sinphi;

	return TRUE;
}


double Dis2PointsOnEarth(ECEF *pPos1, ECEF *pPos2)
{
	const double BIG_DISTANCE=1.0e100;
	double distance=0.0;
	
	WGS wgs1, wgs2;

	if(ECEF2WGS(pPos1, &wgs1) != 0)
		return BIG_DISTANCE;
	
	if(ECEF2WGS(pPos2, &wgs2) != 0)
		return BIG_DISTANCE;

	distance = Dis2PointsOnEarthWGS(&wgs1, &wgs2);

	return distance;
}


double Dis2PointsOnEarthWGS(WGS *pWGS1, WGS *pWGS2)
{
	double r = 6.371229*1e6;	//nearly earth radius, unit meter
	double cos_angle = sin(pWGS1->lat) * sin(pWGS2->lat) + cos(pWGS1->lat) * cos(pWGS2->lat) * cos(pWGS2->lon - pWGS1->lon);
	double distance;

	if(cos_angle >= 1.0)
		cos_angle = 1.0;
	else if(cos_angle <= -1.0)
		cos_angle = -1.0;
	
	distance =  r * acos(cos_angle);
	
	return distance;
}

int32 errcnt=0;
double sinx(double x)
{
	double result=x,temp=x;
   double den=x,fac=1;
   int n=1,sign=1;

   int inta = (int)(x/(2*PI));
   x = x-inta*2*PI;

   result=temp=den=x;

   while((temp>1e-15)||(temp<-1e-15))        
   {
       n++;fac*=n;den*=x;
       n++;fac*=n;den*=x;
       temp=den/fac;sign=-sign;
       result=sign>0?result+temp:result-temp;
   }

   if(fabs(result)<1e-20)
   	errcnt++;

   return result;
}

double cosx(double x)
{
	double result=1.0,temp=1.0;
   double den=1,fac=1;
   int n=0,sign=1;

   int inta = (int)(x/(2*PI));
   x = x-inta*2*PI;

   while((temp>1e-15)||(temp<-1e-15))        
   {
       n++;fac*=n;den*=x;
       n++;fac*=n;den*=x;
       temp=den/fac;sign=-sign;
       result=sign>0?result+temp:result-temp;
   }

   if(fabs(result)<1e-20)
   	errcnt++;

   return result;
}


void ClearBitWord64(word64* wd, byte i)
{
	wd->wd[i/32] &= ~(1<<( i%32));

	return;
}

void SetBitWord64(word64* wd, byte i)
{
	wd->wd[i/32] |= (1<<( i%32));

	return;
}

byte GetBitWord64(word64 wd, byte i)
{
	byte bit = 0;
	byte wdid = i/32;
	if(wd.wd[wdid] & (1<<(i%32)))
		bit = 1;

	return bit;
}

boolean IsZeroInWord64(word64 *wd)
{
	if((wd->wd[0] == 0) && (wd->wd[1] == 0))
		return TRUE;
	else
		return FALSE;
}

void ResetWord64(word64 *wd)
{
	wd->wd[0]=0;
	wd->wd[1]=0;

	return;
}

word64 OpAndWord64(word64 *wd1,word64 *wd2)
{
	word64 andop = {{0, 0}};
	andop.wd[0] = wd1->wd[0] & wd2->wd[0];
	andop.wd[1] = wd1->wd[1] & wd2->wd[1];
	
	return andop;
}

word64 OpOrWord64(word64 *wd1,word64 *wd2)
{
	word64 andop = {{0, 0}};
	andop.wd[0] = wd1->wd[0] | wd2->wd[0];
	andop.wd[1] = wd1->wd[1] | wd2->wd[1];
	
	return andop;
}


void ClearBitWord256(word256* wd, byte i)
{
	wd->wd[i/32] &= ~(1<<( i%32));

	return;
}

void SetBitWord256(word256* wd, byte i)
{
	wd->wd[i/32] |= (1<<( i%32));

	return;
}

byte GetBitWord256(word256* wd, byte i)
{
	byte bit = 0;
	byte wdid = i/32;
	if(wd->wd[wdid] & (1<<(i%32)))
		bit = 1;

	return bit;
}

boolean IsZeroInWord256(word256 *wd)
{
	int32 i=0;
	for(i=0;i<8;i++)
	{
		if(wd->wd[i] != 0)
			return FALSE;
	}

	return TRUE;
}

void ResetWord256(word256 *wd)
{
	memset(wd, 0, sizeof(word256));

	return;
}

word256 OpAndWord256(word256 *wd1,word256 *wd2)
{
	word256 andop = {{0,}};
	int32 i=0;
	for(i=0;i<8;i++)
		andop.wd[i] = wd1->wd[i] & wd2->wd[i];
	
	return andop;
}

word256 OpOrWord256(word256 *wd1,word256 *wd2)
{
	word256 orop = {{0,}};
	int32 i=0;
	for(i=0;i<8;i++)
		orop.wd[i] = wd1->wd[i] | wd2->wd[i];
	
	return orop;
}

word256 OpReverseWord256(word256* wd)
{
	word256 reversop = {{0,}};
	int32 i=0;
	for(i=0; i<8; i++)
		reversop.wd[i] = ~(wd->wd[i]);

	return reversop;
}

int8 CalcBitcntWord256(word256* wd, word32 bitvalid)
{
	int8 bitcnt=0;
	int32 i=0,wdidx=0;

	for(wdidx=0; wdidx<8; wdidx++)
	{
		for(i=0; i<32; i++)
		{
			if(((wd->wd[wdidx]>>i) & 0x1)==bitvalid)
				bitcnt++;
		}
	}

	return bitcnt;
}


int8 FindBitWord16(word16 wd, word16 bitvalue)
{
	int32 i=0;
	for(i=0; i<16; i++)
	{
		if(((wd>>i) & 0x1)==bitvalue)
			return i;
	}
	return -1;
}

int8 CalcBitcntWord32(word32 wd, word32 bitvalid)
{
	int8 bitcnt=0;
	int32 i=0;

	for(i=0; i<32; i++)
	{
		if(((wd>>i) & 0x1)==bitvalid)
			bitcnt++;
	}

	return bitcnt;
}


/**
*	@brief		convert high32 bit + low 32 bit data to a double value
*	@RETURN		none
*	@AUTHOR		haiquan
*   @DATE		Apr 7, 2006
*/
void F64ToH32L32(float64 clk, word32 *high, word32 *low)
{
	word32 l,h;

	h = (word32)(clk / TWO_P32);
	l = (word32)(clk - h * TWO_P32 + 0.5);
	
	*low = l;
	*high = h;
}


/**
*	@brief		convert a double value to high32 bit + low 32 bit data
*	@RETURN		none
*	@AUTHOR		haiquan
*   @DATE		Apr 7, 2006
*/
void H32L32ToF64(float64 *ret, word32 high, word32 low)
{
	*ret =(double)high * TWO_P32 + (double)low;

}

double SGN(double x)
{
	return (x<=0.0)?-1.0:1.0;
}
double ROUND(double x)
{
	return floor(x+0.5);
}

unsigned int ROUND_U(double x)
{
	return (unsigned int)floor(x+0.5);
}

void SWAP(double* x,double* y)
{
	double tmp_; 
	tmp_=x[0];	x[0]=y[0]; 	y[0]=tmp_;
}




