/*******************************************************************
Company:   hwacreate
Engineer:  
Create Date: 2015.12.07
File  Name:  RTKTools.c
Description:tool functions for RTK
Function List: 
version: V1.0
Revision Date:
Modifier: 
Additional Comments: 
********************************************************************/
#include <math.h>
#include "RTKTools.h"
#include "constdef.h"

INT16 MONTH_TABLE[] = {0,  31,	59, 90, 120,151,181,212,243,273,304,334,365};

//-----------------------------------------------------------------------------
//鐩殑锛氬皢绌洪棿鍧愭爣XYZ锛圕GS2000锛夎浆鎹㈣嚦澶у湴鍧愭爣BLH
//鍙傛暟锛�
//[IN]	绌洪棿鍧愭爣XYZ
//[OUT]	澶у湴鍧愭爣BLH
//杩斿洖锛氭棤
//-------------------------------------------------------------------------------
void Trans_XYZ_to_BLH(CHAR cSystem, DOUBLE X, DOUBLE Y, DOUBLE Z, DOUBLE* B, DOUBLE* L, DOUBLE* H)
{
    DOUBLE p,n,thet,esq,epsq,temp;
	DOUBLE a, b, f, a2,b2,slat,clat,sth,sth3,cth,cth3; // Save some float math.
	 
	if(fabs(X)<1e-6 && fabs(Y)<1e-6 && fabs(Z)<1e-6)
	{
		*B = 0;
		*L = 0;
		*H = 0;
		return;
	}
	a=GetBDREFParam(BDREF_PARAM_a,cSystem);
	f=GetBDREFParam(BDREF_PARAM_f,cSystem);

	b = a * (1-f);
	a2 = a * a;
	b2 = b * b;

	p = sqrt( X * X + Y * Y);
	thet = atan(Z * a / ( p * b));
	esq = 1.0 - b2 / a2;
	epsq = a2 / b2 - 1.0;

	sth = sin( thet);
	cth = cos( thet);

	sth3 = sth * sth * sth;
	cth3 = cth * cth * cth;

	temp=p - esq * a * cth3;
	if (temp<0)  
	{  
		*B = atan2(-Z - epsq * b * sth3,-temp);
	}
	else
	{
		*B = atan2(Z + epsq * b * sth3,temp);
	}

	*L = atan2( Y, X);

	clat = cos(*B);
	slat = sin(*B);
	n = a2/sqrt(a2 * clat * clat + b2 * slat * slat);
	*H = p / clat - n;
}


//-----------------------------------------------------------------------------
//鐩殑锛氬皢澶у湴鍧愭爣BLH锛圕GS2000锛夎浆鎹㈣嚦绌洪棿鍧愭爣XYZ
//鍙傛暟锛�
//[IN]	澶у湴鍧愭爣BLH
//[OUT]	绌洪棿鍧愭爣XYZ
//杩斿洖锛氭棤
//-------------------------------------------------------------------------------
void Trans_BLH_to_XYZ(CHAR cSystem, DOUBLE B, DOUBLE L, DOUBLE H, DOUBLE *X, DOUBLE *Y, DOUBLE *Z)
{
	DOUBLE N, a, f, e2, sinB, cosB,sinB2;

	a=GetBDREFParam(BDREF_PARAM_a,cSystem);
	f=GetBDREFParam(BDREF_PARAM_f,cSystem);

	e2 = 2*f-f*f;
	sinB = sin(B);
	cosB = cos(B);
	sinB2=sinB*sinB;
	N = a/(sqrt(1-e2*sinB2));

	*X = (N + H)*cosB*cos(L);
	*Y = (N + H)*cosB*sin(L);
	*Z = (N*(1-e2) + H)*sinB;
}


/*--------------------------------------------------------------------------------------
// 鍔熻兘:寰楀埌鍩烘湰鍦扮悆闂�
// 鍙傛暟:
// [IN] nType				鍙傛暟绫诲瀷
// [IN] bGPS				GPS TURE锛孋GS2000 FALSE
// 杩斿洖: 鍙傛暟鍊�
---------------------------------------------------------------------------------------*/
// 寰楀埌鍩烘湰妞悆鍙傛暟
DOUBLE GetBDREFParam(UINT32 nType, CHAR cSystem)
{
	DOUBLE retval = 0;
	
	switch(nType)
	{
		case BDREF_PARAM_a:
			if (cSystem==GLN_SYSTEM)
			{
				retval = GLN_EARTH_A;
				break;
			}
			retval = (cSystem==BD2_SYSTEM)?(CGS2000_A):(WGS84_A);
			break;

		case BDREF_PARAM_f:
			if (cSystem==GLN_SYSTEM)
			{
				retval = GLN_F;
				break;
			}
			retval = (cSystem==BD2_SYSTEM)?(CGS2000_F):(WGS84_F);
			break;

		case BDREF_PARAM_GM:
			if (cSystem==GLN_SYSTEM)
			{
				retval = GLN_MIU;
				break;
			}
			retval = (cSystem==BD2_SYSTEM)?(CGS2000_MIU):(WGS84_MIU);
			break;

		case BDREF_PARAM_w:
			if (cSystem==GLN_SYSTEM)
			{
				retval = GLN_OMIGA_E;
				break;
			}
			retval = (cSystem==BD2_SYSTEM)?(CGS2000_OMEGA_E):(WGS84_OMEGA_E);
			break;
	}
	
	return retval;
}


//杈撳叆XYZ鍩虹嚎鐭㈤噺锛岃緭鍑哄壇澶╃嚎鐩稿涓诲ぉ绾跨殑ENU鍧愭爣
void ECEF2ENU_DeltaXYZ(BD2_BASELINE* baseline, ECEF* pBasePos)
{
	WGS wgspos={0.0,0.0,0.0};
	double dRotMatrix[9] = {0};
	double delx,dely,delz=0.0, base_B, base_L, base_H, dSinLat, dCosLat, dSinLon, dCosLon;
	double base_x,base_y,base_z=0.0;

	ECEF2WGS(pBasePos, &wgspos);
	
	base_B = wgspos.lat;
	base_L = wgspos.lon;
	base_H = wgspos.alt;
	
	dSinLat = sin(base_B);
	dCosLat = cos(base_B);
	dSinLon = sin(base_L);
	dCosLon = cos(base_L);

	Trans_BLH_to_XYZ(BD2_SYSTEM,base_B,base_L,base_H,&base_x,&base_y,&base_z);

	delx = base_x + baseline->dxyz.x;
	dely = base_y + baseline->dxyz.y;
	delz = base_z + baseline->dxyz.z;

	Trans_XYZ_to_BLH(BD2_SYSTEM,delx,dely,delz,&baseline->B,&baseline->L,&baseline->H);
	
	//璁＄畻鏃嬭浆鐭╅樀
	dRotMatrix[0] = -dSinLon;
	dRotMatrix[1] = dCosLon;
	dRotMatrix[2] = 0;
	dRotMatrix[3] = -dSinLat*dCosLon;
	dRotMatrix[4] = -dSinLat*dSinLon;
	dRotMatrix[5] = dCosLat;
	dRotMatrix[6] = dCosLat*dCosLon;
	dRotMatrix[7] = dCosLat*dSinLon;
	dRotMatrix[8] = dSinLat;
	//delta_x,y,z -->ENU鏂瑰悜鐭㈤噺
	baseline->East 	= 	dRotMatrix[0]*baseline->dxyz.x + dRotMatrix[1]*baseline->dxyz.y;
	baseline->North = 	dRotMatrix[3]*baseline->dxyz.x + dRotMatrix[4]*baseline->dxyz.y + dRotMatrix[5]*baseline->dxyz.z;
	baseline->Up 	= 	dRotMatrix[6]*baseline->dxyz.x + dRotMatrix[7]*baseline->dxyz.y + dRotMatrix[8]*baseline->dxyz.z;
	baseline->BaselineLongth = sqrt(	baseline->East*baseline->East
		+ 	baseline->North*baseline->North
		+ 	baseline->Up*baseline->Up);	
}


// 灏嗙粡绾害鍧愭爣鍙樻崲鎴愰珮鏂姇褰卞潗鏍�
void CGCS2000_CoordToGauss(EARTH_COORD *m_coord,GAUSS_COORD *Gauss)
{

	UINT16 usLen = 0;
	UINT32 iD;
	BOOL Flag;
	DOUBLE dRadLat,L0,dL,XA,XB,XC,XD,XE,X0,N,t,t2,t4,n2=0.0;

	Flag = FALSE;
	dRadLat = m_coord->dLatitude * CGCS2000_PI / 180.0;
	L0 = ((INT16)(m_coord->dLongitude/6))*6 + 3;	// 鍗曚綅搴�
	dL = 0;
	XA = 1 + 3 * CGCS2000_e2 / 4 + 45 * CGCS2000_e4 / 64 + 175 * CGCS2000_e6 / 256 + 11025 * CGCS2000_e8 / 16384;
	XB = 3 * CGCS2000_e2 / 4 + 15 * CGCS2000_e4 / 16 + 525 * CGCS2000_e6 / 512 + 2205 * CGCS2000_e8 / 2048;
	XC = 15 * CGCS2000_e4 / 64 + 105 * CGCS2000_e6 / 256 + 2205 * CGCS2000_e8 / 4096;
	XD = 35 * CGCS2000_e6 / 512 + 315 * CGCS2000_e8 / 2048;
	XE = 315 * CGCS2000_e8 / 16384;
	X0 = CGCS2000_a * (1 - CGCS2000_e2) * (XA * m_coord->dLatitude / CGCS2000_p0 - 
					XB * sin(2 * dRadLat) / 2 + XC * sin(4 * dRadLat) / 4 - 
					XD * sin(6 * dRadLat) / 6 + XE * sin(8 * dRadLat) / 8);

	N = CGCS2000_a / sqrt(1 - CGCS2000_e2 * pow(sin(dRadLat), 2));
	t = tan(dRadLat);
	t2 = pow(t, 2);
	t4 = pow(t2, 2);
	n2 = CGCS2000_e12 * pow(cos(dRadLat), 2);
	
    //080326, 澶勭悊6鐨勫�鏁版暟鎹笉鍑嗙‘.
    if((INT16)m_coord->dLongitude == m_coord->dLongitude)
    {
        if(((INT16)m_coord->dLongitude % 6) == 0)// &&(m_coord->dLongitude!=0))
        {
        	L0=L0-6;
        	Flag = TRUE;
        }
    }
    
    dL = (m_coord->dLongitude -  L0) * 3600;

	Gauss->X = X0 + N * sin(dRadLat) * cos(dRadLat) * pow(dL, 2) / (2 * pow(CGCS2000_p2, 2))
		 + N * pow(dL, 4) * sin(dRadLat) * pow(cos(dRadLat), 3) * (5 - t2 + 9 * n2 + 4 * pow(n2, 2)) / (24 * pow(CGCS2000_p2, 4))
		 + N * sin(dRadLat) * pow(cos(dRadLat), 5) * (61 - 58 * t2 + pow(t2, 2)) * pow(dL, 6) / (720 * pow(CGCS2000_p2, 6));

	Gauss->Y = N * (cos(dRadLat) * dL / CGCS2000_p2
		+ pow(cos(dRadLat), 3) * (1 - pow(t, 2) + n2) * pow(dL, 3) / (6 * pow(CGCS2000_p2, 3))
		+ pow(cos(dRadLat), 5) * (5 - 18 * pow(t, 2) + pow(t, 4) + 14 * n2 - 58 * n2 * pow(t, 2)) * pow(dL, 5) / (120 * pow(CGCS2000_p2, 5)));

	// 鍔犲甫鍙风籂姝�
	Gauss->Y += 500000;
	
	iD = (UINT32)Gauss->Y;
	
	while (iD >= 1)
	{
		usLen ++;
		iD = iD / 10;
	}

	if (Flag)
	{
		Gauss->Y += ((INT16)(m_coord->dLongitude / 6)) * pow(10.0, usLen);
	}
	else
	{
		Gauss->Y += ((INT16)((m_coord->dLongitude / 6) + 1)) * pow(10.0, usLen);
	}
	Gauss->H = m_coord->dAltitude - m_coord->dAltDelta;
}


//------------------------------------------------------------------------------
//鍔熻兘锛氳绠楀弽姝ｅ垏鍑芥暟锛屾牴鎹畑,y鐨勭鍙疯嚜鍔ㄨ绠楁墍鍦ㄨ薄闄�
//鍙傛暟锛�
//[IN]	瀵硅竟 y
//[IN]	閭昏竟 x
//杩斿洖锛氳搴﹀�锛屽崟浣嶏紙寮у害锛�
//-------------------------------------------------------------------------------
DOUBLE atan_2(DOUBLE y, DOUBLE x)
{
	DOUBLE ret = 0;
	if( ( x == 0.0) && (y >= 0.0) )    ret = 0.5 * PI;
	else if( (x == 0.0) && (y < 0.0) ) ret = 1.5 * PI;
	else
	{
		ret = atan( y / x );
		if( x < 0.0 )  ret = ret + PI;
		else if( (x > 0.0) && (y < 0.0) ) ret = ret + 2.0 * PI;
	}
	return ret;
}

#ifndef _POSTPROC
#ifndef _SIMULATE
//-------------------------------------------------------------------------------
//鍔熻兘锛氶�杩囧懆璁℃暟鍜屽懆鍐呯璁℃暟寰楀埌骞存湀鏃ャ�鏃跺垎绉掞紝鍚屾椂涔熷緱鍒板勾绉棩鍜屽ぉ绉掞紙BD2锛�
//鍙傛暟锛�
//[IN] 鏃堕棿缁撴瀯浣撴寚閽�ObsTime*
//杩斿洖锛氭棤
//-------------------------------------------------------------------------------
void BD2GetYMDHMS(BD2_OBSTIME* time,BOOL utc)
{
	UINT32 total_sec;
	INT32 four_year, one_year;
	INT32 four_year_sec, year_sec, year_day;
	INT32 day_sec, delta;
	INT16 month;

	BOOL leap = FALSE;

	//assert(time);

     if(utc)
     	{
	     if(time->nWeeks>100)
		    time->dSeconds=time->dSeconds;
		 if(time->dSeconds<0)
		 	{
		 	  time->dSeconds+=SECONDS_ONE_WEEK;
			  time->nWeeks--;
		 	}
     	}
	total_sec = time->nWeeks*SECONDS_ONE_WEEK + (UINT32)time->dSeconds;	// 鍏ㄩ儴绉掓暟
	total_sec += SECONDS_COMMON_YEAR;	

	// 鎻愬墠涓�勾锛岃繖鏍蜂粠2005骞村紑濮嬶紝鍥涘勾涓�釜鍛ㄦ湡锛屾瘡鍛ㄦ湡涓渶鍚庝竴骞存槸闂板勾锛�
	// 寰楀埌鍦ㄧ鍑犱釜鍥涘勾鍛ㄦ湡銆佸洓骞翠腑鐨勫摢涓�勾浠ュ強骞村唴鐨勭鏁�
	four_year = total_sec/SECONDS_FOUR_YEAR;
	four_year_sec = total_sec%SECONDS_FOUR_YEAR;
	one_year = four_year_sec/SECONDS_COMMON_YEAR;
	year_sec = four_year_sec%SECONDS_COMMON_YEAR;

	if(one_year>3)
		one_year = 3;

	// 浜庢槸寰楀埌浜嗗勾
	time->year = 2005 + four_year*4 + one_year;

	// 鍒ゆ柇鏄惁鏄棸骞�
	if(time->year%400==0 || (time->year%4==0 && time->year%100!=0))
	{
		leap = TRUE;
		//year_sec += SECONDS_COMMON_YEAR;	// 闂板勾鐨勬渶鍚庝竴澶�
	}

	// 寰楀埌骞村唴澶╂暟
	year_day = year_sec/SECONDS_ONE_DAY + 1;
	// 鏍规嵁澶╂暟寰楀埌鏈堜唤鍜屾暟
	for(month=1; month<13; month++)
	{
		if(month == 2 && leap)
		{
			year_day--;	// 骞�鏈堣鐗规畩澶勭悊
			delta = 1;
		}
		else
			delta = 0;

		if(year_day > (MONTH_TABLE[month-1]-delta) && year_day <= MONTH_TABLE[month])
			break;
	}
	time->month = month;
	if(month == 2 && leap)
		year_day++;
	time->day = year_day - MONTH_TABLE[month-1];

	// 鍚屾椂杩樺緱鍒颁簡骞寸Н鏃�
	time->DayOfYear = year_day;

	// 寰楀埌澶╁唴绉掓暟
	day_sec = ((INT32)time->dSeconds)%SECONDS_ONE_DAY;
	time->SecOfDay = day_sec + (time->dSeconds - (INT32)time->dSeconds);

	// 鍗冲彲寰楀埌鏃躲�鍒嗐�绉�
	time->hour = day_sec/3600;
	time->min = (day_sec%3600)/60;
	time->sec = day_sec - (time->hour*3600 + time->min*60)
		+ (time->dSeconds - (INT32)time->dSeconds);
	time->weekday=time->dSeconds/(3600*24)+1;
}

//-------------------------------------------------------------------------------
//鍔熻兘锛氶�杩囧懆璁℃暟鍜屽懆鍐呯璁℃暟寰楀埌骞存湀鏃ャ�鏃跺垎绉掞紝鍚屾椂涔熷緱鍒板勾绉棩鍜屽ぉ绉掞紙GPS锛�
//鍙傛暟锛�
//[IN] 鏃堕棿缁撴瀯浣撴寚閽�ObsTime*
//杩斿洖锛氭棤
//-------------------------------------------------------------------------------
void GPSGetYMDHMS(BD2_OBSTIME* time,BOOL gps)
{
	UINT32 total_sec;
	INT32 four_year, one_year;
	INT32 four_year_sec, year_sec, year_day;
	INT32 day_sec, delta;
	INT16 month;


	BOOL leap = FALSE;

	//assert(time);
	
    if(gps)
       {
     	 time->dSeconds=time->dSeconds;
		 if(time->dSeconds<0)
		 	{
		 	  time->dSeconds+=SECONDS_ONE_WEEK;
			  time->nWeeks--;
		 	}
     	}
		
	total_sec = (time->nWeeks+1024)*SECONDS_ONE_WEEK + (UINT32)time->dSeconds+86400*5;	// 鍏ㄩ儴绉掓暟
	total_sec += SECONDS_COMMON_YEAR*3;	
	

	// 鎻愬墠涓�勾锛岃繖鏍蜂粠2005骞村紑濮嬶紝鍥涘勾涓�釜鍛ㄦ湡锛屾瘡鍛ㄦ湡涓渶鍚庝竴骞存槸闂板勾锛�
	// 寰楀埌鍦ㄧ鍑犱釜鍥涘勾鍛ㄦ湡銆佸洓骞翠腑鐨勫摢涓�勾浠ュ強骞村唴鐨勭鏁�
	four_year = total_sec/SECONDS_FOUR_YEAR;
	four_year_sec = total_sec%SECONDS_FOUR_YEAR;
	one_year = four_year_sec/SECONDS_COMMON_YEAR;
	year_sec = four_year_sec%SECONDS_COMMON_YEAR;

	if(one_year>3)
		one_year = 3;

	// 浜庢槸寰楀埌浜嗗勾
	time->year = 1977 + four_year*4 + one_year;

	// 鍒ゆ柇鏄惁鏄棸骞�
	if(time->year%400==0 || (time->year%4==0 && time->year%100!=0))
	{
		leap = TRUE;
		//year_sec += SECONDS_COMMON_YEAR;	// 闂板勾鐨勬渶鍚庝竴澶�
	}

	// 寰楀埌骞村唴澶╂暟
	year_day = year_sec/SECONDS_ONE_DAY + 1;
	// 鏍规嵁澶╂暟寰楀埌鏈堜唤鍜屽ぉ鏁�
	for(month=1; month<13; month++)
	{
		if(month == 2 && leap)
		{
			year_day--;	// 闂板勾鐨�鏈堣鐗规畩澶勭悊
			delta = 1;
		}
		else
			delta = 0;

		if(year_day > (MONTH_TABLE[month-1]-delta) && year_day <= MONTH_TABLE[month])
			break;
	}
	time->month = month;
	if(month == 2 && leap)
		year_day++;
	time->day = year_day - MONTH_TABLE[month-1];

	// 鍚屾椂杩樺緱鍒颁簡骞寸Н鏃�
	time->DayOfYear = year_day;

	// 寰楀埌澶╁唴绉掓暟
	day_sec = ((INT32)time->dSeconds)%SECONDS_ONE_DAY;
	time->SecOfDay = day_sec + (time->dSeconds - (INT32)time->dSeconds); 

	// 鍗冲彲寰楀埌鏃躲�鍒嗐�绉�
	time->hour = day_sec/3600;	
	time->min = (day_sec%3600)/60;
	time->sec = day_sec - (time->hour*3600 + time->min*60)
		+ (time->dSeconds - (INT32)time->dSeconds);
}
#endif
#endif




