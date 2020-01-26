
#include "define.h"
#include "CalcSvPVT.h"
#include "math.h"
#include "Cfgpara.h"
#include "PVTRcvrInfo.h"
#ifndef _POSTPROC
#ifndef _SIMULATE
#include "GLOSfParse.h"
#include "GnssTYProcfg.h"
#include "sysbackup.h"
#endif
#endif

double AdjustValue(double invalue,  double Left, double Right);
#if SUPPORT_GLONASS
void GloEph2pos(int32 sv, double second,double *x, double *y, double *z,double *vx, double *vy, double *vz);
void GloEph2clk(double time, int32 sv, double *p_svclkbias , double *p_svfreqbias );
void glorbit(double t, double *x, const double *acc);
void GloAlamcePrediction(int32 svFre,int32 DayNumber, double DayTime, ECEF *pPos, ECEF *pVel);
void deq(const double *x, double *xdot, const double *acc);
double dot(const double *a, const double *b, int n);
#endif

short EphemerisPredictionGPS(unsigned short sv, double sec, double *relativistic,double *x, double*y, double *z,double *vx, double *vy, double *vz );


double Kepler_equation(double e,double M_);
double SvClockCorrection(double sat_time,GPSBD2EphFrameStruct *Ephem,double *del_toe);

double PuttimeInWeek(double sec) ;
double AdjustValue(double invalue,  double Left, double Right);
void GpsBD2AlmanacePrediction( unsigned short SvID,unsigned short wk, double sec, ECEF *pPos, ECEF *pVel );
void MeteorologicExt(double *IN_StationHgt,  double  *StationTemperature, double *StationPressure,  double *StationES );
double Meo_IonoSphericCorrModel(double fSec, const IonoStruct *ionoutc, double fLat, double fLon, double fAlt, double fElvRad, double fAziRad);
double IONOSPHERE_GetL1KlobucharCorrection(double fSec, const IonoStruct *ionoutc, double fLat, double fLon, double fAlt, double fElvRad, double fAziRad);
double HopfiedTroposphereCorr(double fElevation, double fStationHgt);

void EphemerisPredictionGeo(int32 SvID, double sec, double *relativistic, double *x, double *y, double *z, double *vx, double *vy, double *vz );


void CalcSV_PVT_Alm(int32 truesvid, int32 wn, float64 time, ECEF *pPos, ECEF *pVel)
{
#ifndef _POSTPROC
#ifndef _SIMULATE

#if SUPPORT_GLONASS
	int32 DayNumber = 0;
	double Daytime = 0.0;
#endif

	if (SV_IsBd2(truesvid))
	{
		time -= GPS_BD_SYSTIME_OFFSET;
		if(time < MICRO_NUM)
			time += SECONDS_IN_WEEK;
	}

	SWI_disable();
#if SUPPORT_GLONASS
	if ((truesvid>= MinGloFreID) && (truesvid<=MaxGloSvIDTRUE))
	{
		GpsToGloTime(wn,time,&DayNumber,&Daytime);
		GloAlamcePrediction(truesvid - MinGloFreID+1,DayNumber,Daytime,pPos,pVel);
	}
	else
#endif
		GpsBD2AlmanacePrediction( truesvid, wn, time,pPos,pVel);

	SWI_enable();
#endif
#endif
}

#if SUPPORT_GLONASS
bool GloFreidToSvid(int32 FreId, int32 *Svid, int32 *n)
{
	int32 iloop=0,FreN=10,idx =0;
	bool bAllSVAlmParsed = TRUE;
	
	FreId = FreId - MinGloFreID -7;
	for (iloop=0; iloop<SV_NUM_GLO_TRUE; iloop++)
	{
		if(Sys3AlmEphUTCInfo.Glo_AlmStruct[iloop].vflg == FALSE)
		{
			bAllSVAlmParsed = FALSE;
			continue;
		}
	
		FreN = Sys3AlmEphUTCInfo.Glo_AlmStruct[iloop].HnFreq;
		
		if (FreId != FreN)
			continue;
		
		Svid[idx] = iloop+1;
		idx++;
		
		if (idx>=2)
			break;
	}

	if(bAllSVAlmParsed || (idx==2))
	{
		*n = idx;
		return TRUE;
	}
	else
		return FALSE;
}


void GloAlamcePrediction(int32 truesvid,int32 DayNumber, double DayTime, ECEF *pPos, ECEF *pVel)
{ 
	double tk,T,n,Mk,Ek,cycles,a,e,i,omega,OMEGA,Px,Py,Pz,Qx,Qy,Qz,A,B,C,D;    
    double cos_omega,sin_omega,cos_OMEGA,sin_OMEGA,cos_i,sin_i;
    double sinEk,cosEk,K,rora,EII,Delta_T,K1,K2; 
	GLOAlmanaceStruct *alm;

    alm = &Sys3AlmEphUTCInfo.Glo_AlmStruct[truesvid-1];

	if (alm->vflg !=1)
		return;    

	tk = (DayNumber - alm->Day)*SECONDS_IN_DAY + DayTime - alm->TNode;
    tk = tk + alm->DelTclk;
    T = SECONDS_IN_DAY/2.0 ;
    cycles = tk/T;
    T = T + alm->DelTn+alm->Del2Tn*cycles;
    n = 2.0*PI/T;
    a = pow((double)(MU_GLO/n/n),(double)(1.0/3.0));
    e = alm->ecc;
    i = alm->DelI0;
    cos_i = cos(i);
    sin_i = sin(i);
	K=pow(RE_GLO/a,3.5)*PI/(15552000.0);
	K1=(pow(a/RE_GLO,1.5)*PI/(15552000.0))*cos_i;
	K2=(pow(a/RE_GLO,1.5)*PI/(15552000.0))*(4.0-5.0*sin_i*sin_i);
    rora = 5.0*K*(5.0*cos_i*cos_i-1.0)-0.0002887*K1;
    omega = alm->wn+rora*tk;
    cos_omega = cos(omega);
    sin_omega = sin(omega);
	rora = -10.0*K*cos_i+0.0001414*K2;
    OMEGA = alm->LonNode+ (rora - OMGE_GLO) * tk;
    cos_OMEGA = cos(OMEGA);
    sin_OMEGA = sin(OMEGA);
	EII=2.0*atan(sqrt((1.0-e)/(1.0+e))*tan(omega/2.0));
	Delta_T=(EII-e*sin(EII))/n;
	if(omega>PI)
		Delta_T+=T;
	Mk=n*(tk-Delta_T);
	Ek=Kepler_equation( e,Mk);		
    sinEk = sin(Ek);
    cosEk = cos(Ek);
    A = a*(cosEk - e);
    B = a*sqrt(1.0 -e*e)*sinEk;
	Px = cos_omega * cos_OMEGA - sin_omega * cos_i * sin_OMEGA;
	Qx = -sin_omega * cos_OMEGA - cos_omega * cos_i * sin_OMEGA;
	Py = cos_omega * sin_OMEGA + sin_omega * cos_i * cos_OMEGA;
	Qy = -sin_omega * sin_OMEGA + cos_omega * cos_i * cos_OMEGA;
	Pz = sin_omega * sin_i;
	Qz = cos_omega * sin_i;
	pPos->x = A*Px + B*Qx;
    pPos->y = A*Py + B*Qy;
    pPos->z = A*Pz + B*Qz;	
	C=-(n*a*sinEk)/(1.0-e*cosEk);  
    D=(n*a*cosEk*sqrt(1.0-e*e))/(1.0-e*cosEk);    
	pVel->x=C*Px+D*Qx+OMGE_GLO*( pPos->y);
	pVel->y=C*Py + D*Qy-OMGE_GLO*( pPos->x);
	pVel->z=  C*Pz + D*Qz;

    return;
}

#endif

void GpsBD2AlmanacePrediction( unsigned short trueSvID,unsigned short wk, double sec, ECEF *pPos, ECEF *pVel )
{
   
	double a, n0, M, E, P, R, I, L;
	double sE, cE, sI, cI, sL, cL, sP, cP;
	double dEdM, ecc, sqrt1mee,  Xp, Yp;
	double Edot, Pdot, Rdot,  Xpdot, Ypdot, Ldot, sPdot, cPdot;
	double tk,tempvalue;
	int32 jloop = 0;
	GPSBD2AlmanaceStruct *alm;	
	
	alm = &(Sys3AlmEphUTCInfo.GpsBd2_AlmStruct[trueSvID-1]);

	if (alm->vflg !=1)
		return;
	
	tk = ( wk -alm->refweek) * SECONDS_IN_WEEK + sec - alm->toa;
	tk= PuttimeInWeek(tk);	
	a = alm->sqrta * alm->sqrta;
	tempvalue=GPSGravConstant;
	if( trueSvID>MaxGpsSvID )
		tempvalue=BD2GravConstant;
	n0 = sqrt(tempvalue/(a*a*a));
	M = alm->m0 + n0 * tk;
	M = fmod(M, TWO_PI);
	ecc = alm->ecc;
	sqrt1mee = sqrt (1.0 - ecc * ecc);
    E=Kepler_equation(ecc, M);
	sE = sin(E);
	cE = cos(E);
	dEdM = 1.0 / (1.0 - ecc * cE);
	Edot = dEdM * n0;
	P = atan2 (sqrt1mee * sE, cE - ecc) + alm->w;
	Pdot = sqrt1mee * dEdM * Edot;
  	sP = sin(P);
	cP = cos(P);
	sPdot = cP*Pdot;
	cPdot = -sP*Pdot;
	R = a * (1.0 - ecc * cE);
	Rdot = a * ecc * sE * Edot;
	I = alm->i0; 
   	sI = sin (I); cI = cos (I);
	Xp = R * cP;
	Yp = R * sP;
	tempvalue=WGS84oe;
	if( trueSvID>MaxGpsSvID )
		tempvalue=BD2WGS84oe;
   	L = alm->omega0 + tk * (alm->omegadot - tempvalue) - tempvalue * alm->toa;
	Ldot = alm->omegadot - tempvalue;
	sL = sin (L); 
	cL = cos (L);
	pPos[jloop].x = Xp * cL - Yp * cI * sL;
	pPos[jloop].y = Xp * sL + Yp * cI * cL;
	pPos[jloop].z = Yp * sI;
	Xpdot = Rdot * cP + R * cPdot;
	Ypdot = Rdot * sP + R * sPdot;
	pVel[jloop].x = -Ldot * (pPos[jloop].y)+ Xpdot * cL- Ypdot * cI * sL;
	pVel[jloop].y = Ldot * (pPos[jloop].x)+ Xpdot * sL+ Ypdot * cI * cL;
	pVel[jloop].z = Ypdot * sI;
        
}
//


void CalcSV_PVT_Eph(int32 svid, double ts,  ECEF* p_svpos, ECEF* p_svvel, double* p_svclkbias, double* p_svfreqbias,word32* ptoe)
{
	double svirelTmp=0.0;	//not use in this function
#if SUPPORT_GLONASS
	double tk = 0;
#endif
	if(SV_IsBd2(svid))
	{
		ts = ts - GPS_BD_SYSTIME_OFFSET;
		if(ts < MICRO_NUM)
			ts += SECONDS_IN_WEEK;
	}

#if SUPPORT_GLONASS
	if (SV_IsGlo(svid))
	{	
		// 输入ts，输出tk
		GpsToGloTimeTod(ts, &tk);

		// 读取全局结构体Sys3AlmEphUTCInfo，输入svid、tk，输出p_svpos、p_svvel
		GloEph2pos(svid, tk, &(p_svpos->x), &(p_svpos->y), &(p_svpos->z), &(p_svvel->x), &(p_svvel->y), &(p_svvel->z));

		// 读取全局结构体Sys3AlmEphUTCInfo，输入svid、tk，输出p_svclkbias、p_svfreqbias
		GloEph2clk(tk, svid, p_svclkbias, p_svfreqbias);

		// 读取全局结构体Sys3AlmEphUTCInfo，输出ptoe
		if(ptoe!=NULL)
			*ptoe = Sys3AlmEphUTCInfo.Glo_EphStruct[svid-MinGloFreID].tb;
	}
	else
#endif
	{
		if(SV_IsGps(svid) || SV_IsBd2Meo(svid))
		{		
			// 输入svid、ts，读取全局结构体Sys3AlmEphUTCInfo，输出svirelTmp、p_svpos、p_svvel
			EphemerisPredictionGPS(svid, ts, &svirelTmp, &(p_svpos->x), &(p_svpos->y), &(p_svpos->z), &(p_svvel->x), &(p_svvel->y), &(p_svvel->z));
		}
		else if(SV_IsBd2Geo(svid))
		{
			// 输入svid、ts，读取全局结构体Sys3AlmEphUTCInfo，输出svirelTmp、p_svpos、p_svvel
			EphemerisPredictionGeo(svid, ts, &svirelTmp, &(p_svpos->x), &(p_svpos->y), &(p_svpos->z), &(p_svvel->x), &(p_svvel->y), &(p_svvel->z));
		}

		if((p_svclkbias != NULL) && (p_svfreqbias != NULL))
			// 输入ts、svirelTmp、svid，读取全局结构体Sys3AlmEphUTCInfo，输出p_svclkbias、p_svfreqbias
			CalcSV_ClkErr_Eph(ts, svirelTmp,svid , p_svclkbias, p_svfreqbias);

		if(ptoe!=NULL)
			// 读取全局结构体Sys3AlmEphUTCInfo，输出ptoe
			*ptoe = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[svid-MinGpsSvID].toe;
	}

	return;
}

#if SUPPORT_GLONASS
void GloEph2pos(int32 sv, double second,double *x, double *y, double *z,double *vx, double *vy, double *vz)
{
	CordTrans7Para Para = {0.0,0,0.0,0.0,0.0,-1.599885147661469e-006,0.0};
	//CordTrans7Para Para = {0.0,2.5,0.0,0.0,0.0,-1.939254724438144e-006,0.0};
	GLOEphFrameStruct *pEph = &(Sys3AlmEphUTCInfo.Glo_EphStruct[sv-MinGloFreID]);
	double t,tt,pos[6],acc[3];
	double svclkbias = 0.0, svfreqbias = 0.0;

	if (pEph->vflg !=1)
		return;
	
	memset(pos,0,sizeof(pos));
	memset(acc,0,sizeof(acc));

	GloEph2clk(second, sv, &svclkbias, &svfreqbias); //intime is GPS time
	
	t=second-svclkbias-(pEph->tb)*SECONDS_IN_MINUTE;
    t=AdjustValue(t,-43200.0,43200.0); 
    pos[0]=(pEph->x)*1000.0;
    pos[1]=(pEph->y)*1000.0;
    pos[2]=(pEph->z)*1000.0; 
	pos[3]=(pEph->vx)*1000.0;
    pos[4]=(pEph->vy)*1000.0;
    pos[5]=(pEph->vz)*1000.0;  
	acc[0]=(pEph->vvx)*1000.0;
	acc[1]=(pEph->vvy)*1000.0;
	acc[2]=(pEph->vvz)*1000.0;
    for (tt=t<0.0?-TSTEP:TSTEP;fabs(t)>1E-9;t-=tt)
	{
        if (fabs(t)<TSTEP) tt=t;
        glorbit(tt,pos,acc);
    }
	
	// 输入Para 修改POS[0][1][2],
	CordTrans_Using7Para(&pos[0], &pos[1],&pos[2], &Para);

	*x  = pos[0] ;
	*y  = pos[1] ;
	*z  = pos[2] ;
	*vx = pos[3] ;
	*vy = pos[4] ;
	*vz = pos[5] ;
		
}

 void glorbit(double t, double *x, const double *acc)
{
    double k1[6],k2[6],k3[6],k4[6],w[6];
    int i;    
    deq(x,k1,acc); 
    for (i=0;i<6;i++)
		w[i]=x[i]+0.5*k1[i]*t;
    deq(w,k2,acc);
	for (i=0;i<6;i++)
		w[i]=x[i]+0.5*k2[i]*t;
    deq(w,k3,acc);
	for (i=0;i<6;i++)
		w[i]=x[i]+k3[i]*t;
    deq(w,k4,acc);
    for (i=0;i<6;i++)
		x[i]+=(k1[i]+2.0*k2[i]+2.0*k3[i]+k4[i])*t/6.0;
}
 
 void deq(const double *x, double *xdot, const double *acc)
{

	double r,r2,r5,r3;

	r = sqrt(x[0] * x[0] + x[1]*x[1] + x[2]*x[2]);
	r2 = r*r;
	r3 = r*r*r;
	r5 = r2*r3;
	

	xdot[0] = x[3];
	xdot[1] = x[4];
	xdot[2] = x[5];
	xdot[3] = -MU_GLO/r3 * x[0] - 1.5 * J2_GLO * MU_GLO*RE_GLO*RE_GLO/r5 * x[0]*(1 - 5 * x[2]*x[2]/r2) 
		+  OMGE_GLO*OMGE_GLO*x[0] + 2*OMGE_GLO*x[4] + acc[0];
	xdot[4] =  -MU_GLO/r3 * x[1] - 1.5 * J2_GLO * MU_GLO*RE_GLO*RE_GLO/r5 * x[1]*(1 - 5 * x[2]*x[2]/r2) 
		+  OMGE_GLO*OMGE_GLO*x[1] - 2*OMGE_GLO*x[3] + acc[1];
	xdot[5] = -MU_GLO/r3 * x[2] - 1.5 * J2_GLO * MU_GLO*RE_GLO*RE_GLO/r5 * x[2]*(3 - 5 * x[2]*x[2]/r2) + acc[2];
}

 double dot(const double *a, const double *b, int n)
{
	double c=0.0;	  
	while (--n>=0)
	 c+=a[n]*b[n];
	return c;
}

 void GloEph2clk(double time, int32 sv, double *p_svclkbias , double *p_svfreqbias )
{
	double del_toc ; 
	GLOEphFrameStruct *pGloEph = &(Sys3AlmEphUTCInfo.Glo_EphStruct[sv-MinGloFreID]);

	if (pGloEph->vflg !=1)
		return;

	del_toc = time - pGloEph->tb*SECONDS_IN_MINUTE;
	del_toc  =AdjustValue(del_toc ,-43200.0,43200.0) ; 
	*p_svclkbias = -pGloEph->Tn+pGloEph->gaman*del_toc ; //已经反向了注意。   
	*p_svfreqbias = pGloEph->gaman; //已经反向了注意。   	
}
#endif
void EphemerisPredictionGeo(int32 SvID, double sec, double *relativistic, double *x, double *y, double *z, double *vx, double *vy, double *vz )
{ 
	double tk, M, E, P, U, R, I, cU, sU, Xp, Yp,L, sI, cI, sL, cL, ecc, s2P, c2P,Edot, Pdot, Udot, Rdot, sUdot, cUdot, Xpdot, Ypdot, Idot, Ldot;    
	double Mdot, sqrt1mee,sE,cE,a,n0,omdtk,BDx,BDy,BDz,BDvx,BDvy,BDvz; 
	GPSBD2EphFrameStruct *pEph = &Sys3AlmEphUTCInfo.GpsBd2_EphStruct[SvID-MinGpsSvID];

	if (pEph->vflg !=1)
		return;

	sE = 0.0;
	cE = 0.0;    
	a = pEph->sqrta * pEph->sqrta;     
	n0 = sqrt( BD2GravConstant / ( a * a * a ) );
	SvClockCorrection(sec,pEph,&tk);
	Mdot = n0 + pEph->deltan;
	M = pEph->m0 + Mdot * tk;
	ecc = pEph->ecc;
	sqrt1mee = sqrt( 1.0 - ecc * ecc );    
	E=Kepler_equation(ecc, M);
	sE = sin(E);
	cE = cos(E);    
	*relativistic = -4.4428073090439775E-10 * ecc * pEph->sqrta * sE;  
	Edot = (1.0 / (1.0 - ecc * cE)) * Mdot;
	//phik
	P = atan2( sqrt1mee * sE, cE - ecc ) ;        
	P += pEph->w;
	//phikdot
	Pdot = sqrt1mee * (1.0 / (1.0 - ecc * cE)) * Edot;
	s2P = sin(2.0 * P);
	c2P = cos(2.0 * P);
	//siu
	U = P + (pEph->cus*s2P + pEph->cuc * c2P);
	sU = sin(U);
	cU = cos(U);
	//ukdot
	Udot = Pdot * (1.0 + 2.0 * (pEph->cus * c2P - pEph->cuc * s2P));
	sUdot = cU * Udot;
	cUdot = -sU * Udot;
	R = a * (1.0 - ecc * cE) + (pEph->crs * s2P + pEph->crc * c2P);
	//rkdot
	Rdot = a * ecc * sE * Edot + 2.0 * Pdot * (pEph->crs * c2P - pEph->crc * s2P);
	I = pEph->i0 + pEph->idot * tk + (pEph->cis * s2P + pEph->cic * c2P);

	sI = sin(I);
	cI = cos(I);
	//ikdot
	Idot = pEph->idot + 2.0 * Pdot * (pEph->cis * c2P - pEph->cic * s2P);
	Xp = R * cU;
	Yp = R * sU;
	Xpdot = Rdot * cU + R * cUdot;
	Ypdot = Rdot * sU + R * sUdot;    
	L = pEph->omega0 + tk * pEph->omegadot - BD2WGS84oe * pEph->toe;
	Ldot = pEph->omegadot;    
	sL = sin(L);
	cL = cos(L);   
	*x = Xp * cL - Yp * cI * sL;
	*y = Xp * sL + Yp * cI * cL;
	*z = Yp * sI;       
	*vx = Xpdot * cL - Ypdot * sL * cI + (*z) * sL * Idot - Ldot * (*y);      
	*vy = Xpdot * sL + Ypdot * cI * cL - (*z) * cL * Idot + Ldot * (*x);
	*vz = Ypdot * sI + Yp * cI * Idot;            
	omdtk = BD2WGS84oe * tk;    
	sE = sin(omdtk);
	cE = cos(omdtk);      
	BDx = *x;
	BDy = *y;
	BDz = *z;    
	*x = BDx * cE + sE * (BDy * cos5 + BDz * sin5);
	*y = -BDx * sE + cE * (BDy * cos5 + BDz * sin5);          
	*z = -BDy * sin5 + BDz * cos5;
	BDvx = *vx;
	BDvy = *vy;
	BDvz = *vz;
	*vx = (*y) * BD2WGS84oe + sE * (BDvy * cos5 + BDvz * sin5) + BDvx * cE;
	*vy = -(*x) * BD2WGS84oe - BDvx * sE + cE * (BDvy * cos5 + BDvz * sin5);         
	*vz = -BDvy * sin5 + BDvz * cos5;                      
}



short EphemerisPredictionGPS(unsigned short sv, double sec, double *relativistic,double *x, double*y, double *z,double *vx, double *vy, double *vz )
{
	const double const_ecc = -4.4428076333930602E-10;
    short iter;
    double tk, M, E, P, U, R, I, cU, sU, Xp, Yp,L, sI, cI, sL, cL, ecc, s2P, c2P,Edot, Pdot, Udot, Rdot, sUdot, cUdot, Xpdot, Ypdot, Idot, Ldot;    
    double Mdot, sqrt1mee,sE,cE,a,n0,ek1,ek2; 

	GPSBD2EphFrameStruct* Ephemeris = &Sys3AlmEphUTCInfo.GpsBd2_EphStruct[sv-MinGpsSvID];
	
    if( Ephemeris->vflg!=1 )
    {
        *relativistic = 0.0;
        *x = *y = *z = 0.0;
        *vx = *vy = *vz = 0.0;        
        return 0;
    }
    
    sE = 0.0;
    cE = 0.0;  
    
    a = Ephemeris->sqrta * Ephemeris->sqrta;  //椭圆轨道半长轴
    //n0 = sqrt( GPSGravConstant / ( a * a * a ) );    
    if(SV_IsBd2(sv))
        n0 = sqrt( BD2GravConstant / ( a * a * a ) );
    else
        n0 = sqrt( GPSGravConstant / ( a * a * a ) );   

  //  tk = sec -Ephemeris->toe;

  //  tk = fmod(tk,SECONDS_IN_WEEK);

    // Mean anomaly, M (rads).

	// 输入sec和全局Ephemeris，输出tk
    SvClockCorrection(sec,Ephemeris,&tk);  
    Mdot = n0 + Ephemeris->deltan;
    M = Ephemeris->m0 + Mdot * tk;

    // Obtain eccentric anomaly E by solving Kepler's equation.

    ecc = Ephemeris->ecc;
    sqrt1mee = sqrt(1.0-ecc*ecc); 
    
    ek1=M;    
    for( iter=0;iter<20;iter++ )
    {
        ek2=M+ecc*sin(ek1);
        if(fabs(ek2-ek1)<=MICRO_NUM)
            break;
        ek1=ek2;
    }
    E=ek2;
    sE = sin(E);
    cE = cos(E);
    
    // Compute the relativistic correction term (seconds).

    *relativistic = const_ecc * ecc * Ephemeris->sqrta * sE;

    Edot = (1.0 / (1.0 - ecc*cE)) * Mdot;

    // Compute the argument of latitude, P.

    P = atan2(sqrt1mee*sE,cE-ecc) + Ephemeris->w;
    Pdot = sqrt1mee * (1.0 / (1.0 - ecc*cE)) * Edot;

    // Generate harmonic correction terms for P and R.

    s2P = sin(2.0*P);
    c2P = cos(2.0*P);

    // Compute the corrected argument of latitude, U.

    U = P + (Ephemeris->cus*s2P+Ephemeris->cuc*c2P);
    sU = sin(U);
    cU = cos(U);
    Udot = Pdot * (1.0 + 2.0*(Ephemeris->cus*c2P - Ephemeris->cuc*s2P));
    sUdot = cU*Udot;
    cUdot = -sU*Udot;

    // Compute the corrected radius, R.

    R = a*(1.0-ecc*cE) + (Ephemeris->crs*s2P+Ephemeris->crc*c2P);
    Rdot =a*ecc*sE*Edot + 2.0*Pdot*(Ephemeris->crs*c2P - Ephemeris->crc*s2P);

    // Compute the corrected orbital inclination, I.

    I =Ephemeris->i0 + Ephemeris->idot*tk + (Ephemeris->cis*s2P+Ephemeris->cic*c2P);
    sI = sin(I);
    cI = cos(I);
    Idot = Ephemeris->idot + 2.0*Pdot*(Ephemeris->cis*c2P -Ephemeris->cic*s2P);

    // Compute the satellite's position in its orbital plane, (Xp,Yp).

    Xp = R*cU;
    Yp = R*sU;
    Xpdot = Rdot*cU + R*cUdot;
    Ypdot = Rdot*sU + R*sUdot;

    // Compute the longitude of the ascending node, L.

   // L = Ephemeris->omega0 + tk*(Ephemeris->omegadot-WGS84oe) - WGS84oe*Ephemeris->toe;
    if(SV_IsBd2(sv))
    {
        L = Ephemeris->omega0 + tk * (Ephemeris->omegadot - BD2WGS84oe) - BD2WGS84oe * Ephemeris->toe;
        Ldot = Ephemeris->omegadot - BD2WGS84oe; 
	}
    else
	{
        L = Ephemeris->omega0 + tk * (Ephemeris->omegadot-WGS84oe) - WGS84oe * Ephemeris->toe;
         Ldot = Ephemeris->omegadot - WGS84oe;  
	}
  //  Ldot = Ephemeris->omegadot - WGS84oe;
    sL = sin(L);
    cL = cos(L);

    // Compute the satellite's position in space, (x,y,z).

    *x = Xp*cL - Yp*cI*sL;
    *y = Xp*sL + Yp*cI*cL;
    *z = Yp*sI;

    // Satellite's velocity, (vx,vy,vz).

    *vx = - Ldot*(*y)+Xpdot*cL-Ypdot*cI*sL+Yp*sI*Idot*sL;

    *vy = Ldot*(*x)+Xpdot*sL+Ypdot*cI*cL-Yp*sI*Idot*cL;

    *vz = + Yp*cI*Idot + Ypdot*sI;

    return 1;
}

double SvClockCorrection(double sat_time,GPSBD2EphFrameStruct *Ephem,double *del_toe)
{
	double del_toc,del_tsv,tc,rettime;
	del_toc = sat_time - Ephem->toc;	
	del_toc= PuttimeInWeek(del_toc);
	del_tsv =Ephem->af0+ del_toc * (Ephem->af1 + Ephem->af2 * del_toc);
	del_tsv -= Ephem->tgd;
	tc = sat_time - del_tsv;
    rettime = tc - Ephem->toe;
    rettime= PuttimeInWeek(rettime);
	*del_toe=rettime;
	return del_tsv;
}


//algorithm from TD
 void CalcSV_ClkErr_Eph(float64 time, float64 svirel,int32 svid, float64 *pClkErr, float64 *pFrqErr )
{
	double tcorr, delta_time;
	GPSBD2EphFrameStruct *pEph = &(Sys3AlmEphUTCInfo.GpsBd2_EphStruct[svid-MinGpsSvID]);

	if (pEph->vflg !=1)
		return;

	delta_time = time-pEph->toc;	
	delta_time = PuttimeInWeek(delta_time);

		
	(*pFrqErr) = pEph->af1 + 2.0*delta_time*pEph->af2;  
		
	tcorr = pEph->af0 + delta_time*(pEph->af1+delta_time*pEph->af2) - pEph->tgd;     		   

	tcorr += svirel;
	
	if ( (tcorr>1000000) || (tcorr<-1000000) )
	{
		tcorr = 0.0;
	}
	
	(*pClkErr) = tcorr;
	(*pFrqErr) = (*pFrqErr) * SPEED_OF_LIGHT;

	return;
}

/*************************************************
  Function:       double PuttimeInWeek(double sec)
  Description:
  Calls:          无
  Called By:
  Input:          无
  Output:         无
  Return:         无
  Others:
*************************************************/
 double PuttimeInWeek(double sec)  //???
{
	double rettime = sec;
	if(rettime > SECONDS_IN_HALF_WEEK)
	{
    	rettime = rettime - SECONDS_IN_WEEK;
	}
	else if (rettime < NEG_SECONDS_IN_HALF_WEEK)
    {
    	rettime = rettime + SECONDS_IN_WEEK;
	}
	return rettime;
}
 double Kepler_equation(double e,double M_)
{
	int iter;
	double ek1,ek2;
	ek1 = M_;    
	for( iter=0;iter<20;iter++ )
	{
		ek2 =  M_ + e * sin(ek1);
		if(fabs(ek2-ek1)<=1.0E-20)
		{
			break;
		}
		ek1 = ek2;
	}
	return ek2;
}

double AdjustValue(double invalue,  double Left, double Right)
{
	double   Length;
	double value = invalue;
	short LoopCount = 0;	
	Length = Right - Left;	
	while( value<Left )
	{
		value = value + Length;
		
		// 控制循环次数
		LoopCount++;
		if( LoopCount>=20 )
			break;
	}	
	LoopCount = 0;
	while( value>=Right )
	{
		value = value - Length;
		// 控制循环次数
		LoopCount++;
		if( LoopCount>=20 )
			break;
	}
	
	return value;
}

double CalcCrossVel(int32 svid, ECEF SVPos, ECEF SV_vel, ECEF rcvrPos, ECEF rcvrVel)
{
	double xvs, yvs, zvs, xls, yls, zls, dis;

	xvs = rcvrVel.x - SV_vel.x;
	yvs = rcvrVel.y - SV_vel.y;
	zvs = rcvrVel.z - SV_vel.z;
	
	xls = SVPos.x - rcvrPos.x;
	yls = SVPos.y - rcvrPos.y;
	zls = SVPos.z - rcvrPos.z;
	
	dis = sqrt(xls*xls + yls*yls + zls*zls);
	dis = (xvs*xls+yvs*yls+zvs*zls) / dis;
	
	return dis;
}


int32 CalcSVEPHAge(int32 sv_id, int32 wn, int32 sec_of_week)
{
	int32 eph_age = SV_EPH_ALM_AGE_INVALID;

	int32 toe_next_eph;
	int32 wn_next_eph;//= SVInfo[sv_id-1].eph.wkn;
	int32 ephAgeUseTh=0;
#if SUPPORT_GLONASS
	int32 N4,NT;
	double toe_glo;
	int32 glotoe;
#endif

	if(SV_IsGps(sv_id)||SV_IsBd2(sv_id))
	{
		if(Sys3AlmEphUTCInfo.GpsBd2_EphStruct[sv_id-MinGpsSvID].vflg==FALSE)
			return SV_EPH_ALM_AGE_INVALID;

		toe_next_eph = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[sv_id-MinGpsSvID].toe;
		wn_next_eph = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[sv_id-MinGpsSvID].wkn;

		if(SV_IsGps(sv_id))
		{
			if (toe_next_eph < SECONDS_IN_HOUR)
				wn_next_eph ++;
		}
		else	//BD
		{
			sec_of_week -= GPS_BD_SYSTIME_OFFSET;
			toe_next_eph += SECONDS_IN_HOUR;
		}
	}
#if SUPPORT_GLONASS
	else if(SV_IsGlo(sv_id))
	{
		if(Sys3AlmEphUTCInfo.Glo_EphStruct[sv_id-MinGloFreID].vflg==FALSE)
			return SV_EPH_ALM_AGE_INVALID;

		glotoe = (Sys3AlmEphUTCInfo.Glo_EphStruct[sv_id-MinGloFreID].tb) *60;
		N4 = Sys3AlmEphUTCInfo.Glo_EphStruct[sv_id-MinGloFreID].N4;
		NT = Sys3AlmEphUTCInfo.Glo_EphStruct[sv_id-MinGloFreID].NT;
		GloToGpsTime(N4,NT,(double)(glotoe),&wn_next_eph,&toe_glo);	
		toe_next_eph = (int32) (toe_glo+900);//15min
	}
#endif
	else
		return SV_EPH_ALM_AGE_INVALID;

	eph_age = (wn - wn_next_eph)*SECONDS_IN_WEEK + sec_of_week - toe_next_eph;

#if SUPPORT_GLONASS
	if(SV_IsGlo(sv_id) )
		ephAgeUseTh = GLO_SV_EPH_AGE_USABLE_THRES;
	else
#endif
		ephAgeUseTh = SV_EPH_AGE_USABLE_THRES;
		
	if((eph_age<SV_MIN_EPH_AGE) || (eph_age>ephAgeUseTh))
	{
	#ifndef _POSTPROC
	#ifndef _SIMULATE
		ClearEphBackupBitmap(sv_id); //clear bitmap in backup region	
	#endif
	#endif
		eph_age = SV_EPH_ALM_AGE_INVALID;
#if SUPPORT_GLONASS
		if(SV_IsGlo(sv_id))
			Sys3AlmEphUTCInfo.Glo_EphStruct[sv_id-MinGloFreID].vflg = FALSE;
		else
#endif
			Sys3AlmEphUTCInfo.GpsBd2_EphStruct[sv_id-MinGpsSvID].vflg = FALSE;
	}

	return eph_age;	
}


int32 CalcSVALMAge(int32 sv_id, int32 wn, int32 sec_of_week)
{
#if SUPPORT_GLONASS
	int32 alm_age = SV_EPH_ALM_AGE_INVALID,Gloalm_age = SV_EPH_ALM_AGE_INVALID;
	int32 wn_alm ,NA,N4,toa;
	double Dtow;
	int32 iloop=0,GloAgeflag =0;
	int32 GloSvid[2]={0,0};
	int32 GloN = 0,AlmIndex = 0;
#else
	int32 alm_age = SV_EPH_ALM_AGE_INVALID;
	int32 wn_alm ,toa;
#endif
	if(SV_IsGps(sv_id) || SV_IsBd2(sv_id))
	{
		if(Sys3AlmEphUTCInfo.GpsBd2_AlmStruct[sv_id-MinGpsSvID].vflg == FALSE)
			return SV_EPH_ALM_AGE_INVALID;
		
		wn_alm = Sys3AlmEphUTCInfo.GpsBd2_AlmStruct[sv_id-MinGpsSvID].refweek;
		toa = Sys3AlmEphUTCInfo.GpsBd2_AlmStruct[sv_id-MinGpsSvID].toa;

		if(SV_IsBd2(sv_id))
		{
			sec_of_week -= GPS_BD_SYSTIME_OFFSET;
			wn_alm ++;
		}
		
		alm_age = (wn - wn_alm) * SECONDS_IN_WEEK + (sec_of_week - toa);

		if((alm_age < SV_MIN_ALM_AGE) || (alm_age > SV_ALM_AGE_USABLE_THRES))
		{
		#ifndef _POSTPROC
		#ifndef _SIMULATE
			ClearAlmBackupBitmap(sv_id); //clear bitmap in backup region	
		#endif
		#endif
			Sys3AlmEphUTCInfo.GpsBd2_AlmStruct[sv_id-MinGpsSvID].vflg = FALSE;
		}
	}
#if SUPPORT_GLONASS
	else if(SV_IsGlo(sv_id))
	{
		if((!GloFreidToSvid(sv_id,GloSvid,&GloN)) || (GloN==0))
			return SV_EPH_ALM_AGE_INVALID;
		
		for (iloop=0;iloop<GloN;iloop++)
		{
			AlmIndex = GloSvid[iloop];
			NA = (Sys3AlmEphUTCInfo.Glo_AlmStruct[AlmIndex-1].Day) % 1461 +1;
			N4 = (Sys3AlmEphUTCInfo.Glo_AlmStruct[AlmIndex-1].Day) / 1461 +1;
			GloToGpsTime(N4,NA,0.0,&wn_alm,&Dtow);
			
			toa = Dtow;
			Gloalm_age = (wn - wn_alm) * SECONDS_IN_WEEK + (sec_of_week - toa);
			if (abs(Gloalm_age) < SV_ALM_AGE_USABLE_THRES)///两个卫星的都有效，才出来
				GloAgeflag ++;
			else
			{
				//ClearAlmBackupBitmap(sv_id); //clear bitmap in backup region	
				Sys3AlmEphUTCInfo.Glo_AlmStruct[AlmIndex].vflg = FALSE;
			}
		}

		if(GloAgeflag == GloN)
			alm_age = Gloalm_age;
		else
			alm_age = SV_EPH_ALM_AGE_INVALID;
	}
#endif
	return  alm_age;	
}


/**
*	@brief   This routine calculates SV elevation & azimuth.
*
*	@param SVPos	SV position
*	@param rcvrPos	receiver position
*	@param pAz		pointer to azimuth
*	@param pEl		pointer to elevation
*	@return N/A
*/
void CalcSVAngle(ECEF SVPos, ECEF rcvrPos, float64 *pAz, float64 *pEl)
{
	NEH sv_neh;
	float64 az, el, dis;
		
	if (pAz!=NULL && pEl!=NULL)
	{
		// 输入SVPos、rcvrPos，输出sv_neh
		if( ECEF2NEH(&SVPos, &rcvrPos, &sv_neh) != 0)
			return;

		dis = sqrt(sv_neh.north*sv_neh.north + sv_neh.east*sv_neh.east + sv_neh.head*sv_neh.head);
		
		az = atan2(sv_neh.east, sv_neh.north);
		el = asin(sv_neh.head/dis);
		
		az *= R2D;
		el *= R2D;
		
		if (az < 0)
		{
			az += 360;
		}

		*pAz = az;
		*pEl = el;
	}
}

int32 CheckEphHealth(int32 SvID)
{
#if SUPPORT_GLONASS
	word16 Glo_Bn = 0;
#endif
	if (SV_IsGps(SvID)||SV_IsBd2(SvID))
	{
		if ((Sys3AlmEphUTCInfo.GpsBd2_EphStruct[SvID-1].vflg == 1) && (SVInfo[SvID-1].eph_age<SV_EPH_AGE_USABLE_THRES)) //星历有效
		{
			if ( (Sys3AlmEphUTCInfo.GpsBd2_EphStruct[SvID-1].s1hlth!=0) || (Sys3AlmEphUTCInfo.GpsBd2_EphStruct[SvID-1].ura>7) )  // 子帧1健康码及测距精度判断
	        {    
	        	return -1;
			}

			return 1;
		}	
	}
#if SUPPORT_GLONASS
	else if (SV_IsGlo(SvID))
	{
		if ((Sys3AlmEphUTCInfo.Glo_EphStruct[SvID-MinGloFreID].vflg == 1) && (SVInfo[SvID-1].eph_age<GLO_SV_EPH_AGE_USABLE_THRES)) //星历有效
		{
			Glo_Bn = ((Sys3AlmEphUTCInfo.Glo_EphStruct[SvID-MinGloFreID].Bn)>>2)&0x01;
			if ((Glo_Bn == 1) || (Sys3AlmEphUTCInfo.Glo_EphStruct[SvID-MinGloFreID].FT>10) )  // 子帧1健康码及测距精度判断
	        {    
	        	return -1;
			}

			return 1;
		}	
	}
#endif
	return 0;
}

int32 CheckAlmHealth(int32 SvID)
{
	if (SV_IsGps(SvID)||SV_IsBd2(SvID))
	{
		if ((Sys3AlmEphUTCInfo.GpsBd2_AlmStruct[SvID-1].vflg == 1) && (SVInfo[SvID-1].alm_age<SV_ALM_AGE_USABLE_THRES)) //
		{
			if ( (Sys3AlmEphUTCInfo.GpsBd2_AlmStruct[SvID-1].almhlth!=0) )  // 
			{	 
				return -1;
			}

			return 1;
		}	
	}
#if SUPPORT_GLONASS
	else if (SV_IsGlo(SvID))
	{
		if ((Sys3AlmEphUTCInfo.Glo_AlmStruct[SvID-MinGloFreID].vflg == 1) && (SVInfo[SvID-1].eph_age<GLO_SV_EPH_AGE_USABLE_THRES)) //星历有效
		{
			if ((Sys3AlmEphUTCInfo.Glo_AlmStruct[SvID-MinGloFreID].HealthSvflag== 0))  // 
			{	 
				return -1;
			}

			return 1;
		}	
	}
#endif
	return 0;
}

void GetSvTropoInfo(int32 trkch, double* pTropo)
{
#ifndef _POSTPROC
	ECEF rcvrecefpos={0.0,0.0,0.0};
	WGS rcvrwgspos={0.0,0.0,0.0};
	int32 histposflag;
	//int32 svid = PVTTrkchInfo[trkch].svid;
	int32 svid = GetTRKChSvid(trkch);
	double el=0.0;

	if ((pActiveCPT->SysmCptWorkConfig.FunSwitch & FUNSWITCH_TROPOCORRECT ) == 0)
		return;

	histposflag = getRcvrHistoricalPos(&rcvrecefpos, NULL); 
	if((histposflag > HISTORICAL_POS_LEVEL_40KM) || (ECEF2WGS(&rcvrecefpos, &rcvrwgspos) != 0))
		return;

	if(SVInfo[svid-1].el > SV_ELEVATION_VALID_TH)
		CalcSVAngle(PVTTrkchInfo[trkch].svpos, rcvrecefpos, &(SVInfo[svid-1].az), &(SVInfo[svid-1].el));

	el = SVInfo[svid-1].el * D2R;

    if ( fabs(rcvrwgspos.alt) < 200.0 )
	{
		(*pTropo) = 2.47/(sin(el)+0.0121);
    }
	else
	{
		(*pTropo) = HopfiedTroposphereCorr(el, rcvrwgspos.alt);
	}

	// 限制范围
	if ( ((*pTropo)<0.0) || ((*pTropo)>100.0) )
	{
		(*pTropo) = 0.0;
	}
#endif
	return;
}

double HopfiedTroposphereCorr(double fElevation, double fStationHgt)
{	
	double theta,ae,sinth,costh,alpha[9], r, a, b, E,TropError,h[2], n[2],Temperature,Pressure,ES;
	int k,i;
    TropError=0.0;
    Temperature=0.0;
    Pressure=0.0;
    ES=0.0;
    k=0;	
	if (fStationHgt>44247.787)
	{
		return 0.0;
	}
	
	MeteorologicExt(&fStationHgt, &Temperature, &Pressure, &ES);	
	Temperature += 273.15;	
	
	n[0]  = 0.776E-4*Pressure/Temperature;               //Ndry
	n[1]  = 0.373*ES/(Temperature*Temperature);          //Nwet	
	h[0]  = 40.136+0.14872*(Temperature-273.15);         //Hdry
	h[1]  = 11.0;//Hwet		
	theta = fElevation;
	ae    = 6378.137 + fStationHgt*0.001;	
	sinth = sin(theta);
	costh = cos(theta);	
	for(  i = 0 ; i < 2 ; i++)
	{
		r = sqrt(pow((ae+h[i]),2) - pow((ae*costh),2)) - ae*sinth;
		a = -sinth/h[i];
		b = -costh*costh/(2.*h[i]*ae);
		alpha[0] = 1.;
		alpha[1] = 4.*a;
		alpha[2] = 6.*a*a+4.*b;
		alpha[3] = 4.*a*(a*a+3.*b);
		alpha[4] = a*a*a*a+12.*a*a*b+6.*b*b;
		alpha[5] = 4.*a*b*(a*a+3.*b);
		alpha[6] = b*b*(6.*a*a+4.*b);
		alpha[7] = 4.*a*b*b*b;
		alpha[8] = b*b*b*b;		
		E = 0.0;					
		for(k=0 ; k<9 ; k++)
			E = E + alpha[k]/(k+1)*pow(r,k+1);
		TropError = TropError + 1.e3 * E * n[i];		
	}
	return TropError;
}   



 void MeteorologicExt(double *IN_StationHgt,  double  *StationTemperature, double *StationPressure,  double *StationES )
{
	*StationTemperature=SurfaceTemperatureOfSea-(*IN_StationHgt)*0.0065;	
	*StationPressure=SurfacePressureOfSea*pow((1.0-(2.26E-5)*(*IN_StationHgt)),5.225);
	*StationES=RelativeHumidity*exp(-(6.396E-4)*(*IN_StationHgt));		
	*StationES = ((*StationES)*0.01)*exp(-37.2465+0.213166*(*StationTemperature+273.15)-0.000256908*(*StationTemperature+273.15)*(*StationTemperature+273.15));
	if( (*StationES)>100.0 )
		*StationES = 100.0;	
}

byte GetSvIonoInfo(int32 trkch, double ts, double* pIono)
{
	ECEF rcvrecefpos={0.0,0.0,0.0};
	WGS rcvrwgspos={0.0,0.0,0.0};
	byte ionoSrc = IONO_SRC_NONE;
	IonoStruct *pIonoStruct;
	int32 histposflag;
	//int32 svid = PVTTrkchInfo[trkch].svid;
	int32 svid = GetTRKChSvid(trkch);
	double el=0.0, az=0.0;

	if ((pActiveCPT->SysmCptWorkConfig.FunSwitch & FUNSWITCH_IONOCORRECT) == 0)
		return ionoSrc ;
		
	histposflag = getRcvrHistoricalPos(&rcvrecefpos, NULL); 
	if((histposflag > HISTORICAL_POS_LEVEL_40KM) || (ECEF2WGS(&rcvrecefpos, &rcvrwgspos) != 0))
		return ionoSrc;

	if(SVInfo[svid-1].el > SV_ELEVATION_VALID_TH)
		CalcSVAngle(PVTTrkchInfo[trkch].svpos, rcvrecefpos, &(SVInfo[svid-1].az), &(SVInfo[svid-1].el));

	el = SVInfo[svid-1].el * D2R;
	az = SVInfo[svid-1].az * D2R;

	if((Sys3AlmEphUTCInfo.Gpsionoutc.vflg == 1) && SV_IsGps(svid))
	{
		pIonoStruct = &Sys3AlmEphUTCInfo.Gpsionoutc;
		ionoSrc = IONO_SRC_GPS_ALM;
	}
	else if((Sys3AlmEphUTCInfo.MeoD1Iono.vflg == 1) && SV_IsBd2Meo(svid))
	{
		pIonoStruct = &Sys3AlmEphUTCInfo.MeoD1Iono;
		ionoSrc = IONO_SRC_BD2_MEO;
	}
	else if((Sys3AlmEphUTCInfo.GeoD2Iono.vflg == 1) && SV_IsBd2Geo(svid))
	{
		pIonoStruct = &Sys3AlmEphUTCInfo.GeoD2Iono;
		ionoSrc = IONO_SRC_BD2_GEO;
	}
	else
		return IONO_SRC_NONE;

#if 1
	if(ionoSrc == IONO_SRC_GPS_ALM)
		(*pIono) = IONOSPHERE_GetL1KlobucharCorrection(ts, &Sys3AlmEphUTCInfo.Gpsionoutc, rcvrwgspos.lat, rcvrwgspos.lon, rcvrwgspos.alt, el, az);
	else
		(*pIono) = Meo_IonoSphericCorrModel(ts, pIonoStruct, rcvrwgspos.lat, rcvrwgspos.lon, rcvrwgspos.alt, el, az);	
#else
		(*pIono) = IONOSPHERE_GetL1KlobucharCorrection(ts, pIonoStruct, rcvrwgspos.lat, rcvrwgspos.lon, rcvrwgspos.alt, el, az);
#endif
	// 限制范围
	if ( ((*pIono)<5.0e-9) || ((*pIono)>100.0) )
	{
		(*pIono) = 0.0;
	}
	
	return ionoSrc;
}


double Meo_IonoSphericCorrModel(double fSec, const IonoStruct *ionoutc, double fLat, double fLon, double fAlt, double fElvRad, double fAziRad)
{
	double  es, a, phu, lmu, temp, f, psi, phi, lmi, phm, phm2, phm3,suma, sumb, sumc, xtemp, tlocal, ic, A4ofFour;
	const double re = 6378.0, hion = 375.0;
	double cosa,cose,rp,sinap,tanap,tanu;
	
	ic = 0.0;
	es = fElvRad; 
	if (es<=0.0 || fAlt<-1000.0|| fAlt>375000|| ionoutc->vflg==0)
	{
	   ic=0.0;
	   return ic;
	}	                      
	a = fAziRad;
	phu = fLat; 
	lmu = fLon; 
	cosa = cos(a); 
	cose = cos(es);
	rp=re/(re+hion)*cose;
	psi=PI*0.5-es-asin(rp);
	sinap=sin(psi);
	tanap=tan(psi);
	tanu =tan(PI*0.5-phu);   
	phi=asin(sin(phu)*cos(psi)+cos(phu)*sinap*cosa);    
	if ((phi> (70.0*D2R)&& tanap*cosa>tanu)||
	   (phi<(-70.0*D2R)&&-tanap*cosa>tan(PI*0.5+phu)))
	   lmi=lmu+PI-asin(sinap*sin(a)/cos(phi));
	else
	   lmi=lmu+asin(sinap*sin(a)/cos(phi));
	lmi=lmi * RECIP_PI;
	tlocal = fSec + SECONDS_IN_DAY*0.5*lmi;
	tlocal-=floor(tlocal/86400.0)*86400.0;	
	temp = 0.53 - es* RECIP_PI;
	f = 1.0 + 16.0*temp*temp*temp;
	phm = phi* RECIP_PI;
	phm2 = phm*phm;
	phm3 = phm2*phm;
	//A2
	suma = ionoutc->a0 + ionoutc->a1*phm + ionoutc->a2*phm2 + ionoutc->a3*phm3;
	if(suma<0.0)
	   suma=0.0;
	//A4
	sumb = ionoutc->b0 + ionoutc->b1*phm + ionoutc->b2*phm2 + ionoutc->b3*phm3;
	if(sumb<72000.0 )
	   sumb = 72000.0;	
	sumc=tlocal-50400.0;
	xtemp = 2.0*PI*(tlocal-50400.0)/sumb;		
	A4ofFour= sumb*0.25;	
	if(fabs(sumc)<A4ofFour)
	{
	   xtemp = 2.0*PI*sumc/sumb;	
	   ic = f*(5.0e-9+suma*cos(xtemp))*SPEED_OF_LIGHT;
	}
	else
	{
		ic = f*(5.0e-9)*SPEED_OF_LIGHT;
	}

	return ic;	
}


double IONOSPHERE_GetL1KlobucharCorrection(double fSec, const IonoStruct *ionoutc, double fLat, double fLon, double fAlt, double fElvRad, double fAziRad)
{
	double es, a, phu, lmu, temp, f, psi, phi, lmi, phm, phm2, phm3,suma, sumb, xtemp,  tlocal, ic;
	double x2,x4;
	IonoStruct ionostruc;
	
    es = fElvRad* RECIP_PI;           
    a = fAziRad;                           	
    phu = fLat * RECIP_PI; 
    lmu = fLon* RECIP_PI; 
	
    ic = 0.0;    
    if (es<=0.0 || fAlt<-1000.0 || fAlt>350000)
    {
        ic = 0.0;
		return ic;
    }
	
   /* if (ionoutc->vflg==0)
    {  
    #if 0
		ionostruc.a0=1.024454832077026e-008;
		ionostruc.a1=7.450580596923828e-009;
		ionostruc.a2=-5.960464477539063e-008;
		ionostruc.a3=-5.960464477539063e-008;
		ionostruc.b0=88064.0;
		ionostruc.b1=0.0;
		ionostruc.b2=-196608.0;
		ionostruc.b3=-65536.0;
	#else 
		ionostruc.a0=4.6566129e-009;
		ionostruc.a1=1.4901161e-008;
		ionostruc.a2=-5.96046e-008;
		ionostruc.a3=-5.96046e-008;
		ionostruc.b0=79872.0;
		ionostruc.b1=65536.0;
		ionostruc.b2=-65536.0;
		ionostruc.b3=-393216.0;
	#endif
		return 0;
    } 
    else
    */
	{
		ionostruc= *ionoutc;
    }
	
    psi = 0.0137/(es+0.11) - 0.022;	   
    phi = phu + psi*cos(a);	   
    if(phi>0.416)
        phi = 0.416;
    else if	(phi<(-0.416))
		phi = -0.416;			
	lmi = lmu + psi*sin(a)/cos(phi*PI);			
	tlocal = fSec + SECONDS_IN_DAY/2.0*lmi;
	tlocal-=floor(tlocal/86400.0)*86400.0;
	temp = 0.53 - es;
	f = 1.0 + 16.0*temp*temp*temp;			
	phm = phi + 0.064*cos((lmi-1.617)*PI);
	phm2 = phm*phm;
	phm3 = phm2*phm;		
	suma = ionostruc.a0 + ionostruc.a1*phm + ionostruc.a2*phm2 +ionostruc.a3*phm3;
	if(suma<0.0)
		suma=0.0;
	sumb = ionostruc.b0 + ionostruc.b1*phm + ionostruc.b2*phm2 + ionostruc.b3*phm3;
	if(sumb<72000.0 )
		sumb = 72000.0;		
	xtemp = 2.0*PI*(tlocal-50400.0)/sumb;		
	if(fabs(xtemp)<1.57)
	{		
		x2 = xtemp*xtemp;
        x4 = x2*x2;
        temp = 1.0 - x2*0.5 + x4*0.04166667;
        ic = f*(5.0e-9 + suma*temp)*SPEED_OF_LIGHT ;
		
	}
	else
	{
		ic = f * (5.0e-9) * SPEED_OF_LIGHT;
	}

	return ic;	
}

#if SUPPORT_GLONASS
void GloToGpsTime(int32 N4,int32 NT,double t, int32 *weekn, double *tow)
{
	word32 Day= 0, weekDay = 0;
	// GPS_GLO_SYSTIME_OFFSET虽然是全局的，但在所有地方都是读取操作，不涉及多线程冲突，怀疑是忘记了加const
	t = t + GPS_GLO_SYSTIME_OFFSET;
	Day = (N4-1)*1461 + NT-1 + MJD_GLO_GPS;
	weekDay = Day % 7;
	*weekn = Day / 7;
	*tow = t + ((double)weekDay * SECONDS_IN_DAY);

	if((*tow)<0)
	{
		*tow +=SECONDS_IN_WEEK;
		*weekn = (*weekn) -1;
	}
	else if ((*tow)>=SECONDS_IN_WEEK)
	{
		*tow -=SECONDS_IN_WEEK;
		*weekn = (*weekn) +1;
	}
	
	
	return;
}

void GpsToGloTime(int32 weekn,double tow,int32 *Day,double *t)
{
	double ts;
	int32 gloTimeDayInt;
	ts = tow - GPS_GLO_SYSTIME_OFFSET;
	gloTimeDayInt = ((int32)ts)/((int32)SECONDS_IN_DAY);
	*t = ts - gloTimeDayInt * SECONDS_IN_DAY;
	*Day = weekn*7 -MJD_GLO_GPS + gloTimeDayInt;
	return;
}

void GpsToGloTimeTod(double tow, double *tod)
{
	double ts;
	int32 gloTimeDayInt;
	ts = tow - GPS_GLO_SYSTIME_OFFSET;
	gloTimeDayInt = ((int32)ts)/((int32)SECONDS_IN_DAY);
	*tod = ts - gloTimeDayInt * SECONDS_IN_DAY;

	return;
}


bool CalcGloSVToGpsTime(int32 slotid, double sec_day, int32 *weekn, double *tow)
{
	int32 N4 = 0, NT = 0;
	
	if(Sys3AlmEphUTCInfo.Glo_EphStruct[slotid-MinGloFreID].vflg==FALSE)
		return FALSE;

	N4 = Sys3AlmEphUTCInfo.Glo_EphStruct[slotid-MinGloFreID].N4;
	NT = Sys3AlmEphUTCInfo.Glo_EphStruct[slotid-MinGloFreID].NT;

	GloToGpsTime(N4,NT,sec_day, weekn, tow);
	
	if((f_abs(Sys3AlmEphUTCInfo.Glo_EphStruct[slotid-MinGloFreID].tb * SECONDS_IN_MINUTE) - sec_day)> SECONDS_IN_HALF_DAY)
	{
		(*tow) += SECONDS_IN_DAY;
		if((*tow)>SECONDS_IN_WEEK)
		{
			(*tow) -= SECONDS_IN_WEEK;
			(*weekn)++;
		}	
	}

	return TRUE;
}

int32 ConvertGloSVID2SlotID(int32 svid)
{
	int32 slotid=-1;
	for(slotid=MinGloFreID; slotid<=MaxGloFreID; slotid++)
	{
		if((SVInfo[slotid-1].eph_age<GLO_SV_EPH_AGE_USABLE_THRES)
			&&(Sys3AlmEphUTCInfo.Glo_EphStruct[slotid-MinGloFreID].svid == svid-MinGloFreID+1))
			break;
	}

	if(slotid>MaxGloFreID)
		slotid=-1;

	return slotid;
}

int32 ConvertGloSlotID2SVID(int32 slotid)
{
	int32 svid=-1;
	if(SVInfo[slotid-1].eph_age<GLO_SV_EPH_AGE_USABLE_THRES)
		svid = Sys3AlmEphUTCInfo.Glo_EphStruct[slotid-MinGloFreID].svid+MinGloFreID-1;

	if(svid<MinGloFreID || svid>MaxGloSvIDTRUE)
		svid=-1;

	return svid;
}


#endif

