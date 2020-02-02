#ifndef GPSMODLE_H     //防止头文件被重复引用
#define GPSMODLE_H
//#include "stdafx.h"  //MFC放在第一个

//#include "math.h"
//#include <iomanip>   //setw(4)
//#include <fstream>
//#include <string.h>
/**/


void Sun1(int Year, int Month, int Day, double Hour, long double *SUNPOS);   //惯性系
void Moon(int Year, int Month, int Day, double Hour, long double *Pos); //地固系
void SUN(int Year, int Month, int Day, double Hour, long double *POS);  //地固系

void Solide_Tide(double gmst, int year, int month, int day, double hour, double XSTA[3], long double XSUN[4], long double XMON[4], double DXTIDE[3], double dj, double ut1);

double DJUL(int J1, int M1, double T);

int  newSUN(int IORSYS, int TABINT, double T, long double XYZ[]); //新的太阳坐标计算

void SUNPosition(double XMJD, long double *POS, long double &Rr, long double &Ll, long double &Bb);
int  QITPOL(int NEP, int NFCT, double TTAB[], long double TAB[], double T, long double XYZ[]);
void FK4FK5(long double XSTAR0[3], long double VSTAR0[3], long double PARAL0, long double VRAD0, long double XSTAR1[3], long double VSTAR1[3], long double PARAL1, long double VRAD1);

void PRAE(double XMJD, long double PRAEZ[3][3]);
void SPROD(long double X[3], long double Y[3], long double *SCAL, long double *R1, long double *R2);
double fjldy(int iyr, int imon, int iday, double rhour, double *jldy);
double ARCMOD(double X);
void MONSTO(double M, double M1, double F, double D, double V, double J, double E, double E2, double K, double *dl, double *db);

void DMLMTV(long double X[3], long double T[3][3], long double Y[3]);
extern void Nutation(double t, double n[3][3], double * Dpsi);
extern void Precession(double t, double p[3][3]);
extern double S0(double t);
extern double Julian(int tyear, int tmonth, int tday, double thour);
void MultMatrix1(double a[3][3], long double b[3], long double c[3]);


void Pole_Tide(int iy, int im, int id, double ih, double iminute, double isecond, double X0[], double erp[], double dr[]);

void Ocean_Tide(double X0[], double time[], double zhenfu[], double xiangwei[], double oceanNEU[], double namita, double oceanxyz[]);

int  Receiver_Offset(double X0[], double RDETXYZ[], double DReceiver_Offset[]);

int  Receiver_Variation(double Elev, double Azimuth, double RVarL1[], double RVarL2[], double &DReceiver_variationdelay);
int  Receiver_VariationNew(double Elev, double Azimuth, double RVarL1[], double RVarL2[], double &DReceiver_variationdelay,
	double FL1elem, double FL2elem); //考虑到GLONASS等不同频率需求
int  Receiver_HI(double Xsta[], double detH);

void Earthrotation_correction(double tau, double SAT[]); //地球自转改正


void Sat_offset(double SatPo[], long double SunPo[], double Offset[]);
void Relativity_correction(double SAT[], double STA[], double &relativity_delay);

void Pathrange_delay(double SAT[], double XSTA[], double &pathrange_dalay);


void Sat_Variation_correction(double SAT[], double Elev, double VarL1[], double &DSat_variationdelay);

void trop_gmf_map(double dmjd, double XSTA[], double Elev, double &gmfh, double &gmfw);
double tropodelay(double el, double XSTA[], double &wetzth, double &dryzth);

double min1(int  &t1, int &t2);


#endif
