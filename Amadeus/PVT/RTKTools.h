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
#include "Rtk.h"
#include "RTKConstants.h"
#include "RTKCalBaseline.h"

void Trans_XYZ_to_BLH(CHAR cSystem, DOUBLE X, DOUBLE Y, DOUBLE Z, DOUBLE* B, DOUBLE* L, DOUBLE* H);

void Trans_BLH_to_XYZ(CHAR cSystem, DOUBLE B, DOUBLE L, DOUBLE H, DOUBLE *X, DOUBLE *Y, DOUBLE *Z);

DOUBLE GetBDREFParam(UINT32 nType, CHAR cSystem);

void ECEF2ENU_DeltaXYZ(BD2_BASELINE* baseline, ECEF* pBasePos);

void CGCS2000_CoordToGauss(EARTH_COORD *m_coord,GAUSS_COORD *Gauss);

DOUBLE atan_2(DOUBLE y, DOUBLE x);

void BD2GetYMDHMS(BD2_OBSTIME* time,BOOL utc);

void GPSGetYMDHMS(BD2_OBSTIME* time,BOOL gps);



