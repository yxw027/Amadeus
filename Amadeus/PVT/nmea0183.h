#ifndef __NMEA_0183__
#define __NMEA_0183__


#include "typedefine.h"
#include "coordinate.h"
#include "define.h"

#ifndef _POSTPROC
#ifndef _SIMULATE
#include "assitFunc4Bin.h"
#endif
#endif

#define M_PER_S_TO_KNOT		(1.94)

typedef struct{
	//Time information
	//UTC_TIME time;

	//Position information
	float64 lat;		//format: ddmm.mmmm,
	float64 lon;		//format: dddmm.mmmm,
	float64 alt;		//altitude above the mean-sea-level, wgs.alt - geoid, unit: m
	float64 geoid;	//difference between WGS84 and mean-sea-level, unit: m
	
	//Velocity information
	float64 head;	//unit: degree
	float64 speed; //unit: m/s

	//for TD interface
	float64 nvel;
	float64 evel;
	ECEF vel;
	
	//System information
	byte gpsquality;		// 0 - invalid, 1 - SPS, 2 - DGPS, 3 - PPS, 4 - RTK, 5 - float RTK, 6 - DR, 7, manual input, 8 simulator
	byte fixstatus; 	// 1 - Not Fix, 2 - 2D, 3 - 3D
	byte positionstatus;	//A - valid, V - Invalid
	byte modeindicator; //A - autonomous, D - Differential, E - DR, M - Manual input, S - Simulator, N - invalid
	
	//Dop information
	float64 hdop;
	float64 pdop;
	float64 vdop;
	float64 gpstdop;
	float64 bdtdop;
#if SUPPORT_GLONASS
	float64 glotdop;
#endif

	//SV information
	word16 svnuminfix;
	word16 svnuminview;
	word256 fixsvmap;
	//NMEA_SV_INFO sv[MAX_SV_INVIEW_NUM]; 
}NMEA_DATA;


#ifdef __cplusplus
extern "C" {
#endif

extern UINT8 CalcCheckSum(const UINT8 *pBuff, UINT32 uLen);
extern void NMEA0183_Main(byte comid);
extern void Uart_Send_GGA(byte comid);
extern void Uart_Send_RMC(byte comid);
extern void Uart_Send_GLL(byte comid);
extern void Uart_Send_GSA(UINT16 u2SvType,byte comid);
extern void Uart_Send_GSV(UINT16 u2SvType,byte comid);
extern void Uart_Send_DHV(byte comid);
extern void Uart_Send_NEMA0183_NPR(byte comid);

#ifndef _POSTPROC
#ifdef _SIMULATE
extern void SenddataRS232_ATT(byte comid);
extern void SenddataRS232_DBG(void);
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif //__NMEA_0183__

