/*
 * constded.h
 *
 *  Created on: 2016-6-23
 *      Author: dell_dtd
 */
#ifndef CONSTDEF_H_
#define CONSTDEF_H_

#include "typedefine.h"
#include "define.h"

//sys time define
#define GPS_BD_SYSTIME_OFFSET		(14)		//unit: second, GPSTIME - BDTIME = 14S
#define GPS_BD_WN_OFFSET		    (1356)		//GPSWeekNumber - BDWeekNumer = 1356

#ifdef __cplusplus
extern "C" {
#endif
// ...............
extern const double B1_FREQUENCE;
extern const double B2_FREQUENCE;
extern const double B3_FREQUENCE;
extern const double L1_FREQUENCE;
extern const double L2_FREQUENCE;
extern const double L5_FREQUENCE;
#if SUPPORT_GLONASS
extern const double G1_FREQUENCE;
extern const double G2_FREQUENCE;
#endif
extern const double L1_WAVELENGTH;
extern const double L2_WAVELENGTH;
extern const double L5_WAVELENGTH;
extern const double B1_WAVELENGTH;
extern const double B2_WAVELENGTH;
extern const double B3_WAVELENGTH;
#if SUPPORT_GLONASS
extern const double G1_WAVELENGTH[14];
extern const double G2_WAVELENGTH[14];

extern const double G1_DETA_FREQUENCE;
extern const double G2_DETA_FREQUENCE;
#endif
extern const double TSTEP;//  =  60.0;
extern const double GEODESY_REFERENCE_ELLIPSE_WGS84_A ;// =     6.378137e6 ;
extern const double GEODESY_REFERENCE_ELLIPSE_WGS84_B ;// =     6356752.3142451793;
extern const double GEODESY_REFERENCE_ELLIPSE_CGS2000_B ;// =   6356752.3141403580;
extern const double GEODESY_REFERENCE_ELLIPSE_CGS2000_E2;// =   6.69438002290e-3;
extern const double BD2GravConstant ;//  = 	  3.986004418E14;
extern const double GPSGravConstant;//	=	   3.986005E14;
extern const double R2D;//=                	                    57.2957795131;
extern const double D2R;//=                                     0.017453292519943334;         /* deg to rad */
extern const double BIG_DOP;//=            	                    999.9;
extern const double SPEED_OF_LIGHT;//=     	                    2.99792458e8;
extern const double RECIP_SPEED_OF_LIGHT;// =                   3.33564095198e-9;
extern const double PI;									//	=					(3.1415926535898);
extern const double RECIP_PI;//=           	                    0.3183098861838;
extern const double TWO_PI;// =            	                    6.2831853071796;
extern const double SECONDS_IN_WEEK ;//=   	                    604800.00;
extern const double SECONDS_IN_HALF_WEEK ;// =    	            302400.00;
extern const double NEG_SECONDS_IN_HALF_WEEK ;//=               -302400.00;
extern const double SECONDS_IN_DAY ;//=    			            86400.00;
extern const double SECONDS_IN_HALF_DAY;
extern const double SECONDS_IN_HOUR ;//=   			            3600.00;
extern const double SECONDS_IN_MINUTE;// = 			            60.00;
extern const double DAYS_IN_FOUR_YEARS;// =        	            1461;
extern const double SurfacePressureOfSea;// =      	            1013.25;
extern const double SurfaceTemperatureOfSea;// =   	            18.0;
extern const double RelativeHumidity ;//= 			            50.0;
extern const double BD2WGS84oe;// =        			            7.292115E-5;
extern const double WGS84oe;// =          			            7.2921151467E-5;

//.....................
extern const double cos5;// =		                            0.996194698091745532295;
extern const double sin5;// =		                            -0.087155742747658173558;

extern const double RECIP_P1022000;// =                         9.78473581213307e-7;
extern const double RECIP_P1023000;// =                            9.77517106549e-7;
extern const double RECIP_P2046000 ;//=                            4.8875855327e-7;
extern const double RECIP_P20460000 ;
extern const double RECIP_P4092000 ;//=                            4.8875855327e-7;

extern const double RECIP_1000;// =                             0.001;
extern const double RECIP_P10230;// =                           9.775171065493e-5;
extern const double RECIP_P65536;// =                           0.0000152587890625;


extern const double TWO_P3;
extern const double TWO_P4;
extern const double TWO_P5;
extern const double TWO_P10;
extern const double TWO_P11;
extern const double TWO_P12;// =			                    4096.0;
extern const double TWO_P14;// =		                    	16384.0;			//2^14
extern const double TWO_P16;// =	                    		65536.0;		    //2^16
extern const double TWO_P19;
extern const double TWO_P24;
extern const double TWO_P29;
extern const double TWO_P31 ;//=								2147483648.0;		// 2^31
extern const double TWO_P32 ;//=								4294967296.0;		// 2^32
extern const double TWO_P33;
extern const double TWO_P43;
extern const double TWO_P55;


extern const double TWO_N4	;
extern const double TWO_N5	;//=		                    	(0.03125);					//!< 2^-5
extern const double TWO_N9 ;//=                               (0.001953125);
extern const double TWO_N10 ;//=                              (9.765625e-04);
extern const double TWO_N11 ;//=			                    (4.882812500000000e-004);	//!< 2^-11
extern const double TWO_N13 ;
extern const double TWO_N14;// = 								(6.103515625000000e-05);
extern const double TWO_N15 ;//= 								(3.051757812500000e-05);
extern const double TWO_N16 ;//= 	                            (1.52587890625e-05);
extern const double TWO_N18;// =  							(3.814697265625000e-06);
extern const double TWO_N19;// =			                    (1.907348632812500e-006);	//!< 2^-19
extern const double TWO_N20 ;//=		                    	(9.536743164062500e-007);	//!< 2^-20
extern const double TWO_N21;// =			                    (4.768371582031250e-007);	//!< 2^-21
extern const double TWO_N23;// = 								(1.19209289550781e-7);
extern const double TWO_N24;// =			                    (5.960464477539063e-008);	//!< 2^-24
extern const double TWO_N27 ;//=			                    (7.450580596923828e-009);
extern const double TWO_N29;// =			                    (1.862645149230957e-009);	//!< 2^-29
extern const double TWO_N30;// =			                    (9.313225746154785e-010);
extern const double TWO_N31;// =		                    	(4.656612873077393e-010);	//!< 2^-31
extern const double TWO_N32;
extern const double TWO_N33 ;//=			                    (1.164153218269348e-010);	//!< 2^-33
extern const double TWO_N38;// = 								(3.63797880709171e-12);
extern const double TWO_N40;// = 								(9.09494701772928e-13);
extern const double TWO_N43;// = 								(1.13686837721616e-13);	//!< 2^-43
extern const double TWO_N50;// = 								(8.88178419700125e-16);	//!< 2^-50
extern const double TWO_N55;// = 								(2.77555756156289e-17);	//!< 2^-55
extern const double TWO_N66;// = 								(1.35525271560688e-20);

extern const double PI_TWO_N19;//	=	                    	(5.992112452678286e-006);	//!< Pi*2^-19
extern const double PI_TWO_N38;//	=	                    	(1.142904749427469e-011);	//!< Pi*2^-38
extern const double PI_TWO_N23	;//=	                    	(3.745070282923929e-007);	//!< Pi*2^-23
extern const double PI_TWO_N43	;//=	                    	(3.571577341960839e-013);	//!< Pi*2^-43
extern const double PI_TWO_N31;//	=	                    	(1.462918079267160e-009);	//!< Pi*2^-31




extern const double RECIP_B1_FREQUENCE;// = (6.4057477493405282692053926146853e-10);  //1/1561.098e6;
extern const double RECIP_L1_FREQUENCE;// = (6.3475136788919780122126163181882e-10);  //1/1575.42e6;
extern const double RECIP_B3_FREQUENCE;// = (7.8832024721722952732317976854918e-10);  //1/1268.52e6;

extern const double MICRO_NUM;// =	(1.0e-12);

extern const double OMEGAE_WGS;// =	(7.2921151467E-5);		 // earth rotation rate rad/sec
extern const double SQRTEU_WGS;// =	(19964981.8432174);	 //sqrt(earth's universal gravitational parameter), EUGP = 3.986005e14
extern const double OMEGAE_CGS;// =	(7.2921150E-5);		 // earth rotation rate rad/sec
extern const double SQRTEU_CGS;// =	(19964980.3856653);	 //sqrt(earth's universal gravitational parameter), EUGP = 3.986004418e14
extern const double CONST_F;// =	(-4.442807633e-10);	 // const F of force on SV from earth

extern int32 GPS_GLO_SYSTIME_OFFSET;
//---------------- GPS system const  --------------//
//  WGS-84 ellipsoid parameters
extern const double SEMIMAJOR_AXIS;// = (6378137.0);		  //semi-major axis of the earth
extern const double SEMIMINOR_AXIS;// = (6356752.314);		 //semi-minor axis of the earth

extern const double PR_NOISE_DifCN0[];//



extern const float64 GEIOD_STEP;// = 10.0;	//define geiod grid step : 10degree
extern const int8 GEIOD_UNDULATION[19][36];//

#ifdef __cplusplus
}
#endif

#endif /* CONSTDED_H_ */
