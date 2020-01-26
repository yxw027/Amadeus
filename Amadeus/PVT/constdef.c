#include "constdef.h"

const double B1_FREQUENCE = 1561.098E6;
const double B2_FREQUENCE = 1207.14E6;
const double B3_FREQUENCE = 1268.52E6;
const double L1_FREQUENCE = 1575.42E6;
const double L2_FREQUENCE = 1227.6E6;
const double L5_FREQUENCE = 1176.45E6;
#if SUPPORT_GLONASS
const double G1_FREQUENCE = 1602.0E6;
const double G2_FREQUENCE = 1246.0E6;
#endif

const double L1_WAVELENGTH = 0.19029367279836490;
const double L2_WAVELENGTH = 0.24421021342456826;
const double L5_WAVELENGTH = 0.254828048790854;
const double B1_WAVELENGTH = 0.19203948631027648;
const double B2_WAVELENGTH = 0.24834936958430671;
const double B3_WAVELENGTH = 0.23633246460442089;

#if SUPPORT_GLONASS
const double G1_DETA_FREQUENCE = 562.5E3;
const double G2_DETA_FREQUENCE = 437.5E3;

const double G1_WAVELENGTH[14]=
{
	0.187597455043216,
	0.187531446086481,
	0.187465483565873,
	0.187399567432411,
	0.18733369763718,
	0.187267874131334,
	0.187202096866097,
	0.187136365792759,
	0.187070680862681,
	0.18700504202729,
	0.186939449238084,
	0.186873902446626,
	0.186808401604549,
	0.186742946663552,
};

const double G2_WAVELENGTH[14]=
{
	0.241196727912707,
	0.241111859254046,
	0.24102705029898,
	0.240942300984529,
	0.240857611247803,
	0.240772981026001,
	0.24068841025641,
	0.240603898876404,
	0.240519446823447,
	0.240435054035088,
	0.240350720448965,
	0.240266446002805,
	0.24018223063442,
	0.24009807428171,
};
#endif

const double TSTEP  =  30.0;
const double GEODESY_REFERENCE_ELLIPSE_WGS84_A  =     6.378137e6 ;
const double GEODESY_REFERENCE_ELLIPSE_WGS84_B  =     6356752.3142451793;
const double GEODESY_REFERENCE_ELLIPSE_CGS2000_B  =   6356752.3141403580;
const double GEODESY_REFERENCE_ELLIPSE_CGS2000_E2 =   6.69438002290e-3;
const double BD2GravConstant   = 	  3.986004418E14;
const double GPSGravConstant	=	   3.986005E14;

const double R2D=                	                    57.2957795131;
const double D2R=                                     0.017453292519943334;         /* deg to rad */

const double BIG_DOP=            	                    999.9;
const double SPEED_OF_LIGHT=     	                    2.99792458e8;
const double RECIP_SPEED_OF_LIGHT =                   3.33564095198e-9;
const double PI =                	                    3.1415926535898;
const double RECIP_PI=           	                    0.3183098861838;
const double TWO_PI =            	                    6.2831853071796;
const double SECONDS_IN_WEEK =   	                    604800.00;
const double SECONDS_IN_HALF_WEEK  =    	            302400.00;
const double NEG_SECONDS_IN_HALF_WEEK =               -302400.00;
const double SECONDS_IN_DAY =    			            86400.00;
const double SECONDS_IN_HALF_DAY =    			        43200.00;
const double SECONDS_IN_HOUR =   			            3600.00;
const double SECONDS_IN_MINUTE = 			            60.00;
const double DAYS_IN_FOUR_YEARS =        	            1461;
const double SurfacePressureOfSea =      	            1013.25;
const double SurfaceTemperatureOfSea =   	            18.0;
const double RelativeHumidity = 			            50.0;
const double BD2WGS84oe =        			            7.292115E-5;
const double WGS84oe =          			            7.2921151467E-5;


const double cos5 =		                            0.996194698091745532295;
const double sin5 =		                            -0.087155742747658173558;
const double RECIP_P1022000 =                         9.78473581213307e-7;
const double RECIP_P1023000 =                         9.77517106549e-7;
const double RECIP_P2046000 =                            4.8875855327e-7;
const double RECIP_P20460000 =                            4.8875855327e-8;
const double RECIP_P4092000 =							2.44379276637341e-007;
const double RECIP_1000 =                             0.001;
const double RECIP_P10230 =                           9.775171065493e-5;
const double RECIP_P65536 =                           0.0000152587890625;


const double TWO_P3	=	                    		8.0;
const double TWO_P4	=	                    		16.0;
const double TWO_P5	=	                    		32.0;
const double TWO_P10 =			                    1024.0;
const double TWO_P11 =			                    2048.0;
const double TWO_P12 =			                    4096.0;
const double TWO_P14 =		                    	16384.0;			//2^14
const double TWO_P16 =	                    		65536.0;		    //2^16
const double TWO_P19 =								524288.0;			//2^19
const double TWO_P24 =	                    		16777216.0;		    //2^24
const double TWO_P29 =	                    		536870912.0;		//2^29
const double TWO_P31 =								2147483648.0;		// 2^31
const double TWO_P32 =								4294967296.0;		// 2^32
const double TWO_P33 =								8589934592.0;		// 2^33
const double TWO_P43 =								8796093022208.0;		// 2^43
const double TWO_P55 =								36028797018963968.0;		// 2^55


const double TWO_N4 =								(0.0625);
const double TWO_N5	=		                    	(0.03125);					//!< 2^-5
const double TWO_N9 =                               (0.001953125);
const double TWO_N10 =                              (9.765625e-04);
const double TWO_N11 =			                    (4.882812500000000e-004);	//!< 2^-11
const double TWO_N13 =			                    (1.220703125e-4);	//!< 2^-13
const double TWO_N14 = 								(6.103515625000000e-05);
const double TWO_N15 = 								(3.051757812500000e-05);
const double TWO_N16 = 	                            (1.52587890625e-05);
const double TWO_N18 =  							(3.814697265625000e-06);
const double TWO_N19 =			                    (1.907348632812500e-006);	//!< 2^-19
const double TWO_N20 =		                    	(9.536743164062500e-007);	//!< 2^-20
const double TWO_N21 =			                    (4.768371582031250e-007);	//!< 2^-21
const double TWO_N23 = 								(1.19209289550781e-7);
const double TWO_N24 =			                    (5.960464477539063e-008);	//!< 2^-24
const double TWO_N27 =			                    (7.450580596923828e-009);
const double TWO_N29 =			                    (1.862645149230957e-009);	//!< 2^-29
const double TWO_N30 =			                    (9.313225746154785e-010);
const double TWO_N31 =		                    	(4.656612873077393e-010);	//!< 2^-31
const double TWO_N32 =		                    	(2.3283064365387e-010);	//!< 2^-32
const double TWO_N33 =			                    (1.164153218269348e-010);	//!< 2^-33
const double TWO_N38 = 								(3.63797880709171e-12);
const double TWO_N40 = 								(9.09494701772928e-13);
const double TWO_N43 = 								(1.13686837721616e-13);	//!< 2^-43
const double TWO_N50 = 								(8.88178419700125e-16);	//!< 2^-50
const double TWO_N55 = 								(2.77555756156289e-17);	//!< 2^-55
const double TWO_N66 = 								(1.35525271560688e-20);

const double PI_TWO_N19	=	                    	(5.992112452678286e-006);	//!< Pi*2^-19
const double PI_TWO_N38	=	                    	(1.142904749427469e-011);	//!< Pi*2^-38
const double PI_TWO_N23	=	                    	(3.745070282923929e-007);	//!< Pi*2^-23
const double PI_TWO_N43	=	                    	(3.571577341960839e-013);	//!< Pi*2^-43
const double PI_TWO_N31	=	                    	(1.462918079267160e-009);	//!< Pi*2^-31




const double RECIP_B1_FREQUENCE = (6.4057477493405282692053926146853e-10);  //1/1561.098e6;
const double RECIP_L1_FREQUENCE = (6.3475136788919780122126163181882e-10);  //1/1575.42e6;
const double RECIP_B3_FREQUENCE = (7.8832024721722952732317976854918e-10);  //1/1268.52e6;

const double MICRO_NUM =	(1.0e-12);

const double OMEGAE_WGS =	(7.2921151467E-5);		 // earth rotation rate rad/sec
const double SQRTEU_WGS =	(19964981.8432174);	 //sqrt(earth's universal gravitational parameter), EUGP = 3.986005e14
const double OMEGAE_CGS =	(7.2921150E-5);		 // earth rotation rate rad/sec
const double SQRTEU_CGS =	(19964980.3856653);	 //sqrt(earth's universal gravitational parameter), EUGP = 3.986004418e14
const double CONST_F =	(-4.442807633e-10);	 // const F of force on SV from earth


int32 GPS_GLO_SYSTIME_OFFSET = -3*3600;

//---------------- GPS system const  --------------//
//  WGS-84 ellipsoid parameters
const double SEMIMAJOR_AXIS = (6378137.0);		  //semi-major axis of the earth
const double SEMIMINOR_AXIS = (6356752.314);		 //semi-minor axis of the earth

const double PR_NOISE_DifCN0[] =
{
	1.0				,	   1.41253754462275 ,         1.99526231496888 ,         2.81838293126445	,
	3.98107170553497,          5.62341325190349,          7.94328234724281,          11.2201845430196	,
	15.8489319246111,          22.3872113856834,          31.6227766016838,          44.6683592150963	,
	63.0957344480193,          89.1250938133745,          125.892541179417,          177.827941003892	,
	251.188643150958,          354.813389233575,          501.187233627272,          707.945784384138	,
	1000.0			,	   1412.53754462275 ,         1995.26231496888 ,         2818.38293126445	,
	3981.07170553497,          5623.41325190349,          7943.28234724281,          11220.1845430196	,
	15848.9319246111,          22387.2113856834,          31622.7766016838,          44668.3592150963	,
	63095.7344480193,          89125.0938133746,          125892.541179417,          177827.941003892	,
	251188.643150958,          354813.389233575,          501187.233627273,          707945.784384137	,
	1000000.0		,	   1412537.54462275 ,         1995262.31496888 ,         2818382.93126446	,
	3981071.70553497,          5623413.25190349,          7943282.34724281,          11220184.5430196	,
};



const float64 GEIOD_STEP = 10.0;	//define geiod grid step : 10degree
const int8 GEIOD_UNDULATION[19][36]=
{
	//90 Degrees N:
	{13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13},

	//80 Degrees N:
	{3,1,-2,-3,-3,-3,-1,3,1,5,9,11,19,27,31,34,33,34,33,34,28,23,17,13,9,4,4,1,-2,-2,0,2,3,2,1,1},

	//70 Degrees N:
	{2,2,1,-1,-3,-7,-14,-24,-27,-25,-19,3,24,37,47,60,61,58,51,43,29,20,12,5,-2,-10,-14,-12,-10,-14,-12,-6,-2,3,6,4},

	//60 Degrees N:
	{2,9,17,10,13,1,-14,-30,-39,-46,-42,-21,6,29,49,65,60,57,47,41,21,18,14,7,-3,-22,-29,-32,-32,-26,-15,-2,13,17,19,6},

	//50 Degrees N:
	{-8,8,8,1,-11,-19,-16,-18,-22,-35,-40,-26,-12,24,45,63,62,59,47,48,42,28,12,-10,-19,-33,-43,-42,-43,-29,-2,17,23,22,6,2},

	//40 Degrees N:
	{-12,-10,-13,-20,-31,-34,-21,-16,-26,-34,-33,-35,-26,2,33,59,52,51,52,48,35,40,33,-9,-28,-39,-48,-59,-50,-28,3,23,37,18,-1,-11},

	//30 Degrees N:
	{-7,-5,-8,-15,-28,-40,-42,-29,-22,-26,-32,-51,-40,-17,17,31,34,44,36,28,29,17,12,-20,-15,-40,-33,-34,-34,-28,7,29,43,20,4,-6},

	//20 Degrees N:
	{5,10,7,-7,-23,-39,-47,-34,-9,-10,-20,-45,-48,-32,-9,17,25,31,31,26,15,6,1,-29,-44,-61,-67,-59,-36,-11,21,39,49,39,22,10},

	//10 Degrees N:
	{13,12,11,2,-11,-28,-38,-29,-10,3,1,-11,-41,-42,-16,3,17,33,22,23,2,-3,-7,-36,-59,-90,-95,-63,-24,12,53,60,58,46,36,26},

	//0 Degrees
	{22,16,17,13,1,-12,-23,-20,-14,-3,14,10,-15,-27,-18,3,12,20,18,12,-13,-9,-28,-49,-62,-89,-102,-63,-9,33,58,73,74,63,50,32},

	//10 Degrees S:
	{36,22,11,6,-1,-8,-10,-8,-11,-9,1,32,4,-18,-13,-9,4,14,12,13,-2,-14,-25,-32,-38,-60,-75,-63,-26,0,35,52,68,76,64,52},

	//20 Degrees S:
	{51,27,10,0,-9,-11,-5,-2,-3,-1,9,35,20,-5,-6,-5,0,13,17,23,21,8,-9,-10,-11,-20,-40,-47,-45,-25,5,23,45,58,57,63},

	//30 Degrees S:
	{46,22,5,-2,-8,-13,-10,-7,-4,1,9,32,16,4,-8,4,12,15,22,27,34,29,14,15,15,7,-9,-25,-37,-39,-23,-14,15,33,34,45},

	//40 Degrees S:
	{21,6,1,-7,-12,-12,-12,-10,-7,-1,8,23,15,-2,-6,6,21,24,18,26,31,33,39,41,30,24,13,-2,-20,-32,-33,-27,-14,-2,5,20},

	//50 Degrees S:
	{-15,-18,-18,-16,-17,-15,-10,-10,-8,-2,6,14,13,3,3,10,20,27,25,26,34,39,45,45,38,39,28,13,-1,-15,-22,-22,-18,-15,-14,-10},

	//60 Degrees S:
	{-45,-43,-37,-32,-30,-26,-23,-22,-16,-10,-2,10,20,20,21,24,22,17,16,19,25,30,35,35,33,30,27,10,-2,-14,-23,-30,-33,-29,-35,-43},

	//70 Degrees S:
	{-61,-60,-61,-55,-49,-44,-38,-31,-25,-16,-6,1,4,5,4,2,6,12,16,16,17,21,20,26,26,22,16,10,-1,-16,-29,-36,-46,-55,-54,-59},

	//80 Degrees S:
	{-53,-54,-55,-52,-48,-42,-38,-38,-29,-26,-26,-24,-23,-21,-19,-16,-12,-8,-4,-1,1,4,4,6,5,4,2,-6,-15,-24,-33,-40,-48,-50,-53,-52},

	//90 Degrees S:
	{-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30,-30}
};



