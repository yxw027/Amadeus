/*******************************************************************
Company:   hwacreate
Engineer:  
Create Date: 2015.12.01
File  Name:  RTKConstants.h
Description:constant and macro definition
Function List: 
version: V1.0
Revision Date:
Modifier: 
Additional Comments: 
********************************************************************/

#ifndef  RTKCONSTANTS_H__
#define  RTKCONSTANTS_H__

//global.h中包含了所有全局变量
#include "global.h"



//global.h中没有包含但需要使用的全局变量
//---------------------------------------------------------------------
// 常用常数宏定义 
//---------------------------------------------------------------------

//星座数
#define MAX_SAT_SYS_NUMBER (3)
//通道数
#define MAX_CHANNEL_TWO (72)

//频点数
#define MAX_FREQ	(6)		//1、2、3是B1、B2、B3，4、5、6是L1、L2、G1
//支路数
#define MAX_BRANCH (2)

// 弧度到度的转换
#define RADIAN_TO_DEGREE	(57.29577951308232)
// 弧度-->度
#define DEGREE_TO_RAD		( 0.017453292519943295769236907684886 )	// ( PI/180.0 )

// 可见卫星高度，单位：度
#define BD2_VISABLE_ELEVATION		(15.0)

// 一天和半天内的秒数
#define	SECONDS_ONE_DAY			(86400)

// 一周和半周内秒数
#define	SECONDS_HALF_WEEK		(302400)
#define	SECONDS_ONE_WEEK		(604800)

// 一年（平年和闰年）和四年之内的秒数（四年内包含一个闰年）
#define	SECONDS_COMMON_YEAR		(31536000)
#define	SECONDS_LEAP_YEAR		(31622400)
#define	SECONDS_FOUR_YEAR		(126230400)

// CGS2000坐标系下的光速，单位：米/秒
#define BD2_LIGHT_SPEED			(2.99792458E+8)
#define BD2_LIGHT_SPEED__1		(3.33564095198152e-009)

#define CGS2000_A			(6378137.0)        // 地球半长轴（米）
#define CGS2000_F			(1/298.257222101)		// 地球扁率的倒数
#define CGS2000_MIU			(3.986004418E+14)	// 地球引力常数，单位：米^3/秒^2
#define CGS2000_OMEGA_E		(7.2921150E-5)		// 地球自转速率，单位：弧度/秒

#define WGS84_A				(6378137.0)        // 地球半长轴（米）
#define WGS84_F				(1/298.257223563)		// 地球扁率的倒数
#define WGS84_MIU			(3.986005E+14)		// 地球引力常数，单位：米^3/秒^2
#define WGS84_OMEGA_E		(7.2921151467E-5)		// 地球自转速率，单位：弧度/秒

//Glonass PZ90 参考框架
#define GLN_OMIGA_E		( 7.292115E-5 )
#define GLN_MIU			( 3.9860044E14 )   
#define GLN_F_MA		( 3.5E8 )
#define GLN_EARTH_A		( 6.378136E6 )
#define GLN_F			( 1/298.257839303 )
#define GLN_C20			( -1.0826257E-3 )
#define GLN_C20_NORM	( -4.841650E-4 )
#define GLN_I_MEAN		( 63 * DEGREE_TO_RAD )
#define GLN_T_MEAN		( 43200.0 )
#define GLN_EARTH_E		(  0.0818191065283645 )
#define GLN_S0			( 6.02401539573 )
#define GLN_TINY		( 1E-2 )//微小值,用来终止迭代计算卫星位置
#define GLN_FREQ_OFFSET ( 7 )	//GLN频率通道号的偏移值，(-7,+6)加上7后变为(0, 13)

#define MAX_BD2_GEO_NUM			(5)
#define MAX_BD2_GEOIGSO_NUM		(10)
#define GPSBD2TIMEOFFSET  (14)

#define GPSUTCTIMEOFFSET         17
#define BD2UTCTIMEOFFSET         3

#define GEOSATMAXPRO             139   //119~139ms
#define GEOSATMINPRO             119
#define GEOSATMEANPRO            (129/1000.0)
#define MEOSATMAXPRO             90    //70~90ms
#define MEOSATMINPRO             70
#define MEOSATMEANPRO            (80/1000.0)

#define LIGHT_SPEED			(2.99792458E+8)
#define B3_CARRIER_FREQ     (1268.52E+6)
#define B2_CARRIER_FREQ     (1207.14E+6)
#define B1_CARRIER_FREQ     (1561.098E+6)
#define L1_CARRIER_FREQ     (1575.42E+6)
#define G1_CARRIER_FREQ     (1602E+6)
#define G1_INTER_FREQ		(0.5625E+6)

#define BD2_B3_CHIP_FREQ    (10.23E+6)
#define BD2_B1_CHIP_FREQ    (2.046E+6)

#define B3_CARRIER_WL       (LIGHT_SPEED/B3_CARRIER_FREQ)
#define B1_CARRIER_WL       (LIGHT_SPEED/B1_CARRIER_FREQ)
#define L1_CARRIER_WL       (LIGHT_SPEED/L1_CARRIER_FREQ)
#define G1_CARRIER_WL       (LIGHT_SPEED/G1_CARRIER_FREQ)

#define TESTSCENCEWN        (344)    //测试场景周计数


// 原始数据坐标系统为CGCS2000，克拉索夫斯椭球常数
#define CGCS2000_PI   3.14159265358979323846 // PI
#define CGCS2000_a   6378137.0 				// 椭球长半径(米)
#define CGCS2000_b   6356752.31414036 				// 椭球长半径(米)
#define CGCS2000_f   0.00335281068118232 				//1/298.257222101 扁率: (a - b) / a
#define CGCS2000_e2   0.00669438002290068 		// 1 - (b * b) / (a * a);
#define CGCS2000_e4   4.48147238910117e-005 
#define CGCS2000_e6   3.00006792347799e-007 
#define CGCS2000_e8   2.00835947742762e-009 
#define CGCS2000_e10   1.34447215644947e-011 
#define CGCS2000_e12   9.00040754548152e-014 	// e'2: (a*a)/(b*b)-1
#define CGCS2000_p0   57.2957795130823210 	// (度)
#define CGCS2000_p1   3437.74677078493917 	// (分)
#define CGCS2000_p2   206264.806247096355 	// (秒)
#define CGCS2000_B0   30 
#define CGCS2000_E0   500.0 
#define CGCS2000_N0   0 
#define CGCS2000_UTM   1 




#define sin_5	(-0.087155742747658173558064270837474)	// sin(5度)
#define cos_5	(0.99619469809174553229501040247389)	// cos(5度)


//-----------------------------------------------------------------------------
// 类型定义
//-----------------------------------------------------------------------------






	



	#ifndef NULL
	#ifdef  __cplusplus
		#define NULL    0
	#else
		#define NULL    ((void *)0)
	#endif
	#endif
	



#endif
