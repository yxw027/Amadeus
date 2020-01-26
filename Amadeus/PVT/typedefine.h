#ifndef _TYPE_DEFINE_H
#define _TYPE_DEFINE_H

// Unsigned integer definitions (32bit, 16bit, 8bit) follow...
typedef unsigned int		    UINT32;
typedef unsigned short          UINT16;
typedef unsigned char           UINT8;
typedef unsigned long long      UINT40;

// Signed integer definitions (32bit, 16bit, 8bit) follow...

typedef unsigned char           byte;
typedef signed char           	int8;
typedef unsigned short          word16;
typedef signed short            int16;
typedef unsigned int		    word32;
typedef signed int              int32;
typedef double               	float64;

typedef unsigned char           BYTE;
typedef char           			CHAR;
typedef double               	DOUBLE;
typedef signed int				INT;
typedef unsigned int			UINT;
typedef unsigned short 			WORD;
//typedef unsigned int 			DWORD;


typedef unsigned char           boolean;

typedef int I4;
typedef unsigned int U4;

typedef long int I5;
typedef unsigned long int U5;

typedef char I1;
typedef unsigned char U1;

typedef short int I2;
typedef unsigned short int U2;

typedef float F4;
typedef double F8;

typedef float	R4;//single precision R4;
typedef double R8;//precision R8;
typedef char CX;


// Signed integer definitions (32bit, 16bit, 8bit) follow...
typedef int                     INT32;
typedef short                   INT16;
typedef long long               INT40;
typedef double               	FLOAT64;
#ifndef _SIMULATE
#ifndef _POSTPROC
typedef char           			INT8;
#endif
#endif

#define BOOL                    bool
//#define BOOL                    unsigned char
#define NULL                    0


#define FALSE                   0
#define TRUE                    1


//#define bool int

#define f_abs(x) 	(((x)>0.0)?(x):(-(x)))

#define LOCAL  static 


#define FAILED          0x00
#define SUCCESSFUL      0x01

#define FIELD_OFFSET(type, field) ((word32)(&(((type *)0)->field)))
#define FIELD_SIZE(type, field)  sizeof(((type *)0)->field)


typedef struct
{
	UINT32 wd[2];
}word64;

typedef struct
{
	UINT32 wd[8];
}word256;


typedef struct
{
	UINT32 high;
	UINT32 low;
}word64_rtc;

typedef struct
{
	UINT8 ip[16];
}C16;

#ifdef _SIMULATE 
#define SWI_disable()	{}
#define SWI_enable()	{}
#endif

#ifdef _POSTPROC
#define SWI_disable()	{}
#define SWI_enable()	{}
#endif

#endif

