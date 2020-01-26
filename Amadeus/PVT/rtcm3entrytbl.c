
/**
 * 
 *
 * contains the implmentation control routines of RTCM3.x
 * It contains: RTCM3.x message handles and datafield parse or create tables.
 *
 * MODIFICATIONS:
 *		July. 9th, 2013 juan.gou
 *			-content: create this file
 *		 
 */
#include "define.h"
#include "typedefine.h"
#include "rtcm3.h"
#include "bititemproc.h"

/**************************RTCM3.x message handle table*******************************/
RTCM3_msg_entry RTCM3_MSG_TBL [] = 
{	//msgid,		maxpayloadlen, 	Recceive handler,			Transmitte handler,	,bitmap in CPT		
	{ 1001,		240, 	handle_RTCM3_RX_MSG_1001, 	NULL,						{0,0,0}},	//gps RTK
	{ 1002,		304, 	handle_RTCM3_RX_MSG_1002, 	NULL,						{0,0,0}},	//gps RTK
	{ 1003,		412, 	handle_RTCM3_RX_MSG_1003, 	NULL,						{0,0,0}},	//gps RTK
	{ 1004,		508, 	handle_RTCM3_RX_MSG_1004, 	handle_RTCM3_TX_MSG_1004,	{0,0,0}},	//gps RTK
	
	{ 1005,		19,		handle_RTCM3_RX_MSG_1005, 	handle_RTCM3_TX_MSG_1005,	{0,0,0}},
	{ 1006,		21,		handle_RTCM3_RX_MSG_1006, 	handle_RTCM3_TX_MSG_1006,	{0,0,0}},	//ref station

	{ 1012,		236, 	handle_RTCM3_RX_MSG_1012, 	handle_RTCM3_TX_MSG_1012,	{0,0,0}},	//glo RTK
	
	{ 1071,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//gps MSM1
	{ 1072,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//gps MSM2
	{ 1073,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//gps MSM3
	{ 1074,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//gps MSM4
	{ 1075,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//gps MSM5
	{ 1076,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//gps MSM6
	{ 1077,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//gps MSM7

	{ 1081,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//glo MSM1
	{ 1082,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//glo MSM2
	{ 1083,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//glo MSM3
	{ 1084,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//glo MSM4
	{ 1085,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//glo MSM5
	{ 1086,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//glo MSM6
	{ 1087,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//glo MSM7

	{ 1121,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//bd MSM1
	{ 1122,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//bd MSM2
	{ 1123,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//bd MSM3
	{ 1124,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//bd MSM4
	{ 1125,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//bd MSM5
	{ 1126,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//bd MSM6
	{ 1127,		RTCM3_PAYLOAD_MAX_LEN,		handle_RTCM3_RX_MSM, 	handle_RTCM3_TX_MSM,{0,0,0}},	//bd MSM7
	{ 1104, 	RTCM3_PAYLOAD_MAX_LEN,		NULL,					NULL,				{0,0,0}},	//

	{ 1019,		61,				handle_RTCM3_RX_MSG_1019, 			handle_RTCM3_TX_MSG_1019,{0,0,0}},	//GPS eph
	{ 1047,		61,				handle_RTCM3_RX_MSG_1047, 			handle_RTCM3_TX_MSG_1047,{0,0,0}},	//bd eph

	{ RTCM3_MSG_ID_INVALID, 	NULL ,	NULL,	0, {0,0,0}},
};


/**************************RTCM3.x message DF description: parse and create table*******************************/
BITITEM_FIELD_DESC RTCM3_MSG_1001_1004_Head_Desc[] =
{
	{3,	12,	U_T,		12,	0,	FIELD_OFFSET(RTCM3_MSG_1001_1004_HEAD, ReferStationID),		FIELD_SIZE(RTCM3_MSG_1001_1004_HEAD, ReferStationID),	},
	{4,	24,	U_T,		30,	0,	FIELD_OFFSET(RTCM3_MSG_1001_1004_HEAD, TOW),				FIELD_SIZE(RTCM3_MSG_1001_1004_HEAD, TOW),				},	
	{5,	54,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1001_1004_HEAD, SyncGNSSFlag),		FIELD_SIZE(RTCM3_MSG_1001_1004_HEAD, SyncGNSSFlag),	},	
	{6,	55,	U_T,		5,	0,	FIELD_OFFSET(RTCM3_MSG_1001_1004_HEAD, SVCnt),				FIELD_SIZE(RTCM3_MSG_1001_1004_HEAD, SVCnt),			},	
	{7,	60,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1001_1004_HEAD, DiverFreeSmooth),	FIELD_SIZE(RTCM3_MSG_1001_1004_HEAD, DiverFreeSmooth),	},	
	{8,	61,	B_T,		3,	0,	FIELD_OFFSET(RTCM3_MSG_1001_1004_HEAD, SmoothInterval),		FIELD_SIZE(RTCM3_MSG_1001_1004_HEAD, SmoothInterval),	},	

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};


BITITEM_FIELD_DESC RTCM3_MSG_1001_Data_Desc[] =
{
	{9,		0,	U_T,		6,	0,	FIELD_OFFSET(RTCM3_MSG_1001_DATA, svid),				FIELD_SIZE(RTCM3_MSG_1001_DATA, svid),				},
	{10,	6,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1001_DATA, L1CodeIndicate),		FIELD_SIZE(RTCM3_MSG_1001_DATA, L1CodeIndicate),		},
	{11,	7,	U_T,		24,	0,	FIELD_OFFSET(RTCM3_MSG_1001_DATA, L1Pseudorange),		FIELD_SIZE(RTCM3_MSG_1001_DATA, L1Pseudorange),		},
	{12,	31,	I_T,		20,	0,	FIELD_OFFSET(RTCM3_MSG_1001_DATA, L1PhaRangeDifPR),	FIELD_SIZE(RTCM3_MSG_1001_DATA, L1PhaRangeDifPR),	},
	{13,	51,	U_T,		7,	0,	FIELD_OFFSET(RTCM3_MSG_1001_DATA, L1LockTimeIdx),		FIELD_SIZE(RTCM3_MSG_1001_DATA, L1LockTimeIdx),		},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

BITITEM_FIELD_DESC RTCM3_MSG_1002_Data_Desc[] =
{
	{9,	0,	U_T,		6,	0,	FIELD_OFFSET(RTCM3_MSG_1002_DATA, svid),				FIELD_SIZE(RTCM3_MSG_1002_DATA, svid),				},
	{10,	6,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1002_DATA, L1CodeIndicate),		FIELD_SIZE(RTCM3_MSG_1002_DATA, L1CodeIndicate),		},
	{11,	7,	U_T,		24,	0,	FIELD_OFFSET(RTCM3_MSG_1002_DATA, L1Pseudorange),		FIELD_SIZE(RTCM3_MSG_1002_DATA, L1Pseudorange),		},
	{12,	31,	I_T,		20,	0,	FIELD_OFFSET(RTCM3_MSG_1002_DATA, L1PhaRangeDifPR),	FIELD_SIZE(RTCM3_MSG_1002_DATA, L1PhaRangeDifPR),	},
	{13,	51,	I_T,		7,	0,	FIELD_OFFSET(RTCM3_MSG_1002_DATA, L1LockTimeIdx),		FIELD_SIZE(RTCM3_MSG_1002_DATA, L1LockTimeIdx),		},
	{14,	58,	I_T,		8,	0,	FIELD_OFFSET(RTCM3_MSG_1002_DATA, L1PRAmbiguity),		FIELD_SIZE(RTCM3_MSG_1002_DATA, L1PRAmbiguity),		},
	{15,	66,	I_T,		8,	0,	FIELD_OFFSET(RTCM3_MSG_1002_DATA, L1CNR),				FIELD_SIZE(RTCM3_MSG_1002_DATA, L1CNR),			},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

BITITEM_FIELD_DESC RTCM3_MSG_1003_Data_Desc[] =
{
	{9,	0,	U_T,		6,	0,	FIELD_OFFSET(RTCM3_MSG_1003_DATA, svid),				FIELD_SIZE(RTCM3_MSG_1003_DATA, svid),				},
	{10,	6,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1003_DATA, L1CodeIndicate),		FIELD_SIZE(RTCM3_MSG_1003_DATA, L1CodeIndicate),		},
	{11,	7,	U_T,		24,	0,	FIELD_OFFSET(RTCM3_MSG_1003_DATA, L1Pseudorange),		FIELD_SIZE(RTCM3_MSG_1003_DATA, L1Pseudorange),		},
	{12,	31,	I_T,		20,	0,	FIELD_OFFSET(RTCM3_MSG_1003_DATA, L1PhaRangeDifPR),	FIELD_SIZE(RTCM3_MSG_1003_DATA, L1PhaRangeDifPR),	},
	{13,	51,	I_T,		7,	0,	FIELD_OFFSET(RTCM3_MSG_1003_DATA, L1LockTimeIdx),		FIELD_SIZE(RTCM3_MSG_1003_DATA, L1LockTimeIdx),		},
	{16,	58,	B_T,		2,	0,	FIELD_OFFSET(RTCM3_MSG_1003_DATA, L2CodeIndicate),		FIELD_SIZE(RTCM3_MSG_1003_DATA, L2CodeIndicate),		},
	{17,	60,	I_T,		14,	0,	FIELD_OFFSET(RTCM3_MSG_1003_DATA, L2PRDifL1PR),		FIELD_SIZE(RTCM3_MSG_1003_DATA, L2PRDifL1PR),		},
	{18,	74,	I_T,		20,	0,	FIELD_OFFSET(RTCM3_MSG_1003_DATA, L2PhaRangeDifL1PR),	FIELD_SIZE(RTCM3_MSG_1003_DATA, L2PhaRangeDifL1PR),	},
	{19,	94,	U_T,		7,	0,	FIELD_OFFSET(RTCM3_MSG_1003_DATA, L2LockTimeIdx),		FIELD_SIZE(RTCM3_MSG_1003_DATA, L2LockTimeIdx),		},
	
	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

BITITEM_FIELD_DESC RTCM3_MSG_1004_Data_Desc[] =
{
	{9,	0,	U_T,		6,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, svid),				FIELD_SIZE(RTCM3_MSG_1004_DATA, svid),				},
	{10,	6,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L1CodeIndicate),		FIELD_SIZE(RTCM3_MSG_1004_DATA, L1CodeIndicate),		},
	{11,	7,	U_T,		24,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L1Pseudorange),		FIELD_SIZE(RTCM3_MSG_1004_DATA, L1Pseudorange),		},
	{12,	31,	I_T,		20,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L1PhaRangeDifPR),	FIELD_SIZE(RTCM3_MSG_1004_DATA, L1PhaRangeDifPR),	},	
	{13,	51,	I_T,		7,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L1LockTimeIdx),		FIELD_SIZE(RTCM3_MSG_1004_DATA, L1LockTimeIdx),		},	
	{14,	58,	I_T,		8,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L1PRAmbiguity),		FIELD_SIZE(RTCM3_MSG_1004_DATA, L1PRAmbiguity),		},
	{15,	66,	I_T,		8,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L1CNR),				FIELD_SIZE(RTCM3_MSG_1004_DATA, L1CNR),			},	
	{16,	74,	B_T,		2,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L2CodeIndicate),		FIELD_SIZE(RTCM3_MSG_1004_DATA, L2CodeIndicate),		},	
	{17,	76,	I_T,		14,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L2PRDifL1PR),		FIELD_SIZE(RTCM3_MSG_1004_DATA, L2PRDifL1PR),		},
	{18,	90,	I_T,		20,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L2PhaRangeDifL1PR),	FIELD_SIZE(RTCM3_MSG_1004_DATA, L2PhaRangeDifL1PR),	},
	{19,	110,	U_T,		7,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L2LockTimeIdx),		FIELD_SIZE(RTCM3_MSG_1004_DATA, L2LockTimeIdx),		},
	{20,	117,	U_T,		8,	0,	FIELD_OFFSET(RTCM3_MSG_1004_DATA, L2CNR),				FIELD_SIZE(RTCM3_MSG_1004_DATA, L2CNR),			},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

BITITEM_FIELD_DESC RTCM3_MSG_1005_Desc[] =
{
	{3,		12,	U_T,		12,	0,	FIELD_OFFSET(RTCM3_MSG_1005_INFO, ReferStationID),		FIELD_SIZE(RTCM3_MSG_1005_INFO, ReferStationID),	},
	{21,		24,	U_T,		6,	0,	FIELD_OFFSET(RTCM3_MSG_1005_INFO, ITRF_year),			FIELD_SIZE(RTCM3_MSG_1005_INFO, ITRF_year),		},
	{22,		30,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1005_INFO, GPS_Indicate),		FIELD_SIZE(RTCM3_MSG_1005_INFO, GPS_Indicate),	},
	{23,		31,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1005_INFO, GLONASS_Indicate),	FIELD_SIZE(RTCM3_MSG_1005_INFO, GLONASS_Indicate),	},
	{24,		32,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1005_INFO, Galileo_Indicate),		FIELD_SIZE(RTCM3_MSG_1005_INFO, Galileo_Indicate),		},
	{141,	33,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1005_INFO, ReferStation_Indicate),	FIELD_SIZE(RTCM3_MSG_1005_INFO, ReferStation_Indicate),},
	{25,		34,	I_T,		38,	0,	FIELD_OFFSET(RTCM3_MSG_1005_INFO, RefPox_x),			FIELD_SIZE(RTCM3_MSG_1005_INFO, RefPox_x),			},
	{142,	72,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1005_INFO, Osci_Indicate),		FIELD_SIZE(RTCM3_MSG_1005_INFO, Osci_Indicate),		},
	{26,		74,	I_T,		38,	0,	FIELD_OFFSET(RTCM3_MSG_1005_INFO, RefPox_y),			FIELD_SIZE(RTCM3_MSG_1005_INFO, RefPox_y),			},
	{27,		114,	I_T,		38,	0,	FIELD_OFFSET(RTCM3_MSG_1005_INFO, RefPox_z),			FIELD_SIZE(RTCM3_MSG_1005_INFO, RefPox_z),			},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

BITITEM_FIELD_DESC RTCM3_MSG_1006_Desc[] =
{
	{3,		12,	U_T,		12,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, ReferStationID),		FIELD_SIZE(RTCM3_MSG_1006_INFO, ReferStationID),	},
	{21,		24,	U_T,		6,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, ITRF_year),			FIELD_SIZE(RTCM3_MSG_1006_INFO, ITRF_year),		},
	{22,		30,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, GPS_Indicate),		FIELD_SIZE(RTCM3_MSG_1006_INFO, GPS_Indicate),	},
	{23,		31,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, GLONASS_Indicate),	FIELD_SIZE(RTCM3_MSG_1006_INFO, GLONASS_Indicate),	},
	{24,		32,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, Galileo_Indicate),		FIELD_SIZE(RTCM3_MSG_1006_INFO, Galileo_Indicate),		},
	{141,	33,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, ReferStation_Indicate),	FIELD_SIZE(RTCM3_MSG_1006_INFO, ReferStation_Indicate),},
	{25,		34,	I_T,		38,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, RefPox_x),			FIELD_SIZE(RTCM3_MSG_1006_INFO, RefPox_x),			},
	{142,	72,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, Osci_Indicate),		FIELD_SIZE(RTCM3_MSG_1006_INFO, Osci_Indicate),		},
	{26,		74,	I_T,		38,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, RefPox_y),			FIELD_SIZE(RTCM3_MSG_1006_INFO, RefPox_y),			},
	{364,	112,	B_T,		2,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, QuCycle_Indicate),		FIELD_SIZE(RTCM3_MSG_1006_INFO, QuCycle_Indicate),	},
	{27,		114,	I_T,		38,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, RefPox_z),			FIELD_SIZE(RTCM3_MSG_1006_INFO, RefPox_z),			},
	{28,		152,	U_T,		16,	0,	FIELD_OFFSET(RTCM3_MSG_1006_INFO, AntHeight),			FIELD_SIZE(RTCM3_MSG_1006_INFO, AntHeight),			},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

BITITEM_FIELD_DESC RTCM3_MSG_1009_1012_Head_Desc[] = 
{
	{3,	    12,	U_T,		12,	0,	FIELD_OFFSET(RTCM3_MSG_1009_1012_HEAD, ReferStationID),		FIELD_SIZE(RTCM3_MSG_1009_1012_HEAD, ReferStationID),	},
	{34,	24,	U_T,		27,	0,	FIELD_OFFSET(RTCM3_MSG_1009_1012_HEAD, TOW),				FIELD_SIZE(RTCM3_MSG_1009_1012_HEAD, TOW),				},	
	{5,	    51,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1009_1012_HEAD, SyncGNSSFlag),		FIELD_SIZE(RTCM3_MSG_1009_1012_HEAD, SyncGNSSFlag),	},	
	{35,	52,	U_T,		5,	0,	FIELD_OFFSET(RTCM3_MSG_1009_1012_HEAD, SVCnt),				FIELD_SIZE(RTCM3_MSG_1009_1012_HEAD, SVCnt),			},	
	{36,	57,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1009_1012_HEAD, DiverFreeSmooth),	FIELD_SIZE(RTCM3_MSG_1009_1012_HEAD, DiverFreeSmooth),	},	
	{37,	58,	B_T,		3,	0,	FIELD_OFFSET(RTCM3_MSG_1009_1012_HEAD, SmoothInterval),		FIELD_SIZE(RTCM3_MSG_1009_1012_HEAD, SmoothInterval),	},	

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

BITITEM_FIELD_DESC RTCM3_MSG_1012_Data_Desc[] = 
{
	{38,    0,	U_T,		6,	0,	FIELD_OFFSET(RTCM3_MSG_1012_DATA, svid),				FIELD_SIZE(RTCM3_MSG_1012_DATA, svid),				},
	{39,	6,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSG_1012_DATA, L1CodeIndicate),		FIELD_SIZE(RTCM3_MSG_1012_DATA, L1CodeIndicate),		},
	{40,	7,	U_T,		5,	0,	FIELD_OFFSET(RTCM3_MSG_1012_DATA, FreCh),		FIELD_SIZE(RTCM3_MSG_1012_DATA, FreCh),		},
	{41,	12,	U_T,		25, 0,	FIELD_OFFSET(RTCM3_MSG_1012_DATA, L1Pseudorange),		FIELD_SIZE(RTCM3_MSG_1012_DATA, L1Pseudorange), 	},
	{42,	37, I_T,		20, 0,	FIELD_OFFSET(RTCM3_MSG_1012_DATA, L1PhaRangeDifPR), FIELD_SIZE(RTCM3_MSG_1012_DATA, L1PhaRangeDifPR),	},	
	{43,	57, U_T,		7,	0,	FIELD_OFFSET(RTCM3_MSG_1012_DATA, L1LockTimeIdx),		FIELD_SIZE(RTCM3_MSG_1012_DATA, L1LockTimeIdx), 	},	
	{46,	64, B_T,		2,	0,	FIELD_OFFSET(RTCM3_MSG_1012_DATA, L2CodeIndicate),		FIELD_SIZE(RTCM3_MSG_1012_DATA, L2CodeIndicate),		},	
	{47,	66, U_T,		14, 0,	FIELD_OFFSET(RTCM3_MSG_1012_DATA, L2PRDifL1PR), 	FIELD_SIZE(RTCM3_MSG_1012_DATA, L2PRDifL1PR),		},
	{48,	80, I_T,		20, 0,	FIELD_OFFSET(RTCM3_MSG_1012_DATA, L2PhaRangeDifL1PR),	FIELD_SIZE(RTCM3_MSG_1012_DATA, L2PhaRangeDifL1PR), },
	{49,	100,	U_T,		7,	0,	FIELD_OFFSET(RTCM3_MSG_1012_DATA, L2LockTimeIdx),		FIELD_SIZE(RTCM3_MSG_1012_DATA, L2LockTimeIdx), 	},
	
	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},

};

BITITEM_FIELD_DESC RTCM3_MSG_1019_1047_Desc[] =
{
	{9,			12,	U_T,		6,	0,	FIELD_OFFSET(EphemerisBits, svid),		FIELD_SIZE(EphemerisBits, svid),	},
	{76,		18,	U_T,		10,	0,	FIELD_OFFSET(EphemerisBits, week),	FIELD_SIZE(EphemerisBits, week),	},
	{77,		28,	U_T,		4,	0,	FIELD_OFFSET(EphemerisBits, uraidx),	FIELD_SIZE(EphemerisBits, uraidx),	},
	{78,		32,	B_T,		2,	0,	FIELD_OFFSET(EphemerisBits, codeOnL2),	FIELD_SIZE(EphemerisBits, codeOnL2),},
	{79,		34,	I_T,		14,	0,	FIELD_OFFSET(EphemerisBits, i_dot),	FIELD_SIZE(EphemerisBits, i_dot),	},
	{71,		48,	U_T,		8,	0,	FIELD_OFFSET(EphemerisBits, iode),	FIELD_SIZE(EphemerisBits, iode),	},
	{81,		56,	U_T,		16,	0,	FIELD_OFFSET(EphemerisBits, toc),	FIELD_SIZE(EphemerisBits, toc),	},
	{82,		72,	I_T,		8,	0,	FIELD_OFFSET(EphemerisBits, af2),	FIELD_SIZE(EphemerisBits, af2),	},
	{83,		80,	I_T,		16,	0,	FIELD_OFFSET(EphemerisBits, af1),	FIELD_SIZE(EphemerisBits, af1),	},
	{84,		96,	I_T,		22,	0,	FIELD_OFFSET(EphemerisBits, af0),	FIELD_SIZE(EphemerisBits, af0),	},
	{85,		118,	U_T,		10,	0,	FIELD_OFFSET(EphemerisBits, iodc),	FIELD_SIZE(EphemerisBits, iodc),	},
	{86,		128,	I_T,		16,	0,	FIELD_OFFSET(EphemerisBits, Crs),	FIELD_SIZE(EphemerisBits, Crs),	},
	{87,		144,	I_T,		16,	0,	FIELD_OFFSET(EphemerisBits, delta_n),	FIELD_SIZE(EphemerisBits, delta_n),	},
	{88,		160,	I_T,		32,	0,	FIELD_OFFSET(EphemerisBits, M0),	FIELD_SIZE(EphemerisBits, M0),		},
	{89,		192,	I_T,		16,	0,	FIELD_OFFSET(EphemerisBits, Cuc),	FIELD_SIZE(EphemerisBits, Cuc),	},
	{90,		208,	U_T,		32,	0,	FIELD_OFFSET(EphemerisBits, ecc),	FIELD_SIZE(EphemerisBits, ecc),	},
	{91,		240,	I_T,		16,	0,	FIELD_OFFSET(EphemerisBits, Cus),	FIELD_SIZE(EphemerisBits, Cus),	},
	{92,		256,	U_T,		32,	0,	FIELD_OFFSET(EphemerisBits, sqra),	FIELD_SIZE(EphemerisBits, sqra),	},
	{93,		288,	U_T,		16,	0,	FIELD_OFFSET(EphemerisBits, toe),	FIELD_SIZE(EphemerisBits, toe),	},
	{94,		304,	I_T,		16,	0,	FIELD_OFFSET(EphemerisBits, Cic),	FIELD_SIZE(EphemerisBits, Cic),	},
	{95,		320,	I_T,		32,	0,	FIELD_OFFSET(EphemerisBits, omega0),	FIELD_SIZE(EphemerisBits, omega0),	},
	{96,		352,	I_T,		16,	0,	FIELD_OFFSET(EphemerisBits, Cis),	FIELD_SIZE(EphemerisBits, Cis),	},
	{97,		368,	I_T,		32,	0,	FIELD_OFFSET(EphemerisBits, inc0),	FIELD_SIZE(EphemerisBits, inc0),	},
	{98,		400,	I_T,		16,	0,	FIELD_OFFSET(EphemerisBits, Crc),	FIELD_SIZE(EphemerisBits, Crc),	},
	{99,		416,	I_T,		32,	0,	FIELD_OFFSET(EphemerisBits, w),	FIELD_SIZE(EphemerisBits, w),		},
	{100,		448,	I_T,		24,	0,	FIELD_OFFSET(EphemerisBits, omega_dot),	FIELD_SIZE(EphemerisBits, omega_dot),	},
	{101,		472,	I_T,		8,	0,	FIELD_OFFSET(EphemerisBits, Tgd),	FIELD_SIZE(EphemerisBits, Tgd),	},
	{102,		480,	U_T,		6,	0,	FIELD_OFFSET(EphemerisBits, health),	FIELD_SIZE(EphemerisBits, health),	},
	{103,		486,	B_T,		1,	0,	FIELD_OFFSET(EphemerisBits, L2Pflag),	FIELD_SIZE(EphemerisBits, L2Pflag),	},
	{137,		487,	U_T,		1,	0,	FIELD_OFFSET(EphemerisBits, fit_interval),	FIELD_SIZE(EphemerisBits, fit_interval),	},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};



//MSM DF description
BITITEM_FIELD_DESC RTCM3_MSM_Head_Desc[] =
{
	{3,		12,	U_T,		12,	0,	FIELD_OFFSET(RTCM3_MSM_HEAD_INFO, ReferStationID),	FIELD_SIZE(RTCM3_MSM_HEAD_INFO, ReferStationID),	},
	{4,		24,	U_T,		30,	0,	FIELD_OFFSET(RTCM3_MSM_HEAD_INFO, TOW),			FIELD_SIZE(RTCM3_MSM_HEAD_INFO, TOW),			},
	{393,	54,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSM_HEAD_INFO, MultiMsgBit),		FIELD_SIZE(RTCM3_MSM_HEAD_INFO, MultiMsgBit),	},
	{409,	55,	U_T,		3,	0,	FIELD_OFFSET(RTCM3_MSM_HEAD_INFO, IODS),			FIELD_SIZE(RTCM3_MSM_HEAD_INFO, IODS),			},
	{411,	65,	U_T,		2,	0,	FIELD_OFFSET(RTCM3_MSM_HEAD_INFO, ClkSteer_Indicate),	FIELD_SIZE(RTCM3_MSM_HEAD_INFO, ClkSteer_Indicate),	},
	{412,	67,	U_T,		2,	0,	FIELD_OFFSET(RTCM3_MSM_HEAD_INFO, ExClk_Indicate),		FIELD_SIZE(RTCM3_MSM_HEAD_INFO, ExClk_Indicate),		},
	{417,	69,	B_T,		1,	0,	FIELD_OFFSET(RTCM3_MSM_HEAD_INFO, DiverFreeSmooth),	FIELD_SIZE(RTCM3_MSM_HEAD_INFO, DiverFreeSmooth),	},
	{418,	70,	B_T,		3,	0,	FIELD_OFFSET(RTCM3_MSM_HEAD_INFO, SmoothInterval),		FIELD_SIZE(RTCM3_MSM_HEAD_INFO, SmoothInterval),		},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

BITITEM_FIELD_DESC RTCM3_MSM_MAP_Desc[] =
{
	{394,	0,	B_T,		64,	0,	FIELD_OFFSET(RTCM3_MSM_MAP, SatMask),		FIELD_SIZE(RTCM3_MSM_MAP, SatMask),	},
	{395,	64,	B_T,		32,	0,	FIELD_OFFSET(RTCM3_MSM_MAP, SigMask),		FIELD_SIZE(RTCM3_MSM_MAP, SigMask),	},
	{396,	96,	B_T,		64,	0,	FIELD_OFFSET(RTCM3_MSM_MAP, CellMask),		FIELD_SIZE(RTCM3_MSM_MAP, CellMask),},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};


// MSM1,2,3 satellite data description
BITITEM_FIELD_DESC RTCM3_MSM1_2_3_Sat_Desc[] =
{
	{398,	0,	U_T,		10,	MSM_SAT_MAPIDX_RouPR,	FIELD_OFFSET(RTCM3_MSM_SAT_INFO, RoughRange),		FIELD_SIZE(RTCM3_MSM_SAT_INFO, RoughRange),	},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

// MSM4,6 satellite data description
BITITEM_FIELD_DESC RTCM3_MSM4_6_Sat_Desc[] =
{
	{397,	0,	U_T,		8,	MSM_SAT_MAPIDX_RouPRInter,	FIELD_OFFSET(RTCM3_MSM_SAT_INFO, RoughRangeInter),	FIELD_SIZE(RTCM3_MSM_SAT_INFO, RoughRangeInter),},
	{398,	8,	U_T,		10,	MSM_SAT_MAPIDX_RouPR,		FIELD_OFFSET(RTCM3_MSM_SAT_INFO, RoughRange),		FIELD_SIZE(RTCM3_MSM_SAT_INFO, RoughRange),	},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

// MSM5,7 satellite data description
BITITEM_FIELD_DESC RTCM3_MSM5_7_Sat_Desc[] =
{
	{397,	0,	U_T,		8,	MSM_SAT_MAPIDX_RouPRInter,	FIELD_OFFSET(RTCM3_MSM_SAT_INFO, RoughRangeInter),	FIELD_SIZE(RTCM3_MSM_SAT_INFO, RoughRangeInter),	},
	{398,	8,	U_T,		4,	MSM_SAT_MAPIDX_ExSatInfo,	FIELD_OFFSET(RTCM3_MSM_SAT_INFO, ExSatInfo),		FIELD_SIZE(RTCM3_MSM_SAT_INFO, ExSatInfo),			},
	{398,	12,	U_T,		10,	MSM_SAT_MAPIDX_RouPR,		FIELD_OFFSET(RTCM3_MSM_SAT_INFO, RoughRange),		FIELD_SIZE(RTCM3_MSM_SAT_INFO, RoughRange),		},
	{399,	14,	I_T,		14,	MSM_SAT_MAPIDX_RouPhaPRate,FIELD_OFFSET(RTCM3_MSM_SAT_INFO, RoughPhasePRRate),	FIELD_SIZE(RTCM3_MSM_SAT_INFO, RoughPhasePRRate),	},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

// MSM1 signal data description
BITITEM_FIELD_DESC RTCM3_MSM1_Sig_Desc[] =
{
	{400,	0,	I_T,		15,	MSM_SIG_MAPIDX_FineRange,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FineRange),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, FineRange),	},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

// MSM2 signal data description
BITITEM_FIELD_DESC RTCM3_MSM2_Sig_Desc[] =
{
	{401,	0,	I_T,		22,	MSM_SIG_MAPIDX_FineRange,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FinePhasePR),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, FinePhasePR),	},
	{402,	22,	U_T,		4,	MSM_SIG_MAPIDX_LockTimeIdx,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, LockTimeIdx),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, LockTimeIdx),	},
	{420,	26,	U_T,		1,	MSM_SIG_MAPIDX_HalfCycleIndicate, FIELD_OFFSET(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),	},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

// MSM3 signal data description
BITITEM_FIELD_DESC RTCM3_MSM3_Sig_Desc[] =
{
	{400,	0,	I_T,		15,	MSM_SIG_MAPIDX_FineRange,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FineRange),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, FineRange),	},
	{401,	15,	I_T,		22,	MSM_SIG_MAPIDX_FinePhasePR,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FinePhasePR),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, FinePhasePR),		},
	{402,	37,	U_T,		4,	MSM_SIG_MAPIDX_LockTimeIdx,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, LockTimeIdx),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, LockTimeIdx),	},
	{420,	41,	B_T,		1,	MSM_SIG_MAPIDX_HalfCycleIndicate, FIELD_OFFSET(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),	},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

// MSM4 signal data description
BITITEM_FIELD_DESC RTCM3_MSM4_Sig_Desc[] =
{
	{400,	0,	I_T,		15,	MSM_SIG_MAPIDX_FineRange,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FineRange),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, FineRange),	},
	{401,	15,	I_T,		22,	MSM_SIG_MAPIDX_FinePhasePR,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FinePhasePR),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, FinePhasePR),		},
	{402,	37,	U_T,		4,	MSM_SIG_MAPIDX_LockTimeIdx,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, LockTimeIdx),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, LockTimeIdx),	},
	{420,	41,	B_T,		1,	MSM_SIG_MAPIDX_HalfCycleIndicate,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),	},
	{403,	42,	U_T,		6,	MSM_SIG_MAPIDX_CNR,		FIELD_OFFSET(RTCM3_MSM_SIG_INFO, CNR),				FIELD_SIZE(RTCM3_MSM_SIG_INFO, CNR),		},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

// MSM5 signal data description
BITITEM_FIELD_DESC RTCM3_MSM5_Sig_Desc[] =
{
	{400,	0,	I_T,		15,	MSM_SIG_MAPIDX_FineRange,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FineRange),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, FineRange),	},
	{401,	15,	I_T,		22,	MSM_SIG_MAPIDX_FinePhasePR,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FinePhasePR),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, FinePhasePR),		},
	{402,	37,	U_T,		4,	MSM_SIG_MAPIDX_LockTimeIdx,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, LockTimeIdx),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, LockTimeIdx),	},
	{420,	41,	B_T,		1,	MSM_SIG_MAPIDX_HalfCycleIndicate,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),	},
	{403,	42,	U_T,		6,	MSM_SIG_MAPIDX_CNR,			FIELD_OFFSET(RTCM3_MSM_SIG_INFO, CNR),				FIELD_SIZE(RTCM3_MSM_SIG_INFO, CNR),	},
	{404,	48,	I_T,		15,	MSM_SIG_MAPIDX_FinePhasePRate,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FinePhasePRate),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, FinePhasePRate),	},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

// MSM6 signal data description
BITITEM_FIELD_DESC RTCM3_MSM6_Sig_Desc[] =
{
	{405,	0,	I_T,		20,	MSM_SIG_MAPIDX_FineRangeEx,		FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FineRangeEx),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, FineRangeEx),			},
	{406,	20,	I_T,		24,	MSM_SIG_MAPIDX_FinePhasePREx,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FinePhasePREx),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, FinePhasePREx),	},
	{407,	44,	U_T,		10,	MSM_SIG_MAPIDX_LockTimeIdxEx,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, LockTimeIdxEx),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, LockTimeIdxEx),	},
	{420,	54,	B_T,		1,	MSM_SIG_MAPIDX_HalfCycleIndicate,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),		},
	{408,	55,	U_T,		10,	MSM_SIG_MAPIDX_CNREx,			FIELD_OFFSET(RTCM3_MSM_SIG_INFO, CNREx),				FIELD_SIZE(RTCM3_MSM_SIG_INFO, CNREx),			},
	
	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};

// MSM7 signal data description
BITITEM_FIELD_DESC RTCM3_MSM7_Sig_Desc[] =
{
	{405,	0,	I_T,		20,	MSM_SIG_MAPIDX_FineRangeEx,		FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FineRangeEx),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, FineRangeEx),			},
	{406,	20,	I_T,		24,	MSM_SIG_MAPIDX_FinePhasePREx,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FinePhasePREx),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, FinePhasePREx),	},
	{407,	44,	U_T,		10,	MSM_SIG_MAPIDX_LockTimeIdxEx,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, LockTimeIdxEx),		FIELD_SIZE(RTCM3_MSM_SIG_INFO, LockTimeIdxEx),	},
	{420,	54,	B_T,		1,	MSM_SIG_MAPIDX_HalfCycleIndicate,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, HalfCycleIndicate),		},
	{408,	55,	U_T,		10,	MSM_SIG_MAPIDX_CNREx,			FIELD_OFFSET(RTCM3_MSM_SIG_INFO, CNREx),				FIELD_SIZE(RTCM3_MSM_SIG_INFO, CNREx),			},
	{404,	65,	I_T,		15,	MSM_SIG_MAPIDX_FinePhasePRate,	FIELD_OFFSET(RTCM3_MSM_SIG_INFO, FinePhasePRate),	FIELD_SIZE(RTCM3_MSM_SIG_INFO, FinePhasePRate),		},

	{BITITEM_DF_ID_INVALID, 0,0,0,0,0,0},
};


/*************************RTCM3.x message table***************************/
RTCM3_FIXLEN_MSG_DESC RTCM3_FIXLEN_MSG_Desc_TBL[] = 
{
	{1005,	RTCM3_MSG_1005_Desc},
	{1006,	RTCM3_MSG_1006_Desc},
	{1019,	RTCM3_MSG_1019_1047_Desc},
	{1047,	RTCM3_MSG_1019_1047_Desc},

	{ RTCM3_MSG_ID_INVALID, 	NULL},
};

RTCM3_VARLEN_MSG_DESC RTCM3_VARLEN_MSG_Desc_TBL[] = 
{
	{1001,	RTCM3_MSG_1001_1004_Head_Desc,	RTCM3_MSG_1001_Data_Desc,	sizeof(RTCM3_MSG_1001_DATA)},
	{1002,	RTCM3_MSG_1001_1004_Head_Desc,	RTCM3_MSG_1002_Data_Desc,	sizeof(RTCM3_MSG_1002_DATA)},
	{1003,	RTCM3_MSG_1001_1004_Head_Desc,	RTCM3_MSG_1003_Data_Desc,	sizeof(RTCM3_MSG_1003_DATA)},
	{1004,	RTCM3_MSG_1001_1004_Head_Desc,	RTCM3_MSG_1004_Data_Desc,	sizeof(RTCM3_MSG_1004_DATA)},
	{1012,	RTCM3_MSG_1009_1012_Head_Desc,	RTCM3_MSG_1012_Data_Desc,	sizeof(RTCM3_MSG_1012_DATA)},

	{ RTCM3_MSG_ID_INVALID, 	NULL,	NULL,	0},
};


RTCM3_MSM_MSG_DESC RTCM3_MSM_MSG_Desc_TBL[] = 
{
	{1,	RTCM3_MSM_Head_Desc,	RTCM3_MSM_MAP_Desc, RTCM3_MSM1_2_3_Sat_Desc,	RTCM3_MSM1_Sig_Desc},
	{2,	RTCM3_MSM_Head_Desc,	RTCM3_MSM_MAP_Desc,	RTCM3_MSM1_2_3_Sat_Desc,	RTCM3_MSM2_Sig_Desc},
	{3,	RTCM3_MSM_Head_Desc,	RTCM3_MSM_MAP_Desc,	RTCM3_MSM1_2_3_Sat_Desc,	RTCM3_MSM3_Sig_Desc},
	{4,	RTCM3_MSM_Head_Desc,	RTCM3_MSM_MAP_Desc,	RTCM3_MSM4_6_Sat_Desc,	RTCM3_MSM4_Sig_Desc},
	{5,	RTCM3_MSM_Head_Desc,	RTCM3_MSM_MAP_Desc,	RTCM3_MSM5_7_Sat_Desc,	RTCM3_MSM5_Sig_Desc},
	{6,	RTCM3_MSM_Head_Desc,	RTCM3_MSM_MAP_Desc,	RTCM3_MSM4_6_Sat_Desc,	RTCM3_MSM6_Sig_Desc},
	{7,	RTCM3_MSM_Head_Desc,	RTCM3_MSM_MAP_Desc,	RTCM3_MSM5_7_Sat_Desc,	RTCM3_MSM7_Sig_Desc},
	
	{ RTCM3_MSM_ID_INVALID, 	NULL,	NULL,	0},
};


/**********************RTCM3.x table process****************************/

void FindMSMType(word16 msg_id, byte* p_msm_type, byte* p_navsys)
{
	if((msg_id >= 1071) && (msg_id<=1077))	//gps MSM
	{
		(*p_msm_type) = msg_id - 1070;
		(*p_navsys) = NAV_SYS_GPS;

	}
	else if((msg_id >= 1081) && (msg_id<=1087))	//glo MSM
	{
		(*p_msm_type) = msg_id - 1080;
		(*p_navsys) = NAV_SYS_GLO;

	}
	else if((msg_id >= 1121) && (msg_id<=1127))	//BD MSM	
	{
		(*p_msm_type) = msg_id - 1120;
		(*p_navsys) = NAV_SYS_BD;
	}

	return;
}

word16 FindMSMMsgID(byte msm_type, byte navsys)
{
	word16 msgid=0;
	if(navsys == NAV_SYS_GPS)
		msgid = 1070 + msm_type;
	else if(navsys == NAV_SYS_BD)
		msgid = 1120 + msm_type;
	else if(navsys == NAV_SYS_GLO)
		msgid = 1080 + msm_type;

	return msgid;
}


boolean FindRTCM3MsgEntry(word16 msg_id, RTCM3_msg_entry** pEntry)
{
	boolean ret = FALSE;
	(*pEntry) = RTCM3_MSG_TBL;
	
	while((*pEntry)->msg_id != RTCM3_MSG_ID_INVALID)
	{
		if((*pEntry)->msg_id == msg_id)
		{
			ret = TRUE;
			break;
		}

		(*pEntry)++;
	}

	return ret;
}


boolean FindRTCM3FixLenMsgDesc(word16 msg_id, BITITEM_FIELD_DESC** pDesc)
{
	boolean ret= FALSE;
	int32 idx=0;
	while(RTCM3_FIXLEN_MSG_Desc_TBL[idx].msg_id != RTCM3_MSG_ID_INVALID)
	{
		if(RTCM3_FIXLEN_MSG_Desc_TBL[idx].msg_id == msg_id)
		{
			*pDesc =  RTCM3_FIXLEN_MSG_Desc_TBL[idx].pFieldDesc;

			ret = TRUE;
			break;
		}
		else
			idx++;
	}

	return ret;
}


boolean FindRTCM3VarLenMsgDesc(word16 msg_id, BITITEM_FIELD_DESC** pHeadDesc, BITITEM_FIELD_DESC** pDataDesc, word16* pRepeatStructLen)
{
	boolean ret= FALSE;
	int32 idx=0;
	while(RTCM3_VARLEN_MSG_Desc_TBL[idx].msg_id != RTCM3_MSG_ID_INVALID)
	{
		if(RTCM3_VARLEN_MSG_Desc_TBL[idx].msg_id == msg_id)
		{
			*pHeadDesc = RTCM3_VARLEN_MSG_Desc_TBL[idx].pMsgHeadDesc;
			*pDataDesc = RTCM3_VARLEN_MSG_Desc_TBL[idx].pMsgDataDesc;
			*pRepeatStructLen = RTCM3_VARLEN_MSG_Desc_TBL[idx].repeatStructLen;

			ret = TRUE;
			break;
		}
		else
			idx++;
	}

	return ret;
}

boolean FindRTCM3MSMMsgDesc(byte msm_id, BITITEM_FIELD_DESC** pHeadDesc, BITITEM_FIELD_DESC** pMapDesc, BITITEM_FIELD_DESC** pSatDataDesc, BITITEM_FIELD_DESC** pSigDataDesc)
{
	boolean ret= FALSE;
	int32 idx=0;
	while(RTCM3_MSM_MSG_Desc_TBL[idx].msm_id != RTCM3_MSM_ID_INVALID)
	{
		if(RTCM3_MSM_MSG_Desc_TBL[idx].msm_id == msm_id)
		{
			*pHeadDesc = RTCM3_MSM_MSG_Desc_TBL[idx].pHeadDesc;
			*pMapDesc = RTCM3_MSM_MSG_Desc_TBL[idx].pMapDesc;
			*pSatDataDesc = RTCM3_MSM_MSG_Desc_TBL[idx].pSatDataDesc;
			*pSigDataDesc = RTCM3_MSM_MSG_Desc_TBL[idx].pSigDataDesc;

			ret = TRUE;
			break;
		}
		else
			idx++;
	}

	return ret;
}




