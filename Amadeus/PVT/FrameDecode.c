#include "FrameDecode.h"
//#ifndef _POSTPROC
//#ifndef _SIMULATE
//#include "BD2SfParse.h"
//#include "GPSSfParse.h"
//#include "GloSfParse.h"
//#endif
//#endif

AlmEphUTCInfo Sys3AlmEphUTCInfo;

#ifndef _POSTPROC
#ifndef _SIMULATE
void TaskFrameDecode()
{
	int32 trkid = 0, svid = 0;

	for(trkid=0; trkid<MAXCHANNELS; trkid++)
	{
		svid = EphFrameBuffer[trkid].Sv;
		if(SV_IsGps(svid))	
			ProcessGpsEphParse(trkid);	
		else if (SV_IsBd2Meo(svid))
			ProcessBd2MeoEphParse(trkid);
		else if(SV_IsBd2Geo(svid))
			ProcessBd2GeoEphParse(trkid);
#if SUPPORT_GLONASS
		else if (SV_IsGlo(svid))
			ProcessGloEphParse(trkid);
#endif
		svid = AlmFrameBuffer[trkid].Sv;
		if(SV_IsGps(svid))	
			ProcessGpsAlmParse(trkid);	
		else if (SV_IsBd2Meo(svid)||SV_IsBd2Geo(svid))
			ProcessBd2AlmParse(trkid);
#if SUPPORT_GLONASS
		else if (SV_IsGlo(svid))
			ProcessGloAlmParse(trkid);
#endif
	}

}
#endif
#endif
void InitFrameDecode()
{
	memset(&Sys3AlmEphUTCInfo, 0, sizeof(Sys3AlmEphUTCInfo));
}


int sbit(unsigned int val, int nbits)
{
	int result = 0x0;
	result = (val << (32 - nbits));
	result = result >> (32 - nbits);
	return(result);
}

