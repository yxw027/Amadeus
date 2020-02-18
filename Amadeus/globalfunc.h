#pragma once

struct _shwdpos {
	double	delx[5]; // 0=[real] 1=[hour] 2=[day] 3=[any] 4=[flt]
	double	dely[5]; // 0=[real] 1=[hour] 2=[day] 3=[any] 4=[flt]
	double	delz[5]; // 0=[real] 1=[hour] 2=[day] 3=[any] 4=[flt]
	double	delr[5]; // 0=[real] 1=[hour] 2=[day] 3=[any] 4=[flt]
};

struct _showspos {
	short	sys;	// gnss system
	short	slntype;	// solution type 1=realtime, 2=hourly, 3=daily
	int	svn;	// validity satellite number
	_shwdpos	delpos;	// delx y z r of position
};

void UpdateNetList(void);
	