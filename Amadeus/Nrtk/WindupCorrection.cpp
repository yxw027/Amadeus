#include "../stdafx.h"
#include "math.h"
#include "GPSmodel.h"
#include "GPS.H"
#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */


//
extern int normv3(const double *a, double *b)
{
	double r;
	if ((r = norm(a, 3)) <= 0.0) return 0;
	b[0] = a[0] / r;
	b[1] = a[1] / r;
	b[2] = a[2] / r;
	return 1;
}
extern double dot(const double *a, const double *b, int n)
{
	double c = 0.0;

	while (--n >= 0) c += a[n] * b[n];
	return c;
}
extern double norm(const double *a, int n)
{
	return sqrt(dot(a, a, n));
}
extern void cross3(const double *a, const double *b, double *c)
{
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}
extern void ecef2pos(const double *r, double *pos)
{
	double e2 = FE_WGS84*(2.0 - FE_WGS84), r2 = dot(r, r, 2), z, zk, v = RE_WGS84, sinp;

	for (z = r[2], zk = 0.0; fabs(z - zk) >= 1E-4;) {
		zk = z;
		sinp = z / sqrt(r2 + z*z);
		v = RE_WGS84 / sqrt(1.0 - e2*sinp*sinp);
		z = r[2] + v*e2*sinp;
	}
	pos[0] = r2>1E-12 ? atan(z / sqrt(r2)) : (r[2]>0.0 ? PI / 2.0 : -PI / 2.0);
	pos[1] = r2>1E-12 ? atan2(r[1], r[0]) : 0.0;
	pos[2] = sqrt(r2 + z*z) - v;
}
extern void xyz2enu(const double *pos, double *E)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
	E[0] = -sinl;      E[3] = cosl;       E[6] = 0.0;
	E[1] = -sinp*cosl; E[4] = -sinp*sinl; E[7] = cosp;
	E[2] = cosp*cosl;  E[5] = cosp*sinl;  E[8] = sinp;
}