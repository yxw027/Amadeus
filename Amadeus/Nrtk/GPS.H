/*
���ܺ���;�����е�GNSS��صĳ������ڴ�ͷ�ļ�(.h)������
������κκ�����

���ߣ�Ϳ��  2011.11.17
*/

#ifndef GPS_H        //��ֹͷ�ļ����ظ�����
#define GPS_H

//ͷ�ļ�
//#include <iostream>
//#include <fstream>
//#include <cmath>
//#include <string.h>
//#include <stdlib.h>
//#include <conio.h>
//#include <stdio.h>
//#include <iomanip>  //���Ʊ�׼���
//#include "ctype.h"    //�ַ�����
#include "rtklib.h"

#define I96 348//116       //epochs of sp3file����ǰһ�����10����Ԫ�ͺ�һ�����10����Ԫ(96+20)5min�Ĳ�Ʒ
#define I32 32        //Number of GPS Satellite����14������
//
#define MaxSat_Epoch 30*2     //Numb of GNSS Satellite in a epoch
//#define MAXSAT 60             //���������Ŀ �����Ӳ��ļ�ͳ��
//Constant
#define C  2.99792458e8 /* light velocity */
//GPS
#define FL1 1575.42e6   /* Hz */
#define FL2 1227.60e6   /* Hz */
#define Lambda1 (C/FL1) /* m */
#define Lambda2 (C/FL2) /* m */
#define MaxGPS 50       /* Max value of GPS satellite */
//GLONASS
#define RFL1 1602.00e6  /* Hz */
#define RFL2 1246.00e6  /* Hz */
#define RdetFL1 0.5625E6/* Hz */
#define RdetFL2 0.4375E6/* Hz */
#define MaxGLONASS 50   /* Max value of GLONASS satellite */
//COMPASS
#define CFL1 1561.098e6 /* Hz */
#define CFL2 1207.140e6 /* Hz */
#define CLambda1 (C/CFL1) /* m */
#define CLambda2 (C/CFL2) /* m */
#define MaxCOMPASS 50   /* Max value of COMPASS satellite */

#endif