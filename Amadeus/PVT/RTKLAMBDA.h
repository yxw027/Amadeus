/*******************************************************************
Company:   hwacreate
Engineer:  
Create Date: 2015.12.01
File  Name:  RTKLAMBDA.h
Description: header file of RTKLAMBDA.c
Function List: 
version: V1.0
Revision Date:
Modifier: 
Additional Comments: 
********************************************************************/
#ifndef RTKLAMBDA_H_
#define RTKLAMBDA_H_


//#include "..\OSInterface\OSInterface.h"
#include "define.h"
#include "typedefine.h"
#include "RTKConstants.h"


//////////////////////////////////////////////////////////////////////////
// LAMBDA算法中各常量宏定义
#define LAMBDA_CHI2_FACTOR				(1.5)
#define LAMBDA_EPSILON					(1E-20)
#define LAMBDA_ERR_Q_NOT_SYMMETRY		(-1)
#define LAMBDA_ERR_NO_ENOUGH_CANDIDATE	(-2)
#define LAMBDA_PI						(3.1415926535898)
#define LAMBDA_N_DIM					(20)
#define LAMBDA_N_CANDS					(2)
#define LAMBDA_N_RATIO					(2)
#define LAMBDA_SAT_BREAK_TIME			(120)
#define LAMBDA_RANGE_DIFF				(100)		
//													//航空天线参数	测量型天线参数			
#define RANGE_VAR						(0.25)		//0.6^2				0.5^2
#define PHASE_VAR						(0.0001)	//0.02^2				0.01^2
//#define RANGE_VAR						(0.36)		//0.6^2				0.5^2
//#define PHASE_VAR						(0.04)	//0.02^2				0.01^2

#define CORVARIANCE_NUM					(1E12)
#define LAMBDA_N_MAX_SATNM				(36)
#define LOOPMAX     10000           /* maximum count of search loop */
#define LAMBDA_N_LOOP     				(15)
#define LAMBDA_N_SWITCH_ERR				(0.4)
#define LAMBDA_RTK_RESIDUAL(d)			(15E-3+1e-6*d)		// d单位为米
#define LAMBDA_BSL_ERR					(3E-2)
#define LAMBDA_PITCH_MAX				(20.0)




double SGN(double x);
double ROUND(double x);
void SWAP(double* x,double* y);



//  **************************************************************
//  目的:	MLAMBDA算法实现
//  -------------------------------------------------------------
//  输入:	
//		a:		浮点解
//		Q:		协方差矩阵
//		nDim:	模糊度个数
//		ncands:	备选固定解个数
//		factor:	方差因子
//  输出:	
//		afixed:	固定解
//		sqnorm:	范数
//		Qahat:	Z变换后的方差矩阵
//		Z:		整数变换
//  返回:	求解成功与否
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
int M_LAMBDA(double *a, double *Q, int nDim, int ncands, double *afixed, double *sqnorm, double *Qahat, double *Z);

/* integer gauss transformation ----------------------------------------------*/
void M_LAMBDA_gauss(int n, double *L, double *Z, int i, int j);

/* permutations --------------------------------------------------------------*/
void M_LAMBDA_perm(int n, double *L, double *D, int j, double del, double *Z);

/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L)  ---------------*/
void M_LAMBDA_reduction(int n, double *L, double *D, double *Z);

/* modified lambda (mlambda) search  -------------------------------*/
bool M_LAMBDA_search(int n, int m, const double *L, const double *D,
                  const double *zs, double *zn, double *s);

//////////////////////////////////////////////////////////////////////////
//***********   LAMBDA算法中所用矩阵操作函数     **************
//////////////////////////////////////////////////////////////////////////


//  **************************************************************
//  目的:	判定方阵是否为对称矩阵
//  -------------------------------------------------------------
//  输入:	double M[]	矩阵名称
//			int nDim	矩阵维数
//			double epsilon	一个很小的数值
//  输出:	无
//  返回:	true为是对称的,FALSE表示矩阵不对称
//  说明:	不用严格相等的方法,而是用||Q-Q'||<1E-10来判断
//  -------------------------------------------------------------
// ***************************************************************
//bool LAMBDA_MO_IsSymmetry2(double M[], int nDim, double epsilon);

//  **************************************************************
//  目的:	对称矩阵的分解 P(n*n) = L(n*n) * D(n*n) * L'(n*n)
//  -------------------------------------------------------------
//  输入:	double P[]	对称矩阵
//			int nDim	矩阵维数
//  输出:	double L[]	下三角矩阵
//			double D[]	对角矩阵
//  返回:	true	成功
//			FALSE	失败
//  说明:	原矩阵必须为对称矩阵,分析的结果为L是一下三角矩阵(对角线上
//			元素为1),而D为对角矩阵.所有矩阵维数均为n*n
//  -------------------------------------------------------------
// ***************************************************************
//bool LAMBDA_MO_LTDL(double P[], double L[], double D[], int nDim);

//  **************************************************************
//  目的:	对称矩阵的分解 P(n*n) = L(n*n) * D(n*n) * L'(n*n)
//  -------------------------------------------------------------
//  输入:	double P[]	对称矩阵
//			int nDim	矩阵维数
//  输出:	double L[]	下三角矩阵
//			double D[]	对角矩阵
//  返回:	true	成功
//			FALSE	失败
//  说明:	原矩阵必须为对称矩阵,分析的结果为L是一下三角矩阵(对角线上
//			元素为1),而D为n*1行向量.其它所有矩阵维数均为n*n
//  -------------------------------------------------------------
// ***************************************************************
bool LAMBDA_MO_LTDL2(double P[], double L[], double D[], int nDim);

//  **************************************************************
//  目的:	求最接近浮点数的整数
//  -------------------------------------------------------------
//  输入:	
//			double* dValue
//			int nRow
//			int nCol
//  输出:	
//			double* nValue
//  返回:	
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
int LAMBDA_MO_Round(double* dValue, int nRow, int nCol, double* nValue);



//  **************************************************************
//  目的:	将矩阵转置
//  -------------------------------------------------------------
//  输入:	double M[]	矩阵名称
//			int nRow	行数
//			int nCol	列数
//  输出:	double M_Transp[]	转置后的矩阵
//  返回:	无
//  说明:	将矩阵转置后放入另一矩阵中,原矩阵不变
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixTransp(double M[], double M_Transp[],int nRow, int nCol);



//  **************************************************************
//  目的:	根据LU分解求解矩阵的逆(保留原矩阵)
//  -------------------------------------------------------------
//  输入:	double M[]		原矩阵
//			int nDim		矩阵维数
//  输出:	double M_Inv[]	逆矩阵
//  返回:	true	成功
//			FALSE	失败
//  说明:	数值稳定性好,结果存放在新的矩阵M_Inv中
//  -------------------------------------------------------------
// ***************************************************************
bool LAMBDA_MO_MatrixInv_LU(double M[], double M_Inv[], int nDim);



//  **************************************************************
//  目的:	矩阵相乘	N(r*r) = AT*P*A, 其中, A(n*r), P(n*n)
//  -------------------------------------------------------------
//  输入:	double A[], double P[]
//			int RowA, int ColA
//  输出:	double ATPA[]
//  返回:	无
//  说明:	无
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixATPA(double A[], double P[], double ATPA[], int RowA, int ColA);


//  **************************************************************
//  目的:	一个矩阵乘以另一矩阵的转置阵 C(m*k) = A(m*n)*B'(k*n)
//  -------------------------------------------------------------
//  输入:	double A[]	double B[]				相乘矩阵
//			int RowA	int ColA	 int RowB	各矩阵行列数
//  输出:	double ABT[]	乘法计算结果
//  返回:	无
//  说明:	无
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixABT(double A[], double B[], double ABT[],
						 int RowA, int ColA, int RowB);



//  **************************************************************
//  目的:	矩阵相乘 C(m*s) = A(m*n) * B(n*s)
//  -------------------------------------------------------------
//  输入:	double A[]
//			double B[]	相乘的矩阵
//			int nRowA	A的行数
//			int nColA	A的列数
//			int nColB	B的列数
//  输出:	double C[]	相乘结果
//  返回:	无
//  说明:	C(m*s) = A(m*n) * B(n*s)
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixMulti(double A[],double B[],double C[], int nRowA, int nColA, int nColB);


//  **************************************************************
//  目的:	一个矩阵的转置乘以另一矩阵   C(m*k) = A'(n*m)*B(n*k)
//  -------------------------------------------------------------
//  输入:	double A[], double B[]			原矩阵 
//			int RowA, int ColA, int ColB	各矩阵行列数
//  输出:	double ATB[]	相乘结果
//  返回:	无
//  说明:	无
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixATB(double A[], double B[], double ATB[], 
				  int RowA, int ColA, int ColB);


//  **************************************************************
//  目的:	矩阵相减 C = A - B
//  -------------------------------------------------------------
//  输入:	double A[]
//			double B[]	相减的矩阵
//			int nRow
//			int nCol	行列数
//  输出:	double C[]	相减结果
//  返回:	无
//  说明:	A,B,C三矩阵维数要相同
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixSub(double A[], double B[], double C[], int nRow, int nCol);


//  **************************************************************
//  目的:	  对数值进行排序(最简单的选择排序法)
//  -------------------------------------------------------------
//  输入:	
//			double M[]	原数据
//			int nLen	数据长度
//  输出:	
//			double M_Sort[]	排序好的数据
//  返回:	
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
//int LAMBDA_MO_Sort(double M[], double M_Sort[], int nIndex[], int nLen);

//  **************************************************************
//  目的:	  计算一个正数的gamma函数值
//  -------------------------------------------------------------
//  输入:	double x
//  输出:	
//  返回:	gamma(x)
//  说明:	gamma(x) = integral from 0 to inf of t^(x-1) exp(-t) dt.
//  -------------------------------------------------------------
// ***************************************************************
//double LAMBDA_MO_GammaFunc(double x);


//  **************************************************************
//  目的:	设矩阵的列向量
//  -------------------------------------------------------------
//  输入:	double M[]	矩阵名称
//			int nRow	行数
//			int nCol	列数
//			double Rv[]	列向量名
//			int k		所设列向量的列数
//  输出: 	无
//  返回:	无
//  说明:	设M矩阵的第k列向量为Rv, k=1:nCol,程序中自动减1
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_SetCol(double M[], double Cv[], int nRow, int nCol, int k);
//void LAMBDA_MO_SetRow(double M[], double Cv[], int nRow, int nCol, int k);

//  **************************************************************
//  目的:	求解M*x=b，其中b在返回时被x覆盖
//  -------------------------------------------------------------
//  输入:	double M[]		矩阵名称
//			double Inx[]	LU分解后的置换
//			double b[]		向量
//			int nDim		维数
//  输出:	double b[]		求得的解x
//  返回:	无
//  说明:	此函数与LU_Decom配合使用,亦可用来解线性方程组
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_LU_BackSub (double M[], double Inx[], double b[], int nDim);


//  **************************************************************
//  目的:	  交换两个向量的值
//  -------------------------------------------------------------
//  输入:	
//			double* V1	向量一
//			double* V2	向量二
//			int nLen	向量长度
//  输出:	
//  返回:	
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
//int LAMBDA_MO_SwapVector(double* V1, double* V2, int nLen);


//  **************************************************************
//  目的:	  找出向量中的最大值,并返回其位置
//  -------------------------------------------------------------
//  输入:	
//			double* V	向量值
//			int nLen	向量长度
//  输出:	
//			double* MaxValue 最大值
//  返回:	最大值位置
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
//int LAMBDA_MO_GetMax(double* V, int nLen, double* MaxValue);


//  **************************************************************
//  目的:	求矩阵的LU分解，P*M = L*U
//  -------------------------------------------------------------
//  输入:	double M[]	原矩阵
//			int nDim	矩阵维数
//  输出:	double M[]		分解后被LU矩阵覆盖
//			double Inx[]	矩阵M的一个置换,在LU_BackSub时有用
//  返回:	true	成功
//			FALSE	失败
//  说明:	P*M = L*U，其中M矩阵被LU矩阵覆盖，Inx是矩阵M的一个置换
//			LU分解是选主元的，带有置换，因此数值稳定性好
//  -------------------------------------------------------------
// ***************************************************************
bool LAMBDA_MO_LU_Decom(double M[], double Inx[], int nDim);

//  **************************************************************
//  目的:	  将行数相同的两个矩阵左右拼接 P = [A B]
//  -------------------------------------------------------------
//  输入:	
//			double *A
//			double *B
//			int nRow
//			int nColA
//			int nColB
//  输出:	
//			double *P
//  返回:	
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
//int LAMBDA_MO_MatrixUnion_LR(double *A, double *B, double *P, int nRow, int nColA, int nColB);

//  **************************************************************
//  目的:	  将行数相同的上下矩阵左右拼接 P = [A ;B]
//  -------------------------------------------------------------
//  输入:	
//			double *A
//			double *B
//			int nRow
//			int nColA
//			int nColB
//  输出:	
//			double *P
//  返回:	
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
//int LAMBDA_MO_MatrixUnion_UD(double *A, double *B, double *P, int nRowA, int nRowB, int nCol);



//  **************************************************************
//  目的:	  根据向量或者矩阵的第一列来排序行
//  -------------------------------------------------------------
//  输入:	
//			double* M	原矩阵
//			int nRow	矩阵行数
//			int nCol	矩阵列数
//  输出:	
//			double *M_Sort	排好序的矩阵
//  返回:	
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
//int LAMBDA_MO_SortRows(double* M, double *M_Sort, int nRow, int nCol);



//  **************************************************************
//  目的:	  拆分矩阵(取大矩阵中的一部分)
//  -------------------------------------------------------------
//  输入:	
//			double *M
//			int nRowM
//			int nColM
//			int nKeyRow		取值范围: (1:nRowM),程序中会在下标减1
//			int nKeyCol		取值范围: (1:nColM),程序中会在下标减1
//			int nRowS
//			int nColS
//  输出:	
//			double *S
//  返回:	
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
//int LAMBDA_MO_MatrixSplit(double *M, double *S, int nRowM, int nColM, int nKeyRow, int nKeyCol, int nRowS, int nColS);


//  **************************************************************
//  目的:	  保存矩阵对称
//  -------------------------------------------------------------
//  输入:	
//			double *M
//			int nRowM
//			int nColM
//  输出:	
//			double *M
//  返回:	
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
int LAMBDA_MO_MatrixMakeSymmetry(double *M, int nDim);



//  **************************************************************
//  目的:	矩阵M赋值给P,即	P = M
//  -------------------------------------------------------------
//  输入:	double M[]	原矩阵名称
//			int iRow	行数
//			int iCol	列数
//  输出:	double P[]	新的矩阵P
//  返回:	无
//  说明:	M为源矩阵, P为目标矩阵
//  -------------------------------------------------------------
// ***************************************************************
//void LAMBDA_MO_MatrixSame(double M[], double P[], int iRow, int iCol);



//  **************************************************************
//  目的:	求矩阵的列向量
//  -------------------------------------------------------------
//  输入:	double M[]	矩阵名称
//			int nRow	行数
//			int nCol	列数
//			int k		所求列向量的列数
//  输出: 	double Rv[]	列向量名
//  返回:	无
//  说明:	求得M矩阵的第k列向量并放入Rv中,k=1:nCol,程序中自动减1
//  -------------------------------------------------------------
// ***************************************************************
//void LAMBDA_MO_GetCol(double M[], double Cv[], int nRow, int nCol, int k);
//void LAMBDA_MO_GetRow(double M[], double Cv[], int nRow, int nCol, int k);




//  **************************************************************
//  目的:	矩阵相加 C = A + B
//  -------------------------------------------------------------
//  输入:	double A[]
//			double B[]	相加的矩阵
//			int nRow
//			int nCol	行列数
//  输出:	double C[]	相加结果
//  返回:	无
//  说明:	A,B,C三矩阵维数要相同
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixPlus(double A[], double B[], double C[], int nRow, int nCol);

//  **************************************************************
//  目的:	矩阵求逆(覆盖原矩阵)
//  -------------------------------------------------------------
//  输入:	double M[]	矩阵名称
//			int nDim	矩阵维数
//  输出:	double M[]	求逆后的矩阵
//  返回:	true	求逆成功
//			FALSE	求逆失败
//  说明:	此种方法求逆后结果会覆盖原来矩阵,数值稳定性不是很好
//  -------------------------------------------------------------
// ***************************************************************
bool LAMBDA_MO_MatrixInv( double M[], int nDim );


//  **************************************************************
//  目的:	矩阵相乘 N(r*r) = A*P*A', 其中A(r*n), P(n*n)
//  -------------------------------------------------------------
//  输入:	double A[], double P[]	相乘矩阵
//			int RowA, int ColA		行列数
//  输出:	double APAT[]	相乘结果
//  返回:	无
//  说明:	无
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixAPAT(double A[], double P[], double APAT[], int RowA, int ColA);



//  **************************************************************
//  目的:	矩阵的数乘 A = k * A, i.e. A[i][j] = k * A[i][j]
//  -------------------------------------------------------------
//  输入:	double A[]	原矩阵
//			double k	数乘因子,也就是放大或者缩小的倍数
//			int nRow	行数
//			int nCol	列数
//  输出:	double A[]	数乘后放在原来矩阵中
//  返回:	无
//  说明:	数乘后覆盖原来矩阵
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixAmplify(double A[],double k,int nRow, int nCol);

void LAMBDA_MO_MatrixAmplify1(double A[],double k[],int nRow, int nCol);

void LAMBDA_Matrix_Eye(double A[],int nRow);



//  **************************************************************
//  目的:	矩阵的数乘 M = k * A, i.e. M[i][j] = k * A[i][j]
//  -------------------------------------------------------------
//  输入:	double A[]	原矩阵
//			double k	数乘因子,即倍数
//			int nRow	行数
//			int nCol	列数
//  输出:	double M[] 数乘后的矩阵
//  返回:	无
//  说明:	数乘后放在新矩阵M中,不覆盖原来矩阵A
//  -------------------------------------------------------------
// ***************************************************************
//void LAMBDA_MO_MatrixAmplify_Stay(double A[], double M[], double k, int nRow, int nCol);

//  **************************************************************
//  目的:	得到单位矩阵
//  -------------------------------------------------------------
//  输入:	int n
//  输出:	double M[] 数乘后的矩阵
//  返回:	无
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
//void LAMBDA_MO_MatrixEye(double M[], int n);

//  **************************************************************
//  目的:	得到矩阵的迹
//  -------------------------------------------------------------
//  输入:	int n
//  输出:	double M[] 数乘后的矩阵
//  返回:	无
//  说明:	
//  -------------------------------------------------------------
// ***************************************************************
//double LAMBDA_MO_MatrixTrace(double M[], int n);

#endif //#define LAMBDA_H_

