
#ifndef _HEAD_MATRIX_H_
#define _HEAD_MATRIX_H_

#include "define.h"

#define SORT_TYPE_ASC	0
#define SORT_TYPE_DESC	1

typedef struct{
	int8 trkch;
	double value;
}SORT_DES;


/* Matrix struct type definition */
typedef struct{
	int32 realrows;
	int32 realcols;
	int32 rows;
	int32 cols;
	double *p;
}TMatrix;


typedef struct{
	int32 realrows;
	int32 rows;
	double* p;
}TVector;


extern void setMatrHandle(TMatrix *handle ,int32 realrows, int32 realcols, int32 rows, int32 cols, void *p);
extern void setVectorHandle(TVector *handle, int32 realrows, int32 rows, void *p);
extern int32 MatrixTranspose(TMatrix *pSrc, TMatrix *pTrans);
extern int32 MatrixInverse(TMatrix *pSrc, TMatrix *pInvs);

extern int32 MatrixCopy(TMatrix *pSrc, TMatrix *pDes);
extern int32 MatrixMul(TMatrix *pM1, TMatrix *pM2, TMatrix *res);

extern int MatrixAdd(TMatrix *pSrc1, TMatrix *pSrc2, TMatrix *pDes);
extern int MatrixSub(TMatrix *pMinuend, TMatrix *pSubtrahend, TMatrix *pDiff);

extern int MatrixMulVector(TMatrix *pM1, TVector *pM2, TMatrix *res);
extern int MatrixAddVector(TMatrix *pSrc1, TVector *pSrc2, TMatrix *pDes);
extern int MatrixSubVector(TMatrix *pSrc1, TVector*pSrc2, TMatrix *pDiff);
extern int VectorSubMatrix(TVector* pSrc1, TMatrix* pSrc2, TMatrix *pDiff);

extern void EyeMatrix(TMatrix* pMatrix, double value);
extern void ZerosMatrix(TMatrix* pMatrix);


#endif

