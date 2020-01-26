
#include "typedefine.h"
#include "define.h"

#include "matrix.h"
#include "KfPVT.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "constdef.h"


void setMatrHandle(TMatrix *handle ,int32 realrows, int32 realcols, int32 rows, int32 cols, void *p)
{
	handle->realrows = realrows;
	handle->realcols = realcols;

	handle->rows = rows;
	handle->cols = cols;

	handle->p = (double*)p;
}

 void setVectorHandle(TVector *handle, int32 realrows, int32 rows, void *p)
{
	handle->realrows = realrows;
	handle->rows = rows;
	handle->p = (double*)p;
}


static void swap(double* p1, double *p2)
{
	double _tmp = *p1;

	*p1 = *p2;
	*p2 = _tmp;

	return;
}


//Gauss-Jordan matrix inverse method
int MatrixInverse(TMatrix *pSrc, TMatrix *pInvs)
{
	int32 i  = 0, j = 0, k = 0;
	int32 js[MAX_Z_DIM]={0,}, is[MAX_Z_DIM]={0,};
	double matrix[MAX_Z_DIM][MAX_Z_DIM] = {{0.0,},};
	double _tmp, fMax=0.0;
	word32 copyaddr = 0;
	int32 copylen = pSrc->cols * sizeof(double);
	
	if((pSrc->cols != pSrc->rows) ||(pInvs->cols != pInvs->rows) || (pSrc->rows != pInvs->cols))
		return -1;
	
	for (i = 0; i < pSrc->rows; i ++)
	{
		copyaddr = (word32)pSrc->p + i*(pSrc->realcols * sizeof(double));
		
		memcpy((void *)&(matrix[i][0]), (const void *)copyaddr, copylen);
	}

	for(k = 0; k < pSrc ->cols; k++)
	{
		//select the max element in each row
		fMax = 0.0;
		
		for(i = k; i < pSrc->rows; i ++)
		{
			for(j = k; j < pSrc->cols; j ++)
			{
				_tmp = f_abs(matrix[i][j]);

				if(_tmp > fMax)
				{
					fMax = _tmp;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if(fMax < MICRO_NUM)
			return -1;		//exist some rows whose elements are all zeros, this matrix do not exist inverse one

		if(is[k] != k)
		{
			for(i = 0; i < pSrc->cols; i ++)
				swap(&matrix[k][i], &matrix[is[k]][i]);	
		}

		if(js[k] != k)
		{
			for(i = 0; i < pSrc->cols; i ++)
				swap(&matrix[i][k], &matrix[i][js[k]]);	
		}
		
		matrix[k][k] = 1.0 / matrix[k][k];

		for(j = 0; j < pSrc->cols; j ++)
		{
			if(j != k)
				matrix[k][j] *= matrix[k][k];
		}

		for(i = 0; i < pSrc->rows; i ++)
		{
			if(i != k)
			{
				for(j = 0; j < pSrc->cols; j ++)
				{
					if(j != k)
						matrix[i][j] -= matrix[i][k] * matrix[k][j];
				}
			}
		}

		for(i = 0; i < pSrc->rows; i ++)
		{
			if(i != k)
				matrix[i][k] *= -matrix[k][k];
		}
	}

	for	(k = pSrc->cols - 1; k >= 0; k --)
	{
		if (js[k] != k)
		{
			for(i = 0; i < pSrc->cols; i ++)
				swap(&matrix[k][i], &matrix[js[k]][i]);			
		}
		
		if (is[k] != k)
		{
			for(i = 0; i < pSrc->rows; i ++)
				swap(&matrix[i][k], &matrix[i][is[k]]);						
		}
	}

	copyaddr = 0;
	copylen = pSrc->cols * sizeof(double);	

	for (i = 0; i < pSrc->rows; i ++)
	{
		copyaddr = (word32)pInvs->p + i*(pInvs->realcols * sizeof(double));
		
		memcpy((void *)copyaddr, (const void *)&(matrix[i][0]), copylen);
	}

	
	return 0;
}


/**
*	@Brief	Transpose a matrix.
*	@Param	source matrix and destination matrix struct pointer
*	@Return	0,success; -1, fail
*/

int MatrixTranspose(TMatrix *pSrc, TMatrix *pTrans)
{
	int row,col;
	int rows, cols;
	double *pSrcMatrx;
	double *pTransMatrx;

	if(pSrc->rows != pTrans->cols || pSrc->cols != pTrans->rows)
		return -1;

	//get source matrix and destination matrix
	pSrcMatrx = pSrc->p;
	pTransMatrx = pTrans->p;

	rows = pSrc->rows;
	cols = pSrc->cols;

	//do transpose operation
    for (row = 0; row < rows; row++)
	 for( col = 0; col < cols; col++)
		pTransMatrx[col * (pTrans->realcols) + row] = pSrcMatrx[row * (pSrc->realcols) + col];

	return 0;
}


/**
*	@Brief	Matrix mul to a matrix, get the OP result matrix
*	@Param	source matrix and destination matrix struct pointer
*	@Return	0,success; -1, fail
*/

int MatrixMul(TMatrix *pM1, TMatrix *pM2, TMatrix *res)
{
	double *pres;
	double *p1;
	double *p2;
	double sum=0.0;
	int i,j,k;

	p1 = pM1->p;
	p2 = pM2->p;
	pres = res->p;

	if(pM1->cols != pM2->rows)
		return -1;
	if(res->rows != pM1->rows || res->cols != pM2->cols)
		return -1;

	for(i=0;i<res->rows;i++)
	{
		for(j=0;j<res->cols;j++)
		{
			sum=0.0;
			for(k=0;k<pM1->cols;k++)
			{
				int32 p1pos = i * (pM1 ->realcols) + k;
				int32 p2pos = k * (pM2 ->realcols) + j;

				sum = sum + p1[p1pos] * p2[p2pos];
			}

			pres[i * (res->realcols) + j]=sum;
		}
	}

	return 0;
}

/**
*	@Brief	Matrix copy operation
*	@Param	source matrix and destination matrix struct pointer
*	@Return	0,success; -1, fail
*/
int MatrixCopy(TMatrix *pSrc, TMatrix *pDes)
{
	int i,j;
	double *src;
	double *des;
	src = pSrc->p;
	des = pDes->p;

	if(pSrc->rows != pDes->rows || pSrc->cols != pDes->cols)
		return -1;

	for(i=0; i<pSrc->rows; i++)
		for(j=0; j<pSrc->cols; j++)
			des[i * (pDes->realcols) + j] = src[i * (pSrc->realcols) + j];

	return 0;
}

/**
*	@Brief	Matrix addition operation
*	@Param	2 source matrixs and 1 destination matrix struct pointer
*	@Return	0,success; -1, fail
*/

int MatrixAdd(TMatrix *pSrc1, TMatrix *pSrc2, TMatrix *pDes)
{
	int i,j;
	double *src1;
	double *src2;
	double *des;
	
	if(pSrc1->rows != pSrc2->rows || pSrc1->rows != pDes->rows)
		return -1;
	if(pSrc1->cols != pSrc2->cols || pSrc1->cols != pDes->cols)
		return -1;

	src1 = pSrc1->p;
	src2 = pSrc2->p;
	des = pDes->p;

	for(i=0; i<pSrc1->rows; i++)
		for(j=0; j<pSrc1->cols; j++)
			des[i * (pDes->realcols) + j] = src1[i * (pSrc1->realcols) + j] + src2[i * (pSrc2 ->realcols) + j];

	return 0;
}


/**
*	@Brief	Matrix subtraction operation
*	@Param	2 source matrixs and 1 destination matrix struct pointer
*	@Return	0,success; -1, fail
*/

int MatrixSub(TMatrix *pMinuend, TMatrix *pSubtrahend, TMatrix *pDiff)
{
	int i,j;
	double *min;
	double *sub;
	double *diff;
	
	if(pMinuend->rows != pSubtrahend->rows || pDiff->rows != pMinuend->rows)
		return -1;
	if(pMinuend->cols != pSubtrahend->cols || pDiff->cols != pMinuend->cols)
		return -1;

	min = pMinuend->p;
	sub = pSubtrahend->p;
	diff = pDiff->p;

	for(i=0; i<pDiff->rows; i++)
		for(j=0; j<pDiff->cols; j++)
			diff[i * (pDiff->realcols) + j] = min[i * (pMinuend->realcols) + j] - sub[i * (pSubtrahend->realcols) + j];

	return 0;
}

/**
*	@Brief	Matrix mul to a Vector(same as the diagonal matrix), get the OP result matrix
*	@Param	source matrix and destination matrix struct pointer
*	@Return	0,success; -1, fail
*/
int MatrixMulVector(TMatrix *pM1, TVector *pM2, TMatrix *res)
{
	double (*pres);
	double (*p1);
	double (*p2);
	int i,j;

	p1 = pM1->p;
	p2 = pM2->p;
	pres = res->p;

	if(pM1->cols != pM2->rows)
		return -1;
	if(res->rows != pM1->rows)
		return -1;

	for(i=0;i<res->rows;i++)
		for(j=0;j<res->cols;j++)
			pres[i * (res->realcols) + j]= p1[i * (pM1->realcols) + j]*p2[j];

	return 0;
}


/**
*	@Brief	Matrix addition operation (matrix + vector)
*	@Param	2 source matrixs and 1 destination matrix struct pointer
*	@Return	0,success; -1, fail
*/
int MatrixAddVector(TMatrix *pSrc1, TVector *pSrc2, TMatrix *pDes)
{
	int i,j;
	double (*src1);
	double (*src2);
	double (*des);
	
	if(pSrc1->rows != pSrc2->rows || pSrc1->rows != pDes->rows)
		return -1;
	if(pSrc1->cols != pDes->cols)
		return -1;

	src1 = pSrc1->p;
	src2 = pSrc2->p;
	des = pDes->p;

	for(i=0; i<pSrc1->rows; i++)
	{
		for(j=0; j<pSrc1->cols; j++)
		{
			if(i == j)
				des[i * (pDes->realcols) + j] = src1[i * (pSrc1->realcols) + j] + src2[i];
			else
				des[i * (pDes->realcols) + j] = src1[i * (pSrc1->realcols) + j];
		}
	}

	return 0;
}



/**
*	@Brief	Matrix subtraction operation (matrix - vector)
*	@Param	2 source matrixs and 1 destination matrix struct pointer
*	@Return	0,success; -1, fail
*/
int MatrixSubVector(TMatrix *pSrc1, TVector*pSrc2, TMatrix *pDiff)
{
	int i,j;
	double (*min);
	double (*sub);
	double (*diff);
	
	if(pSrc1->rows != pSrc2->rows || pDiff->rows != pSrc1->rows)
		return -1;
	if(pDiff->cols != pSrc1->cols)
		return -1;

	min = pSrc1->p;
	sub = pSrc2->p;
	diff = pDiff->p;

	for(i=0; i<pDiff->rows; i++)
		for(j=0; j<pDiff->cols; j++)
		{
			if(i == j)
				diff[i * (pDiff->realcols) + j] = min[i * (pSrc1->realcols) + j] - sub[i];
			else
				diff[i * (pDiff->realcols) + j] = min[i * (pSrc1->realcols) + j];
		}

	return 0;
}


/**
*	@Brief	Matrix subtraction operation (vector - matrix)
*	@Param	2 source matrixs and 1 destination matrix struct pointer
*	@Return	0,success; -1, fail
*/
int VectorSubMatrix(TVector* pSrc1, TMatrix* pSrc2, TMatrix *pDiff)
{
	int i,j;
	double (*min);
	double (*sub);
	double (*diff);
	
	if(pSrc1->rows != pSrc2->rows || pDiff->rows != pSrc1->rows)
		return -1;
	if(pDiff->cols != pSrc2->cols)
		return -1;

	min = pSrc1->p;
	sub = pSrc2->p;
	diff = pDiff->p;

	for(i=0; i<pDiff->rows; i++)
	{
		for(j=0; j<pDiff->cols; j++)
		{
			if(i == j)
				diff[i * (pDiff->realcols) + j] = min[i] - sub[i * (pSrc2->realcols) + j];
			else
				diff[i * (pDiff->realcols) + j] = -sub[i * (pSrc2->realcols) + j];
		}
	}

	return 0;
}

void EyeMatrix(TMatrix* pMatrix, double value)
{
	int32 i=0, j=0;

	double *des = pMatrix->p;

	for(i=0; i<pMatrix->realrows; i++)
	{
		for(j=0; j<pMatrix->realcols; j++)
		{
			if(i==j)
				des[i*pMatrix->realcols+j]= value;
			else
				des[i*pMatrix->realcols+j] = 0.0;
		}
	}

	return;
}


void ZerosMatrix(TMatrix* pMatrix)
{
	int32 i=0, j=0;
	double* des = pMatrix->p;

	for(i=0; i<pMatrix->realrows; i++)
	{
		for(j=0; j<pMatrix->realcols; j++)
			des[i*pMatrix->realcols+j] = 0.0;
	}

	return;
}

																									\



