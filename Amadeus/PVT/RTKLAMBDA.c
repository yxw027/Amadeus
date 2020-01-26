/*******************************************************************
Company:   hwacreate
Engineer:  
Create Date: 2015.12.01
File  Name:  RTKLAMBDA.c
Description: function of LAMBDA
Function List: 
version: V1.0
Revision Date:
Modifier: 
Additional Comments: 
********************************************************************/
#include <math.h>
#include "RTKLAMBDA.h"


//  **************************************************************
//  Ŀ��:	MLAMBDA�㷨ʵ��
//  -------------------------------------------------------------
//  ����:	
//		a:		�����
//		Q:		Э�������
//		nDim:	ģ���ȸ���
//		ncands:	��ѡ�̶������
//		factor:	��������
//  ���:	
//		afixed:	�̶���
//		sqnorm:	����
//		Qahat:	Z�任��ķ������
//		Z:		�����任
//  ����:	���ɹ����
//  ˵��:	
//  -------------------------------------------------------------
// ***************************************************************
int M_LAMBDA(double *a, double *Q, int nDim, int ncands, double *afixed, double *sqnorm, double *Qahat, double *Z)
{
	bool info = 0;
	int i=0;
	//int j=0;
	double L[LAMBDA_N_DIM*LAMBDA_N_DIM] = {0};
	double L2[LAMBDA_N_DIM*LAMBDA_N_DIM] = {0};
	double D[LAMBDA_N_DIM] = {0};
	double z[LAMBDA_N_DIM*LAMBDA_N_CANDS] = {0};
	double E[LAMBDA_N_DIM*LAMBDA_N_CANDS] = {0};
	int n = nDim;
	int m = ncands;

	memset(Z, 0, sizeof(double)*nDim*nDim);
	for (i=0; i<nDim; i++)
	{
		Z[i*nDim+i] = 1;
	}
	/* LD factorization */
	info=LAMBDA_MO_LTDL2(Q,L,D,n);
	if (info)
	{
		/* lambda reduction */
		M_LAMBDA_reduction(n,L,D,Z);
		LAMBDA_MO_Round(Z, n, n, Z);
		LAMBDA_MO_MatrixATB(Z, a, z, nDim, nDim, 1);

		/* mlambda search */
		info=M_LAMBDA_search(n,m,L,D,z,E,sqnorm);
		if (info)
		{	
			LAMBDA_MO_MatrixTransp(Z, L2, nDim, nDim);
			LAMBDA_MO_MatrixInv_LU(L2, Z, nDim);
			LAMBDA_MO_MatrixMulti(Z, E, afixed, nDim, nDim, m);
		}
		else
		{
			return -2;
		}
	}
	else
	{
		return -1;
	}
	return 1;
}


/* integer gauss transformation ----------------------------------------------*/
void M_LAMBDA_gauss(int n, double *L, double *Z, int i, int j)
{
    int k;
	double mu = 0;  
	
	mu=ROUND(L[i*n+j]);
    if (mu!=0.0) {
        for (k=i;k<n;k++) L[k*n+j]-=mu*L[k*n+i];
        for (k=0;k<n;k++) Z[k*n+j]-=mu*Z[k*n+i];
    }
}
/* permutations --------------------------------------------------------------*/
void M_LAMBDA_perm(int n, double *L, double *D, int j, double del, double *Z)
{
    int k;
    double eta,lam,a0,a1;
    
    eta=D[j]/del;
    lam=D[j+1]*L[(j+1)*n+j]/del;
    D[j]=eta*D[j+1]; D[j+1]=del;
    for (k=0;k<=j-1;k++) {
        a0=L[j*n+k]; a1=L[(j+1)*n+k];
        L[j*n+k]=-L[(j+1)*n+j]*a0+a1;
        L[j*n+1*n+k]=eta*a0+lam*a1;
    }
    L[j*n+1*n+j]=lam;
    for (k=j+2;k<n;k++) SWAP(&L[k*n+j],&L[k*n+(j+1)]);
    for (k=0;k<n;k++) SWAP(&Z[k*n+j],&Z[k*n+(j+1)]);
}
/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
void M_LAMBDA_reduction(int n, double *L, double *D, double *Z)
{
    int i,j,k;
    double del;
  
    j=n-2; k=n-2;
    while (j>=0) {
        if (j<=k) for (i=j+1;i<n;i++) M_LAMBDA_gauss(n,L,Z,i,j);
        del=D[j]+L[(j+1)*n+j]*L[(j+1)*n+j]*D[j+1];
        if ((del+1E-6)<D[j+1]) { /* compared considering numerical error */
            M_LAMBDA_perm(n,L,D,j,del,Z);
            k=j; j=n-2;
        }
        else j--;
    }
}
/* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
bool M_LAMBDA_search(int n, int m, const double *L, const double *D,
                  const double *zs, double *zn, double *s)
{
    int i,j,k,c,nn=0,imax=m-1;
	double max_s = 0;
    double newdist=0.0,maxdist=1E99,y=0.0;
	double S[LAMBDA_N_DIM*LAMBDA_N_DIM] = {0};
	double dist[LAMBDA_N_DIM*1] = {0};
	double zb[LAMBDA_N_DIM*1] = {0};
	double z[LAMBDA_N_DIM*1] = {0};
	double step[LAMBDA_N_DIM*1] = {0};
    
    k=n-1; dist[k]=0.0;
    zb[k]=zs[k];
    z[k]=ROUND(zb[k]); y=zb[k]-z[k]; step[k]=SGN(y);
    for (c=0;c<LOOPMAX;c++) {
        newdist=dist[k]+y*y/D[k];
        if (newdist<maxdist) {
            if (k!=0) {
                dist[--k]=newdist;
                for (i=0;i<=k;i++)
                    S[k*n+i]=S[k*n+1*n+i]+(z[k+1]-zb[k+1])*L[k*n+1*n+i];
                zb[k]=zs[k]+S[k*n+k];
                z[k]=ROUND(zb[k]); y=zb[k]-z[k]; step[k]=SGN(y);
            }
            else {
                if (nn<m-1)
				{                    
                    for (i=0;i<n;i++) zn[i*m+nn]=z[i];
                    s[nn++]=newdist;
                }
                else {
					for (i=0;i<n;i++) 
						zn[i*m+imax]=z[i];
					s[imax]=newdist;

					max_s = -1e10;
					for (i=0;i<m;i++) 
					{
						if (max_s<s[i]) 
						{
							imax=i;
							max_s = s[i];
						}
					}
					maxdist = max_s;
                }
                z[0]+=step[k]; y=zb[0]-z[0]; step[0]=-step[0]-SGN(step[0]);
            }
        }
        else {
            if (k==n-1) break;
            else {
                k++;
                z[k]+=step[k]; y=zb[k]-z[k]; step[k]=-step[k]-SGN(step[k]);
            }
        }
    }
    for (i=0;i<m-1;i++) { /* sort by s */
        for (j=i+1;j<m;j++) {
            if (s[i]<s[j]) continue;
            SWAP(&s[i],&s[j]);
            for (k=0;k<n;k++) SWAP(&zn[k*m+i],&zn[k*m+j]);
        }
    }
   
    if (c>=LOOPMAX) 
	{
        return FALSE;
    }
    return TRUE;
}

//////////////////////////////////////////////////////////////////////////


//  **************************************************************
//  Ŀ��:	�Գ���������ķֽ� P(n*n) = L'(n*n) * D(n*n) * L(n*n)
//  -------------------------------------------------------------
//  ����:	double P[]	�Գƾ���
//			int nDim	����ά��
//  ���:	double L[]	�����Ǿ���
//			double D[]	�ԽǾ���
//  ����:	true	�ɹ�
//			FALSE	ʧ��
//  ˵��:	ԭ�������Ϊ�Գƾ���,�����Ľ��ΪL��һ�����Ǿ���(�Խ�����
//			Ԫ��Ϊ1),��DΪn*1������.�������о���ά����Ϊn*n
//  -------------------------------------------------------------
// ***************************************************************
bool LAMBDA_MO_LTDL2(double P[], double L[], double D[], int nDim)
{
	double dPtemp[LAMBDA_N_DIM*LAMBDA_N_DIM] = {0};
	int n = 0;
	int i = 0;
	int j = 0;
	int k = 0;
	
	n = nDim;
	memcpy(dPtemp, P, sizeof(double)*nDim*nDim);//��������,��ֹP�����ƻ�

	for(i=n-1; i>=0; i--)
	{
		if(dPtemp[i*n+i]<=0.0)
			return FALSE;			
		D[i] = dPtemp[i*n+i];
		for (j=0; j<=i; j++)
		{
			L[i*n+j] = dPtemp[i*n+j] / sqrt(dPtemp[i*n+i]);
		}

		for(j=0; j<=i-1; j++)
		{
			for (k=0; k<=j; k++)
			{
				dPtemp[j*n+k] = dPtemp[j*n+k] - L[i*n+k]*L[i*n+j];
			}
		}

		for (j=0; j<=i; j++)
		{
			L[i*n+j] = L[i*n+j]/L[i*n+i];
		}
	}

	for (i=0; i<n; i++)
	{
		if (D[i] < LAMBDA_EPSILON)
		{
			return FALSE;
		}
	}

	return TRUE;
}


//  **************************************************************
//  Ŀ��:	����ӽ�������������
//  -------------------------------------------------------------
//  ����:	
//			double* dValue
//			int nRow
//			int nCol
//  ���:	
//			double* nValue
//  ����:	
//  ˵��:	
//  -------------------------------------------------------------
// ***************************************************************
int LAMBDA_MO_Round(double* dValue, int nRow, int nCol, double* nValue)
{
	int i=0;
	int j=0;
	//double dFloorValue = 0;
	//double dFrac = 0;
	for (i=0; i<nRow; i++)
	{
		for (j=0; j<nCol; j++)
		{
			nValue[i*nRow+j] =  floor(dValue[i*nRow+j]+0.5);
		}
	}
	return 1;
}

//  **************************************************************
//  Ŀ��:	������ת��
//  -------------------------------------------------------------
//  ����:	double M[]	��������
//			int nRow	����
//			int nCol	����
//  ���:	double M_Transp[]	ת�ú�ľ���
//  ����:	��
//  ˵��:	������ת�ú������һ������,ԭ���󲻱�
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixTransp(double M[], double M_Transp[],int nRow, int nCol)
{
	int i=0;
	int j=0;
	for (i=0; i<nCol; i++)
		for (j=0; j<nRow; j++)
			M_Transp[i*nRow+j] = M[j*nCol+i]; 
}



//  **************************************************************
//  Ŀ��:	����LU�ֽ����������(����ԭ����)
//  -------------------------------------------------------------
//  ����:	double M[]		ԭ����
//			int nDim		����ά��
//  ���:	double M_Inv[]	�����
//  ����:	true	�ɹ�
//			FALSE	ʧ��
//  ˵��:	��ֵ�ȶ��Ժ�,���������µľ���M_Inv��
//  -------------------------------------------------------------
// ***************************************************************
bool LAMBDA_MO_MatrixInv_LU(double M[], double M_Inv[], int nDim)
{
	bool bFlag = FALSE;
	int j=0;

	double LU[LAMBDA_N_DIM*LAMBDA_N_DIM] = {0};
	double b[LAMBDA_N_DIM] = {0};	//�����bΪe(1),e(2),...e(n);
	double Indx[LAMBDA_N_DIM] = {0};

	//��M������Lu����ֹ�ں���ķֽ��иı�ԭ�о���
	memcpy(LU, M, sizeof(double)*nDim*nDim);//LAMBDA_MO_MatrixSame(M, LU, n, n);

	// LU decomposition 
	bFlag = LAMBDA_MO_LU_Decom( LU, Indx, nDim);

	//�ֽⲻ�ɹ����޷�����
	if (!bFlag)
	{
		return FALSE;
	}

	// Solve Ax=b for  unit vectors b_1..b_n
	for (j=0; j<nDim; j++ ) 
	{
		memset(b, 0, nDim*sizeof(double));
		b[j]= 1.0;                     // Set b to j-th unit vector 

		LAMBDA_MO_LU_BackSub( LU, Indx, b, nDim);           // Solve Ax=b

		LAMBDA_MO_SetCol(M_Inv, b, nDim, nDim, j+1);
	};

	return TRUE;
}


//  **************************************************************
//  Ŀ��:	�������	N(r*r) = AT*P*A, ����, A(n*r), P(n*n)
//  -------------------------------------------------------------
//  ����:	double A[], double P[]
//			int RowA, int ColA
//  ���:	double ATPA[]
//  ����:	��
//  ˵��:	��
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixATPA(double A[], double P[], double ATPA[], int RowA, int ColA)
{
	double  paa = 0;
	int i=0;
	int j=0;
	int k=0;
	int l=0;

	for (i=0; i<ColA; i++)
	{
		for (j=0; j<i+1; j++)
		{
			paa = 0.0;
			for (k=0; k<RowA; k++)
				for (l=0; l<RowA; l++)
					paa += A[k * ColA + i] * P[k * RowA + l] * A[l * ColA +j];

			ATPA[i * ColA + j] = paa;
		}
	}

	//����ȷ�������ǶԳƵ�
	for (i=0; i<ColA-1; i++)
		for (j=i+1; j<ColA; j++)
			ATPA[i * ColA + j] = ATPA[j * ColA + i];

}


//  **************************************************************
//  Ŀ��:	һ�����������һ�����ת���� C(m*k) = A(m*n)*B'(k*n)
//  -------------------------------------------------------------
//  ����:	double A[]	double B[]				��˾���
//			int RowA	int ColA	 int RowB	������������
//  ���:	double ABT[]	�˷�������
//  ����:	��
//  ˵��:	��
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixABT(double A[], double B[], double ABT[],
						 int RowA, int ColA, int RowB)
{
	double  ab = 0.0;
	int i=0;
	int j=0;
	int l=0;
	for (i=0; i<RowA; i++)
	{
		for (j=0; j<RowB; j++)
		{
			ab = 0.;
			for (l=0; l<ColA; l++)
				ab += A[i * ColA + l] * B[j * ColA + l];
			ABT[i * RowB + j] = ab;
		}
	}
}



//  **************************************************************
//  Ŀ��:	������� C(m*s) = A(m*n) * B(n*s)
//  -------------------------------------------------------------
//  ����:	double A[]
//			double B[]	��˵ľ���
//			int nRowA	A������
//			int nColA	A������
//			int nColB	B������
//  ���:	double C[]	��˽��
//  ����:	��
//  ˵��:	C(m*s) = A(m*n) * B(n*s)
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixMulti(double A[],double B[],double C[], int nRowA, int nColA, int nColB)
{
	int i=0;
	int j=0;
	int k=0;
	for(i=0; i<nRowA; i++ )
	{
		for(j=0; j<nColB; j++ )
		{
			C[i * nColB + j] = 0.0;
			for(k=0; k<nColA; k++ )
				C[i * nColB + j] += A[i * nColA + k] * B[k * nColB + j];
		}
	}
}


//  **************************************************************
//  Ŀ��:	һ�������ת�ó�����һ����   C(m*k) = A'(n*m)*B(n*k)
//  -------------------------------------------------------------
//  ����:	double A[], double B[]			ԭ���� 
//			int RowA, int ColA, int ColB	������������
//  ���:	double ATB[]	��˽��
//  ����:	��
//  ˵��:	��
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixATB(double A[], double B[], double ATB[], 
							int RowA, int ColA, int ColB)
{
	double  ab = 0.0;
	int i=0;
	int j=0;
	int l=0;

	for (i=0; i<ColA; i++)
	{
		for (j=0; j<ColB; j++)
		{
			ab = 0.0;
			for (l=0; l<RowA; l++)
				ab += A[l * ColA + i] * B[l * ColB + j];
			ATB[i * ColB + j] = ab;
		}
	}
}



//  **************************************************************
//  Ŀ��:	������� C = A - B
//  -------------------------------------------------------------
//  ����:	double A[]
//			double B[]	����ľ���
//			int nRow
//			int nCol	������
//  ���:	double C[]	������
//  ����:	��
//  ˵��:	A,B,C������ά��Ҫ��ͬ
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixSub(double A[], double B[], double C[], int nRow, int nCol)
{
	int i = 0;
	int j = 0;

	for (i = 0; i < nRow; i ++)
		for (j = 0; j < nCol; j ++)
			C[i * nCol + j] = A[i * nCol + j] - B[i * nCol + j];

}


//  **************************************************************
//  Ŀ��:	  �������Գ�
//  -------------------------------------------------------------
//  ����:	
//			double *M
//			int nRowM
//			int nColM
//  ���:	
//			double *M
//  ����:	
//  ˵��:	
//  -------------------------------------------------------------
// ***************************************************************
int LAMBDA_MO_MatrixMakeSymmetry(double *M, int nDim)
{
	int i = 0;
	int j = 0;
	
/*	LAMBDA_MO_MatrixTransp(M, temp1, nDim, nDim);
	LAMBDA_MO_MatrixPlus(M, temp1, temp2, nDim, nDim);
	LAMBDA_MO_MatrixAmplify_Stay(temp2, M, 0.5, nDim, nDim);*/

	// ���ֶԽ��������ݲ���	
	for(i=0; i<nDim; i++)
		for(j=i+1; j<nDim; j++)
		{
			M[i*nDim+j] = 0.5*(M[j*nDim+i] + M[i*nDim+j]);	// �������Ͻ�����
			M[j*nDim+i] = M[i*nDim+j];						// ��ֵ���½�����
		}
	return 1;
}


//  **************************************************************
//  Ŀ��:	������� C = A + B
//  -------------------------------------------------------------
//  ����:	double A[]
//			double B[]	��ӵľ���
//			int nRow
//			int nCol	������
//  ���:	double C[]	��ӽ��
//  ����:	��
//  ˵��:	A,B,C������ά��Ҫ��ͬ
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixPlus(double A[], double B[], double C[], int nRow, int nCol)
{
	int i=0;
	int j=0;
	for (i = 0; i < nRow; i ++)
		for (j = 0; j < nCol; j ++)
			C[i * nCol + j] = A[i * nCol + j] + B[i * nCol + j];
}


//  **************************************************************
//  Ŀ��:	����M��ֵ��P,��	P = M
//  -------------------------------------------------------------
//  ����:	double M[]	ԭ��������
//			int iRow	����
//			int iCol	����
//  ���:	double P[]	�µľ���P
//  ����:	��
//  ˵��:	MΪԴ����, PΪĿ�����
//  -------------------------------------------------------------
// ***************************************************************
/*void LAMBDA_MO_MatrixSame(double M[], double P[], int iRow, int iCol)
{
	int i=0;
	int j=0;

	for (i=0; i<iRow; i++)
	{
		for (j=0; j<iCol; j++)
		{
			P[i*iCol+j] = M[i*iCol+j];
		}
	}
}*/

//  **************************************************************
//  Ŀ��:	������LU�ֽ⣬P*M = L*U
//  -------------------------------------------------------------
//  ����:	double M[]	ԭ����
//			int nDim	����ά��
//  ���:	double M[]		�ֽ��LU���󸲸�
//			double Inx[]	����M��һ���û�,��LU_BackSubʱ����
//  ����:	true	�ɹ�
//			FALSE	ʧ��
//  ˵��:	P*M = L*U������M����LU���󸲸ǣ�Inx�Ǿ���M��һ���û�
//			LU�ֽ���ѡ��Ԫ�ģ������û��������ֵ�ȶ��Ժ�
//  -------------------------------------------------------------
// ***************************************************************
bool LAMBDA_MO_LU_Decom(double M[], double Inx[], int nDim)
{
	// Constants
	const int n = nDim;
	const double tiny = 1.0e-20;       // A small number

	// Variables
	int     i=0,j=0,imax=0,k=0;
	double  aAmax, Sum, Dum;

	// Loop over rows to get scaling information
	for (i=0; i<n; i++)
	{
		aAmax = 0.0;
		for (j=0;j<n;j++)
		{
			if (fabs( M[i * n + j] ) > aAmax )
				aAmax=fabs( M[i * n + j] );
		}
		if (aAmax==0.0)
		{
			// No nonzero largest element
			//cerr << "ERROR: Singular matrix A in LU_Decomp";
			return FALSE;
		};
		Inx[i] = 1.0/aAmax;           // V stores the implicit scaling of each row
	};

	// Loop over columns of Crout's method
	for ( j=0; j<n; j++ ) 
	{
		if (j > 0) 
		{
			for ( i=0; i<j; i++ )
			{   // This is equation 2.3.12 except for i=j
				Sum =  M[i * n + j];
				if (i>0) 
				{
					for ( k=0; k<i; k++ )  Sum -=  M[i * n + k] * M[k * n + j] ;
					M[i * n + j]  = Sum;
				};
			};
		};

		aAmax=0.0;                  // Initialize for the search of the largest
		// pivot element

		for ( i=j; i<n; i++ ) 
		{     // This is i=j of equation 2.3.12 and 
			Sum =  M[i * n + j] ;             // i=j+1..N of equation 2.3.13
			if (j > 0) 
			{
				for ( k=0; k<j; k++ ) 
					Sum -= M[i * n + k] * M[k * n + j];

				M[i * n + j] = Sum;
			};

			Dum = Inx[i]*fabs(Sum);     // Figure of merit for the pivot

			if (Dum >= aAmax) 
			{       // Is it better than the best so far ?
				imax  = i;
				aAmax = Dum;
			};
		};

		if (j != imax) 
		{            
			// Do we need to interchange rows?
			for ( k=0; k<n; k++) 
			{    
				// Yes, do so ...
				Dum = M[imax * n + k];
				M[imax * n + k] =  M[j * n + k];
				M[j * n + k] = Dum;
			}
			Inx[imax] = Inx[j];           // Also interchange the scale factor 
		};

		Inx[j] = imax;

		if (j != n-1)
		{             
			// Now finally divide by the pivot element
			if (M[j * n + j] == 0.0)
			{      
				// If the pivot element is zero the matrix 
				M[j * n + j] = tiny;          // is singular (at least to the precision of
			};                        // the algorithm). For some applications on

			Dum=1.0/M[j * n + j];           // singular matrices, it is desirable to 
			for (i=j+1;i<n;i++)
			{     
				// substitute tiny for zero. 
				M[i * n + j]=M[i * n + j]*Dum;
			};
		};

	};   // Go back for the next column in the reduction

	if (M[(n-1) * n + (n-1)]==0.0) M[(n-1) * n + (n-1)]=tiny; 

	return TRUE;
}

//  **************************************************************
//  Ŀ��:	���M*x=b������b�ڷ���ʱ��x����
//  -------------------------------------------------------------
//  ����:	double M[]		��������
//			double Inx[]	LU�ֽ����û�
//			double b[]		����
//			int nDim		ά��
//  ���:	double b[]		��õĽ�x
//  ����:	��
//  ˵��:	�˺�����LU_Decom���ʹ��,������������Է�����
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_LU_BackSub (double M[], double Inx[], double b[], int nDim)
{
	// Constants
	const int  n = nDim;

	// Local variables
	int     ii=0,i=0,ll=0,j=0;
	double  Sum=0;

	// Start
	// When ii is set to a nonegative value
	// it will become the first nonvanishing element of B. 
	ii = -1;  

	for (i=0; i<n; i++)
	{         
		// We now do the forward substitution.
		ll = (int) Inx[i];         // The only wrinkle is to unscramble the 
		Sum = b[ll];                // permutation as we go.
		b[ll] = b[i];
		if (ii != -1) 
		{
			for (j=ii; j<i; j++) Sum -= M[i * n + j]*b[j];
		}
		else 
		{
			if(fabs(Sum) > LAMBDA_EPSILON) 
				ii = i;   // A nonzero element was encountered, so from 
		};                          // now on we will have to do the sums in the
		b[i] = Sum;                 // loop above.
	};

	for (i=n-1; i>=0; i--) 
	{     
		// Now we do the back substitution, eqn 2.3.7.
		Sum=b[i];
		if (i<n-1) 
		{
			for (j=i+1;j<n;j++)
			{
				Sum = Sum-M[i * n + j]*b[j];
			};
		};
		b[i] = Sum/M[i * n + i];         // Store a component of the solution vector X.
	}
}


//  **************************************************************
//  Ŀ��:	������������
//  -------------------------------------------------------------
//  ����:	double M[]	��������
//			int nRow	����
//			int nCol	����
//			double Rv[]	��������
//			int k		����������������
//  ���: 	��
//  ����:	��
//  ˵��:	��M����ĵ�k������ΪRv, k=1:nCol,�������Զ���1
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_SetCol(double M[], double Cv[], int nRow, int nCol, int k)
{
	int i=0;
	if ( k>nCol || k<1)
	{
		return;
	}
	for (i=0; i<nRow; i++)
	{
		M[i*nCol+k-1] = Cv[i];
	}
}


//  **************************************************************
//  Ŀ��:	��������(����ԭ����)
//  -------------------------------------------------------------
//  ����:	double M[]	��������
//			int nDim	����ά��
//  ���:	double M[]	�����ľ���
//  ����:	true	����ɹ�
//			FALSE	����ʧ��
//  ˵��:	���ַ�����������Ḳ��ԭ������,��ֵ�ȶ��Բ��Ǻܺ�
//  -------------------------------------------------------------
// ***************************************************************
bool LAMBDA_MO_MatrixInv( double M[], int nDim )
{
	int i, j, k, m;
	double w, g;  
	double b[LAMBDA_N_DIM*LAMBDA_N_DIM] = {0};

	for( k=0; k<=nDim-1; k++)
	{
		w = M[0];
		if( fabs(w) < 1e-8 )
		{
			return 0;
		}
		m = nDim - k - 1;
		for( i=1; i<=nDim-1; i++)
		{
			g = M[i * nDim];
			b[i] = g / w;
			if( i <= m )  b[i] = - b[i];
			for( j=1; j<=i; j++)
				M[(i - 1) * nDim + j - 1] = M[i * nDim + j] + g * b[j];
		}
		M[nDim * nDim - 1] = 1.0 / w;
		for( i=1; i<=nDim-1; i++)
			M[(nDim - 1) * nDim + i - 1] = b[i];
	}
	for(i=0; i<=nDim-2; i++)
	{
		for( j=i+1; j<=nDim-1; j++)
			M[i * nDim + j] = M[j * nDim + i];
	}

	return 1;
}


//  **************************************************************
//  Ŀ��:	������� N(r*r) = A*P*A', ����A(r*n), P(n*n)
//  -------------------------------------------------------------
//  ����:	double A[], double P[]	��˾���
//			int RowA, int ColA		������
//  ���:	double APAT[]	��˽��
//  ����:	��
//  ˵��:	��
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixAPAT(double A[], double P[], double APAT[], int RowA, int ColA)
{
	double  paa = 0.0;
	int i=0;
	int j=0;
	int k=0;
	int l=0;
	for (i=0; i<RowA; i++)
	{
		for (j=0; j<i+1; j++)
		{
			paa = 0.;
			for (k=0; k<ColA; k++)
				for (l=0; l<ColA; l++)
					paa += A[i * ColA + k] * P[k * ColA + l] * A[j * ColA + l];

			APAT[i * RowA + j] = paa;
		}
	}

	//����ȷ�������ǶԳƵ�
	for (i=0; i<RowA-1; i++)
		for (j=i+1; j<RowA; j++)
			APAT[i * RowA + j] = APAT[j * RowA + i];
}



//  **************************************************************
//  Ŀ��:	��������� A = k * A, i.e. A[i][j] = k * A[i][j]
//  -------------------------------------------------------------
//  ����:	double A[]	ԭ����
//			double k	��������,Ҳ���ǷŴ������С�ı���
//			int nRow	����
//			int nCol	����
//  ���:	double A[]	���˺����ԭ��������
//  ����:	��
//  ˵��:	���˺󸲸�ԭ������
//  -------------------------------------------------------------
// ***************************************************************
void LAMBDA_MO_MatrixAmplify(double A[],double k,int nRow, int nCol)
{
	int i=0;
	int j=0;
	for (i=0; i<nRow; i++)
		for (j=0; j<nCol; j++)
			A[i*nCol + j] *= k;
}

void LAMBDA_MO_MatrixAmplify1(double A[],double k[],int nRow, int nCol)
{
	int i=0;
	int j=0;
	for (i=0; i<nRow; i++)
		for (j=0; j<nCol; j++)
			A[i*nCol + j] *= k[i*nCol + j];
}

void LAMBDA_Matrix_Eye(double A[],int nRow)
{
	int i=0,j=0;
	for (i=0; i<nRow; i++)
	{
		for (j=0; j<nRow; j++)
		{
			if(i==j)
				A[i*nRow + j] = 1;
			else
				A[i*nRow + j] = 0;
		}
	}	
}



