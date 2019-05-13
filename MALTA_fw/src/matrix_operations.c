/*
 * Matrix_operations.c
 *
 *  Created on: 16.04.2018
 *      Author: fabiand
 */


#include "matrix_operations.h"

//**************** Constants ***********************//
const double Cos_Approximation_Coef[4] = {-0.002246,1.027,-0.06993,-0.1139};


//**************** Initialization functions ***********************//
void set_mat(double *val,int N_rows, int N_cols,mat_double *mat)
{
 // Writes the values in the matrix struct
 mat->drow = N_rows;
 mat->dcol = N_cols;
 // The data is now a 1D Array
 mat->data = val;

};


void Init_Matrix(mat_double *mat,int N_rows, int N_cols)
{
	 // Writes the values in the matrix struct
	 mat->drow = N_rows;
	 mat->dcol = N_cols;
	 // Allocates memory for the matrix
	 mat->data = calloc(N_rows * N_cols , sizeof(double));
};

void Delete_Matrix(mat_double *mat)
{
  free(mat->data);

};

//**************** Matrix Computations ***********************//

// Matrix Multiplication
// C = A*B
void matmul(mat_double *A,mat_double *B,mat_double *C)
{
	int C_Lin_Index = 0;

		memset(C->data, 0, sizeof(C->data[0])*(C->dcol)*(C->drow)); //Setting all bytes to zero in IEEE 754 results in a 0.0 float.
		for(int i = 0; i < A->drow; i++)
		{
			for(int j = 0; j < B->dcol; j++)
			{
				C_Lin_Index = getLinearIndex(C,i, j);  // Compute the linear index for Matrix C
				//C->data[C_Lin_Index] = 0.0; //We must initialise to zero | memset is faster!
				for(int k = 0; k <  A->dcol; k++)
				{
					C->data[C_Lin_Index] = C->data[C_Lin_Index] + (A->data[getLinearIndex(A,i, k)] * B->data[getLinearIndex(B,k, j)]);
				}
			}
		}
};

//**** Elementwise Matrix Computations ****//
// For these operations the result matrix can also be one of the input matrix

// Elementwise Matrix Multiplication
//C = A.*B;
void matmul_element(mat_double *A,mat_double *B,mat_double *C)
{
	for (int i=0;i<A->dcol*A->drow;i++)
		  {
			  C->data[i]=A->data[i]*B->data[i];
		  }

};


//Elementwise Matrix Addition
// C = A+B
void matadd(mat_double *A,mat_double *B,mat_double *C)
{
	for (int i=0;i<A->dcol*A->drow;i++)
	  {
		  C->data[i]=A->data[i]+B->data[i];


	  }

};

//Elementwise Matrix Subtraction
// C = A-B
void matsub(mat_double *A,mat_double *B,mat_double *C)
{
	for (int i=0;i<A->dcol*A->drow;i++)
	  {
		  C->data[i]=A->data[i]-B->data[i];


	  }

};

void matadd_val(mat_double *A,double val,mat_double *C)
{
	for (int i=0;i<A->dcol*A->drow;i++)
		  {
			  C->data[i]=A->data[i]+val;


		  }
};

// C = A.*val
void matmul_val(mat_double *A,double val,mat_double *C)
{
	for (int i=0;i<A->dcol*A->drow;i++)
		  {
			  C->data[i]=A->data[i]*val;


		  }
};

//Scales each row of the matrix with the corresponding vector element
void matmul_row(mat_double *mat,mat_double *vec,mat_double *mat_scale)
{
	int vec_idx = 0;

	for (int i=0;i<mat->dcol*mat->drow;i++)
	  {
		  //Needs to be an integer division!
		  vec_idx = i/mat->dcol; //Index of which vector element should be taken, dependent on the number of columns
          mat_scale->data[i]=mat->data[i]*vec->data[vec_idx];


	  }
}

//Scales each column of the matrix with the corresponding vector element
//mat_scale = mat .* |
void matmul_col(mat_double *mat,mat_double *vec,mat_double *mat_scale)
{
	int vec_idx = 0;

	for (int i=0;i<mat->dcol*mat->drow;i++)
	  {
		  //Needs to be an integer division!
		  vec_idx = i%mat->dcol; //Index of which vector element should be taken, dependent on the number of columns
          mat_scale->data[i]=mat->data[i]*vec->data[vec_idx];


	  }
}

//**************** Matrix Manipulations ***********************//
void mat_clear(mat_double *mat)
{
	memset(mat->data, 0, sizeof(mat->data[0])*(mat->dcol)*(mat->drow)); //Setting all bytes to zero in IEEE 754 results in a 0.0 float.
}

void mat_copy(mat_double *target,mat_double *mat)
{
	memcpy(target->data, mat->data, sizeof(mat->data[0])*(mat->drow)*(mat->dcol)); //Copy data
}

void vec_copy_part(mat_double *target,mat_double *mat,int start,int end)
{
	memcpy(target->data, (mat->data+start), sizeof(mat->data[0])*(end-start+1)); //Copy data
}

int getLinearIndex(mat_double *mat,int row, int col)
{
	return row*mat->dcol+col;
};


void mat_rep_col_val(int col, mat_double *mat,double val)
{
   for (int i=0;i<mat->drow;i++)
   {
	mat->data[getLinearIndex(mat,i,col)]=val;

   }
};

void mat_rep_row_val(int row, mat_double *mat,double val)
{
   for (int i=0;i<mat->dcol;i++)
   {
	mat->data[getLinearIndex(mat,row,i)]=val;

   }
};

//Concatenates two matrix along the first dimension
// C = [A;B]
void mat_cat_1dim(mat_double *A,mat_double *B,mat_double *C)
{
	memcpy(C->data, A->data, sizeof(A->data[0])*A->drow*A->dcol); //Copy data
	memcpy(C->data+A->drow*A->dcol, B->data, sizeof(B->data[0])*B->drow*B->dcol); //Copy data


};

//****************Math Matrix Computations ***********************//

void mat_cos(mat_double *A,mat_double *B)
{
  for (int i=0;i<A->dcol*A->drow;i++)
  {
	  B->data[i]=cos(A->data[i]);


  }
};

void mat_cos_approx(mat_double *A,mat_double *B)
{
  for (int i=0;i<A->dcol*A->drow;i++)
  {
	  B->data[i]=cos_approx(A->data[i]);


  }
};

//**************** Approximations for computation speedup ***********************//

double cos_approx(double val)
{
  int sign = 1;
  double result = Cos_Approximation_Coef[3];
  val += M_PI_2; //Shift value by pi halfe for cos appoximation
  if (val<0)
  {
	  sign = -1;
	  val *= -1;
  }


  val = fmod(val,M_2PI);

  if (val > M_PI)
  {
	  sign *= -1;
	  val = fmod(val,M_PI);
  }
  if (val>M_PI_2)
  {
	  val = M_PI-val;
  }
  //Compute polynome using horner scheme
	for (int i=2;i>=0;i--)
	{
		result = Cos_Approximation_Coef[i]+result*val;

	}
 return result*sign;
}

// ******** Visualization Functions ***********//
// Prints MAtrix dimensions
void printMatDim(mat_double *mat)
{
	printf( "MatDim: %dx%d\n", mat->drow,mat->dcol );

};


void printMat(mat_double *mat)
{
	for (int i = 0; i < mat->drow; i++)
	{
		for (int j = 0; j < mat->dcol; j++)
		{
			printf( "%.2f  ", mat->data[getLinearIndex(mat,i,j)] );
		}
		printf( "\n");
	}

};

void printMat_scale(mat_double *mat,double scale)
{
	for (int i = 0; i < mat->drow; i++)
	{
		for (int j = 0; j < mat->dcol; j++)
		{
			printf( "%.6f  ", (mat->data[getLinearIndex(mat,i,j)])*scale );
		}
		printf( "\n");
	}
}
/*static inline void CalcMatMul(const float A[KALMAN_NUMSTATES][KALMAN_NUMSTATES], const float B[KALMAN_NUMSTATES][KALMAN_NUMSTATES], float C[KALMAN_NUMSTATES][KALMAN_NUMSTATES])
{
	//We need to set the array to zero:
	//memset(C, 0, (sizeof C[0][0]) * KALMAN_NUMSTATES * KALMAN_NUMSTATES); //Setting all bytes to zero in IEEE 754 results in a 0.0 float.
	for(uint32_t i = 0; i < KALMAN_NUMSTATES; i++)
	{
		for(uint32_t j = 0; j < KALMAN_NUMSTATES; j++)
		{
			//C[i][j] = 0.0; //We must initialize to zero ==> memset above is faster.
			for(uint32_t k = 0; k < KALMAN_NUMSTATES; k++)
			{
				C[i][j] = C[i][j] + (A[i][k] * B[k][j]);
			}
		}
	}
}*/

