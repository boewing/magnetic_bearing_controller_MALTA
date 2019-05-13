/*
 * Matrix_operations.h
 *
 *  Created on: 16.04.2018
 *      Author: fabiand
 */

#ifndef SRC_MATRIX_OPERATIONS_H_
#define SRC_MATRIX_OPERATIONS_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#define M_2PI		6.283185307179586

//******************** Macros for Matrix Dimension extraction *****************//
#define MatSize_Row(arr) ((int) (sizeof (arr) / sizeof (arr)[0]))
#define MatSize_Col(arr) ((int) (sizeof (arr[0]) / sizeof (arr)[0][0]))

typedef struct
{
  int drow;
  int dcol;
  double *data;
} mat_double;

typedef struct
{
  int drow;
  int dcol;
  const double *data;
} mat_double_const;


//**************** Initialisation Functions ******************//
// Initialises a matrix and allocates memory
void Init_Matrix(mat_double *mat,int N_rows, int N_cols);
void Delete_Matrix(mat_double *mat);
// Sets a matrix to a predefined constant matrix
void set_mat(double *val,int N_rows, int N_cols,mat_double *mat);


//**************** Matrix Computations ***********************//
void matmul(mat_double *A,mat_double *B,mat_double *C);
void matmul_element(mat_double *A,mat_double *B,mat_double *C);
void matadd(mat_double *A,mat_double *B,mat_double *C);
void matsub(mat_double *A,mat_double *B,mat_double *C);
void matadd_val(mat_double *A,double B,mat_double *C);
void matmul_val(mat_double *A,double val,mat_double *C);

void matmul_row(mat_double *A,mat_double *B,mat_double *C);
void matmul_col(mat_double *A,mat_double *B,mat_double *C);

//**************** Matrix Manipulations ***********************//
// Gets the 1D index for the 2D indexes
int getLinearIndex(mat_double *mat,int row, int col) ;

void mat_clear(mat_double *mat);

void mat_copy(mat_double *target,mat_double *mat);

void vec_copy_part(mat_double *target,mat_double *mat,int start,int end);


void mat_rep_col_val(int col, mat_double *mat,double val);  //Replaces specific column with value
void mat_rep_row_val(int row, mat_double *mat,double val);  //Replaces specific row with value

void mat_cat_1dim(mat_double *mat1,mat_double *mat2,mat_double *mat_cat); //Concatenates two matrix along the first dimension


//**************** Math Matrix Computations ***********************//
void mat_cos(mat_double *mat,mat_double *res);
void mat_cos_approx(mat_double *mat,mat_double *res);

double cos_approx(double val);


// ******** Visualization Functions ***********//
void printMatDim(mat_double *mat);
void printMat(mat_double *mat);
void printMat_scale(mat_double *mat,double scale);

//int getIndex(int row, int col) { return row*NCOLS+col; }

#endif /* SRC_MATRIX_OPERATIONS_H_ */
