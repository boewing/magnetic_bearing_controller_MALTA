/*
 * 2dq_transformation.h
 *
 *  Created on: 18.04.2018
 *      Author: fabiand
 */

#ifndef SRC_2DQ_TRANSFORMATION_H_
#define SRC_2DQ_TRANSFORMATION_H_

#include "matrix_operations.h"
#include "math.h"
#include <stdio.h>


#define M_2PI_3		2.094395102393195
#define M_4PI_3		4.188790204786391
#define M_7PI_6		3.665191429188092
#define M_11PI_6	5.759586531581287

#define M_1_sqrt3   0.577350269189626
#define M_2_3       0.666666666666667


//*********** Data Structures ***************//
typedef struct
{
  mat_double BearingMat;
  mat_double BearingMat_T;
  mat_double BearingMat_Scale;
  mat_double BearingVec_Scale;
  double BearingMatConst;
  mat_double DrivingMat;
  mat_double DrivingMat_T;
  mat_double DrivingMat_Scale;
  mat_double DrivingVec_Scale;
  mat_double TempMat1_2x3;
  mat_double TempMat1_3x2;
  mat_double TempMat2_2x3;
  mat_double TempMat2_3x2;
  mat_double TempMat_2x2;

} dq_trans_data;



//************* Function Prototypes **************//
void init_2Ddq_transform(dq_trans_data *dq_data);
void dq2abc_2Ddq_transform(mat_double *dq,mat_double *abc, double cont_BearAng,double control_DriveAng,dq_trans_data *dq_data);
void abc2dq_2Ddq_transform(mat_double *abc,mat_double *dq, double cont_BearAng,double control_DriveAng,dq_trans_data *dq_data);




#endif /* SRC_2DQ_TRANSFORMATION_H_ */
