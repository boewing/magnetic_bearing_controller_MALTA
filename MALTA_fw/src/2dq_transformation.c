/*
 * 2dq_transformation.c
 *
 *  Created on: 18.04.2018
 *      Author: fabiand
 */


#include "2dq_transformation.h"


//************ Constant Defines ****************//

const double dq_BearingMat[3][2] = {{0,0},{0, M_2PI_3},{0, M_4PI_3}};
const double dq_BearingMat_T[2][3] = {{0,0,0},{0, M_2PI_3,M_4PI_3}};
const double dq_BearingMat_Scale[2][2] = {{1,0},{0,M_2_3}};
const double dq_BearingVec_Scale[2][1] = {{1},{M_2_3}};
const double dq_BearingMat_Const = M_1_sqrt3;
const double dq_DrivingMat[2][3] = {{0,M_2PI_3,M_4PI_3},{M_PI_2,M_7PI_6,M_11PI_6 }};
const double dq_DrivingMat_T[3][2] = {{0,M_PI_2},{M_2PI_3,M_7PI_6},{M_4PI_3,M_11PI_6}};
const double dq_DrivingMat_Scale[2][2] = {{M_2_3,0},{0,M_2_3}};
const double dq_DrivingVec_Scale[2][1] = {{M_2_3},{M_2_3}};




void init_2Ddq_transform(dq_trans_data *dq_data)
{


	set_mat(&dq_BearingMat[0][0], MatSize_Row(dq_BearingMat), MatSize_Col(dq_BearingMat),&(dq_data->BearingMat));
	set_mat(&dq_BearingMat_Scale[0][0], MatSize_Row(dq_BearingMat_Scale), MatSize_Col(dq_BearingMat_Scale),&(dq_data->BearingMat_Scale));
	set_mat(&dq_BearingVec_Scale[0][0], MatSize_Row(dq_BearingVec_Scale), MatSize_Col(dq_BearingVec_Scale),&(dq_data->BearingVec_Scale));

	set_mat(&dq_BearingMat_T[0][0], MatSize_Row(dq_BearingMat_T), MatSize_Col(dq_BearingMat_T),&(dq_data->BearingMat_T));
	dq_data->BearingMatConst = dq_BearingMat_Const;

	set_mat(&dq_DrivingMat[0][0], MatSize_Row(dq_DrivingMat), MatSize_Col(dq_DrivingMat),&(dq_data->DrivingMat));
	set_mat(&dq_DrivingMat_Scale[0][0], MatSize_Row(dq_DrivingMat_Scale), MatSize_Col(dq_DrivingMat_Scale),&(dq_data->DrivingMat_Scale));
	set_mat(&dq_DrivingVec_Scale[0][0], MatSize_Row(dq_DrivingVec_Scale), MatSize_Col(dq_DrivingVec_Scale),&(dq_data->DrivingVec_Scale));

	set_mat(&dq_DrivingMat_T[0][0], MatSize_Row(dq_DrivingMat_T), MatSize_Col(dq_DrivingMat_T),&(dq_data->DrivingMat_T));

	Init_Matrix(&(dq_data->TempMat1_2x3),2, 3);
	Init_Matrix(&(dq_data->TempMat1_3x2),3, 2);
	Init_Matrix(&(dq_data->TempMat2_2x3),2, 3);
    Init_Matrix(&(dq_data->TempMat2_3x2),3, 2);
    Init_Matrix(&(dq_data->TempMat_2x2),2, 2);

};


void dq2abc_2Ddq_transform(mat_double *dq,mat_double *abc, double cont_BearAng,double control_DriveAng,dq_trans_data *dq_data)
{

	// Prepare Bearing mat thb'

	matadd_val(&(dq_data->BearingMat_T),cont_BearAng,&(dq_data->TempMat1_2x3));
	mat_cos(&(dq_data->TempMat1_2x3),&(dq_data->TempMat1_2x3));  // Compute cosine of each element
	mat_rep_row_val(0, &(dq_data->TempMat1_2x3),dq_data->BearingMatConst); // Replace first row with constant value
	// Prepare Driving mat thz'
	matadd_val(&(dq_data->DrivingMat_T),control_DriveAng,&(dq_data->TempMat1_3x2));
	mat_cos(&(dq_data->TempMat1_3x2),&(dq_data->TempMat1_3x2));  // Compute cosine of each element

	// Compute 2D dq to abc transformation
	//temp_res = thz'*dq
	matmul(&(dq_data->TempMat1_3x2),dq,&(dq_data->TempMat2_3x2));
	//abc = temp_res*thb'
	matmul(&(dq_data->TempMat2_3x2),&(dq_data->TempMat1_2x3),abc);


};

void abc2dq_2Ddq_transform(mat_double *dq,mat_double *abc, double cont_BearAng,double control_DriveAng,dq_trans_data *dq_data)
{

	// Prepare Bearing mat thb
	matadd_val(&(dq_data->BearingMat),cont_BearAng,&(dq_data->TempMat1_3x2));
	mat_cos(&(dq_data->TempMat1_3x2),&(dq_data->TempMat1_3x2));  // Compute cosine of each element | ET: 1.5us
	mat_rep_col_val(0, &(dq_data->TempMat1_3x2),dq_data->BearingMatConst); // Replace first row with constant value
	// Prepare Driving mat thz
	matadd_val(&(dq_data->DrivingMat),control_DriveAng,&(dq_data->TempMat1_2x3));
	mat_cos(&(dq_data->TempMat1_2x3),&(dq_data->TempMat1_2x3));  // Compute cosine of each element  | ET: 1.7us


	// Compute 2D abc to dq transformation | ET: 2.25us with matmul | ET: 1.8us with matmul_row,matmul_col for the scaling part
	//temp_d = thz*abc
	matmul(&(dq_data->TempMat1_2x3),abc,&(dq_data->TempMat2_2x3));
	//dq = temp_d*thb
	matmul(&(dq_data->TempMat2_2x3),&(dq_data->TempMat1_3x2),dq);
	//dq = D_scale*dq
	//matmul(&(dq_data->DrivingMat_Scale),dq,&(dq_data->TempMat_2x2));
	matmul_row(dq,&(dq_data->DrivingVec_Scale),dq);
	//dq = dq*B_scale
	//matmul(&(dq_data->TempMat_2x2),&(dq_data->BearingMat_Scale),dq);
	matmul_col(dq,&(dq_data->BearingVec_Scale),dq);


};


