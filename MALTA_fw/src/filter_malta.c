/*
 * filter_malta.c
 *
 *  Created on: 14.05.2018
 *      Author: fabiand
 */

#include "filter_malta.h"

void init_filter(filter_data_type *filter,int data_size,float coef_A,float coef_B)
{
	Init_Matrix(&(filter->last_in_vec),data_size,1);
	Init_Matrix(&(filter->last_out_vec),data_size,1);

	filter->coef_A = coef_A;
	filter->coef_B = coef_B;

}


void first_ord_filter(mat_double *data_in, filter_data_type *filter)
{
	// Filter operation
	//u[n]+u[n-1]
	matadd(data_in,&(filter->last_in_vec),&(filter->last_in_vec));

	//a*(u[n]+u[n-1])
	matmul_val(&(filter->last_in_vec),filter->coef_A,&(filter->last_in_vec));

	//b*y[n-1]
	matmul_val(&(filter->last_out_vec),filter->coef_B,&(filter->last_out_vec));

	 //a*(u[n]+u[n-1])+b*y[n-1]
	matadd(&(filter->last_in_vec),&(filter->last_out_vec),&(filter->last_out_vec));

	//Update auxiliary variables
	mat_copy(&(filter->last_in_vec),data_in); //Filter input
	//Filter output is already updated
}

void first_ord_filter_val(double data_in, filter_data_type *filter)
{
	// Filter operation
	//u[n]+u[n-1]
	filter->last_in_vec.data[0] += data_in;

	//a*(u[n]+u[n-1])
	filter->last_in_vec.data[0] *= filter->coef_A;

	//b*y[n-1]
	filter->last_out_vec.data[0] *= filter->coef_B;

	 //a*(u[n]+u[n-1])+b*y[n-1]
	filter->last_out_vec.data[0] +=filter->last_in_vec.data[0];

	//Update auxiliary variables
	filter->last_in_vec.data[0] = data_in;
	//Filter output is already updated
}

void init_medfilt_3 (med_filter_data_type *filter)
{
   for(int i=0;i<3;i++)
   {filter->data[i]=0.0;}
   filter->ptr = 0;
}

double medfilt_3(double data_in, med_filter_data_type *filter)
{

 filter->data[filter->ptr] = data_in;  //Adds new data
 filter->ptr++;           //Increments writing pointer
 if (filter->ptr>=3)
 {filter->ptr=0;}

 double middle;
 double a = filter->data[0];
 double b = filter->data[1];
 double c = filter->data[2];

 if ((a <= b) && (a <= c))
 {
   middle = (b <= c) ? b : c;
 }
 else if ((b <= a) && (b <= c))
 {
   middle = (a <= c) ? a : c;
 }
 else
 {
   middle = (a <= b) ? a : b;
 }
 return middle;
}


