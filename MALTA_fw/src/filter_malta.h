/*
 * filter_malta.h
 *
 *  Created on: 14.05.2018
 *      Author: fabiand
 */

#ifndef SRC_FILTER_MALTA_H_
#define SRC_FILTER_MALTA_H_

#include "matrix_operations.h"

//Filter coefficients for the low pass filter

// Cutoff 10kHz @22.22kHz Sampling
#define LP_FILTER_A_10kHz 0.863271264002681
#define LP_FILTER_B_10kHz -0.726542528005361

// Cutoff 8kHz @22.22kHz Sampling
#define LP_FILTER_A_8kHz 0.680011076547878
#define LP_FILTER_B_8kHz -0.360022153095757

//Filter coefficients for the low pass filter
// Cutoff 5kHz @22.22kHz Sampling
#define LP_FILTER_A_5kHz 0.460649146587691
#define LP_FILTER_B_5kHz 0.0787017068246183

// Cutoff 2kHz @ 25kHz Sampling
#define LP_FILTER_A_2kHz 0.204300824300264
#define LP_FILTER_B_2kHz 0.591398351399471

// Cutoff 1kHz @ 22.22kHz Sampling
#define LP_FILTER_A_1kHz 0.124589380980618
#define LP_FILTER_B_1kHz 0.750821238038765


// Cutoff 500Hz @ 20kHz Sampling
#define LP_FILTER_A_500Hz_20kHz 0.0729596572682667
#define LP_FILTER_B_500Hz_20kHz 0.854080685463467

// Cutoff 300Hz @ 20kHz Sampling
#define LP_FILTER_A_300Hz_20kHz 0.045035005911131
#define LP_FILTER_B_300Hz_20kHz 0.909929988177738

// Cutoff 350Hz @ 20kHz Sampling
#define LP_FILTER_A_350Hz_20kHz 0.052162645814764
#define LP_FILTER_B_350Hz_20kHz 0.895674708370473


// Cutoff 2kHz @ 20kHz Sampling
#define LP_FILTER_A_2kHz_20kHz 0.245237275252786
#define LP_FILTER_B_2kHz_20kHz 0.509525449494429

// Cutoff 30Hz @ 20kHz Sampling
#define LP_FILTER_A_30Hz_20kHz 0.004690321081766
#define LP_FILTER_B_30Hz_20kHz 0.990619357836468

// Cutoff 4kHz @ 20kHz Sampling
#define LP_FILTER_A_4kHz_20kHz 0.420807779837732
#define LP_FILTER_B_4kHz_20kHz 0.158384440324536

// Cutoff 150Hz @ 20kHz Sampling
#define LP_FILTER_A_150Hz_20kHz 0.023023722046118
#define LP_FILTER_B_150Hz_20kHz 0.953952555907763

// Cutoff 100Hz @ 20kHz Sampling
#define LP_FILTER_A_100Hz_20kHz 0.0154662914031034
#define LP_FILTER_B_100Hz_20kHz 0.969067417193793


// Cutoff 500Hz @ 5kHz Sampling
#define LP_FILTER_A_500Hz_5kHz 0.245237275252786
#define LP_FILTER_B_500Hz_5kHz 0.509525449494429




//Data Type Definition
typedef struct
{
  mat_double last_in_vec;
  mat_double last_out_vec;

  double coef_A;
  double coef_B;
}filter_data_type;

//Data Type Definition
typedef struct
{
  double data[3];
  int ptr;
}med_filter_data_type;


void init_filter(filter_data_type *filter,int data_size,float coef_A,float coef_B);
void first_ord_filter(mat_double *data, filter_data_type *filter);   //First order filter computation
void first_ord_filter_val(double data_in, filter_data_type *filter);

void init_medfilt_3 (med_filter_data_type *filter);
double medfilt_3(double data_in, med_filter_data_type *filter);



#endif /* SRC_FILTER_MALTA_H_ */
