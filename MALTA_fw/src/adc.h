

#ifndef SRC_ADC_H_


#include "hw_cfg.h"
#include <stdint.h>
#include <stdio.h>
#include "xil_printf.h"
#include "pwm.h"
#include "math.h"
#include "filter_malta.h"
#include "malta_parameters.h"

#define M_LP_PI 0.004774648292757

// global variables
extern double current_offset[24];
extern double current_offset1[24];
extern double dc_voltage_offset;
extern double curr_meas_const[24];

extern double volatile current1;
extern double volatile current2;
extern double volatile hb_current_avg;
extern uint32_t volatile k_index;



// functions
void adc_disable(void);
uint32_t measure_dc_raw(void);
double measure_dc_with_offset(void);
void measure_dc_offset(void);
double measure_dc(void);
uint32_t measure_current_raw(uint32_t hb_number);
void measure_current_raw_arr(uint32_t *current_raw_arr, uint32_t first_hb, uint32_t N_hb);
void measure_current_offset(void);
void measure_current_offset1(void);
double measure_current_with_offset(uint32_t hb_number);
double measure_current(uint32_t hb_number);
double measure_current1(uint32_t hb_number);
void measure_current_arr(double *current_arr, uint32_t first_hb, uint32_t N_hb);
void test_shunt(void);
// Position ADCs
uint32_t measure_pos1s1_raw(void);
uint32_t measure_pos2s1_raw(void);
int32_t measure_pos3s1_raw(void);
uint32_t measure_pos1s2_raw(void);
uint32_t measure_pos2s2_raw(void);
int32_t measure_pos3s2_raw(void);
double measure_z(void);
double measure_z_method1(void);
double measure_z_method2(void);
double measure_theta_M2(void);
double measure_z_with_flux_V9(void);
void read_flux_V9(mat_double *flux_vec);

double measure_m1_y(void);
double measure_m1_x(void);
double measure_m2_y(void);
double measure_m2_x(void);


#endif /* SRC_ADC_H_ */
