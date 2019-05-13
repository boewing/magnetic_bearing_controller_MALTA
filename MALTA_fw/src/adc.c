
#include "adc.h"

//Array of current offsets for HB 1...24; i1_off = current_offset[1]
double current_offset[24] = {0.0};
double current_offset1[24] = {0.0};
double dc_voltage_offset = 0.0;
double curr_meas_const[24] = {-0.00535};

double volatile current1 = 0.0;
double volatile current2 = 0.0;
double volatile hb_current_avg = 0.0;
uint32_t volatile k_index = 0;




// do not use this fun, there is no disable
void adc_disable(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_CONF_ADDR;
	uint32_t cfg = *cfg_register; //Read the register into a local variable

	cfg &= (uint32_t)~(1<<0); //Set bit 0
	//cfg = cfg & (uint32_t~(1<<0);
	//cfg = cfg & (uint32_t)(11111111111111111111111111111110);
	*cfg_register = cfg; //write the modified value back to the HW
}


// Read DC link raw
uint32_t measure_dc_raw(void)
{
	volatile uint32_t* const cfg_register = (void*)ADCCTRL_DC_ADDR;
	uint32_t val;
	val = *cfg_register;
	return val;
}

double measure_dc_with_offset(void)
{
	return ( ((double)measure_dc_raw()) / 1000.0 * 15.75 );
}

void measure_dc_offset(void)
{
	double cumulate_dc_voltage = 0.0; //Cumulative voltage sum
	//Collect samples
	for(uint32_t m = 1; m <= 500; m += 1){
		cumulate_dc_voltage += measure_dc_with_offset();
	}
	dc_voltage_offset = cumulate_dc_voltage / 500.0;
}

double measure_dc(void)
{
	return ( measure_dc_with_offset() - dc_voltage_offset );
}


//Measure current raw
uint32_t measure_current_raw(uint32_t hb_number)
{
	volatile uint32_t* const dat_register = (void*)(ADCCTRL_I1_ADDR + (hb_number - 1)*4);
	uint32_t val;
	val = *dat_register;
	return val;
}

//Measure current raw
void measure_current_raw_arr(uint32_t *current_raw_arr, uint32_t first_hb, uint32_t N_hb)
{
	uint32_t* dat_register = (uint32_t*)(ADCCTRL_I1_ADDR);

	memcpy(current_raw_arr,(dat_register+(first_hb - 1)),sizeof(dat_register[0])*N_hb);

}


//Measure current offset
void measure_current_offset1(void)
{
	uint32_t hb = 0; //HB count
	double cumulate_current = 0.0; //Cumulative sum

	//Set duty rations of all HB to zero
	for(hb = 1; hb <= 24; hb += 1){
		set_modindex(0.0, hb);
	}
	//Collect for all 24 HBs
	for(hb = 1; hb <= 24; hb += 1){
		//Go through 300 samples
		for(uint32_t m = 1; m <= 300; m += 1){
			cumulate_current += (double)((int)measure_current_raw(hb));
		}
		cumulate_current = cumulate_current / 300.0;
		current_offset1[hb-1] = cumulate_current;
		cumulate_current = 0.0;
	}
}

// Measure current in Amps
double measure_current1(uint32_t hb_number)
{
	double scale_constant = curr_meas_const[hb_number - 1];
	double current_value = 0.0;
	//
	current_value = (double)( (int)measure_current_raw(hb_number) );
	current_value = current_value - current_offset1[hb_number-1];
	current_value = current_value * scale_constant;
	return current_value;
}

// Measure current in Amps
void measure_current_arr(double *current_arr, uint32_t first_hb, uint32_t N_hb)
{
	static uint32_t current_raw_arr[24];
	measure_current_raw_arr(current_raw_arr,first_hb,N_hb);
	//
	for (int i = 0;i<N_hb;i++)
	{
		current_arr[i] = ((double)current_raw_arr[i]-current_offset1[i])*curr_meas_const[i];
	}

}



void test_shunt(void)
{
	//static uint32_t k = 0;
	static double current_array[100] = {0.0};
	double cumulate_var = 0.0;
	if (k_index < 100) {
		current_array[k_index] = current1;//measure_current1(hb_number);
	} else {
		k_index = 0;
	}
	k_index += 1;
	// average of the array
	for (uint32_t i = 0; i < 100; i += 1){
		cumulate_var += current_array[i];
	}
	hb_current_avg = cumulate_var / 100.0;
}





//Measure current in Amps, no offset compensated
double measure_current_with_offset(uint32_t hb_number)
{
	return -(double)(((int)measure_current_raw(hb_number)-2500) * 0.005);
}

//Measure the ADC offsets
void measure_current_offset(void)
{
	uint32_t hb = 0; //HB count
	double cumulate_current = 0.0; //Cumulative sum

	//Set duty rations of all HB to zero
	for(hb = 1; hb <= 24; hb += 1){
		set_modindex(0.0, hb);
	}
	//Collect for all 24 HBs
	for(hb = 1; hb <= 24; hb += 1){
		//Go through 300 samples
		for(uint32_t m = 1; m <= 300; m += 1){
			cumulate_current += measure_current_with_offset(hb);
		}
		cumulate_current = cumulate_current / 300.0;
		current_offset[hb-1] = cumulate_current;
		cumulate_current = 0.0;
	}
}

double measure_current(uint32_t hb_number){
	return (measure_current_with_offset(hb_number) - current_offset[hb_number-1]);
}



// Read position
//Measure current raw

// position Y, sensor 1
uint32_t measure_pos1s1_raw(void)
{
       volatile uint32_t* const dat_register = (void*)(ADCCTRL_1S1_ADDR);
       uint32_t val;
       val = *dat_register;

       //SHUTDOWN on sensor error
       if(val == 0){
    	   cstop();
    	   printf("stopped on y1 sensor error\n");
       }

       return val;
}

// position X, sensor 1
uint32_t measure_pos2s1_raw(void)
{
       volatile uint32_t* const dat_register = (void*)(ADCCTRL_2S1_ADDR);
       uint32_t val;
       val = *dat_register;

       //SHUTDOWN on sensor error
       if(val == 0){
           	   cstop();
           	   printf("stopped on x1 sensor error\n");
       }

       return val;
}

// position Z, sensor 1
int32_t measure_pos3s1_raw(void)
{
       volatile int32_t* const dat_register = (void*)(ADCCTRL_3S1_ADDR);
       int32_t val;
       //int sign = 0;
       val = (int32_t) *dat_register;
      /* //printf("R val %ld \n",val);
       sign = (val& 0x00008000)==0;
       //printf("sidn %d \n",sign);
       val &= ~(0x00008000);
       //printf("S rem %ld \n",val);
       if (sign==0)
       {sign = -1;}
       val *= sign;
       //printf("S add %ld \n",val);*/

       //SHUTDOWN on sensor error
       if(val == 0){
           	   cstop();
           	   printf("stopped on z1 sensor error\n");
       }

       return val;
}


// position Y, sensor 2
uint32_t measure_pos1s2_raw(void)
{
       volatile uint32_t* const dat_register = (void*)(ADCCTRL_1S2_ADDR);
       uint32_t val;
       val = *dat_register;

       //SHUTDOWN on sensor error
       if(val == 0){
           	   cstop();
           	   printf("stopped on y2 sensor error\n");
       }

       return val;
}

// position X, sensor 2
uint32_t measure_pos2s2_raw(void)
{
       volatile uint32_t* const dat_register = (void*)(ADCCTRL_2S2_ADDR);
       uint32_t val;
       val = *dat_register;

       //SHUTDOWN on sensor error
       if(val == 0){
           	   cstop();
           	   printf("stopped on x2 sensor error\n");
       }

       return val;
}

// position Z, sensor 2
int32_t measure_pos3s2_raw(void)
{
       volatile int32_t* const dat_register = (void*)(ADCCTRL_3S2_ADDR);
       int32_t val;
       val = (int32_t) *dat_register;

       //SHUTDOWN on sensor error
       if(val == 0){
           	   cstop();
           	   printf("stopped on z2 sensor error\n");
       }

       return val;
}


double measure_z(void)
{

	//return -(measure_theta_M2()*M_LP_PI);  // z measurement
  return measure_z_with_flux_V9();
}

void read_flux_V9(mat_double *flux_vec)
{
	flux_vec->data[0] = (double)(measure_pos3s2_raw());
	flux_vec->data[1] = (double)(measure_pos3s1_raw());
}

double measure_z_with_flux_V9(void)
{

	// Reconstruction for steel extension rods
	//int N_Poly = 4;
	//Reconstruction for 3D Printed extension rods
	int N_Poly = 3;

       double x_temp[N_Poly];
       double y_temp[N_Poly];

       x_temp[0] = (double)(measure_pos3s2_raw());
       y_temp[0] = (double)(measure_pos3s1_raw());

       // x normalisation: mean = 1908; std = 867.8;
       // y normalisation: mean = 1775; std = 956.4;

       // Reconstruction for steel extension rods
       //x_temp[0] = ( x_temp[0] - 15900.0 ) / 7583.0;
       //y_temp[0] = ( y_temp[0] - 15840.0 ) / 7709.0;
       //int N_Poly = 4;

       //Reconstruction for 3D Printed extension rods
       x_temp[0] = ( x_temp[0] - 14680.0 ) / 7796.0;
       y_temp[0] = ( y_temp[0] - 16220.0 ) / 6439.0;


       //Compute powers for x
       for (int i=1;i<N_Poly;i++)
       {
    	   x_temp[i] = x_temp[0]*x_temp[i-1];
       }

       //Compute powers for x
          for (int i=1;i<N_Poly;i++)
          {
       	   y_temp[i] = y_temp[0]*y_temp[i-1];
          }




       // z position in mm
       double z;
       // Reconstruction for steel extension rods
       /*z = 31.1299 + 16.0972*x_temp[0] - 9.4814*y_temp[0] \
                   + 1.4141*x_temp[1] - 19.2734*x_temp[0]*y_temp[0] + 7.2485*y_temp[1] \
                   - 5.6149*x_temp[2] + 13.9207*x_temp[1]*y_temp[0] + 0.7460*x_temp[0]*y_temp[1] + 2.6229*y_temp[2] \
                   - 2.1281*x_temp[3] + 7.6155*x_temp[2]*y_temp[0] - 11.5303*x_temp[1]*y_temp[1] + 5.1279*x_temp[0]*y_temp[2] - 1.0225*y_temp[3];
*/
       //Reconstruction for 3D Printed extension rods
       z = 20.5557 + 7.1453*x_temp[0] - 3.9629*y_temp[0] \
                          + 7.2253*x_temp[1] - 2.7375*x_temp[0]*y_temp[0] + 10.1524*y_temp[1] \
                          + 0.7049*x_temp[2] + 1.8234*x_temp[1]*y_temp[0] - 4.3258*x_temp[0]*y_temp[1] + 2.3428*y_temp[2] ;

       return z*MM_TO_M;
}

double measure_z_method1(void)
{
       double x_temp[5];
       double y_temp[5];

       x_temp[0] = (double)((int)measure_pos3s1_raw());
       y_temp[0] = (double)((int)measure_pos3s2_raw());

       // x normalisation: mean = 1908; std = 867.8;
       // y normalisation: mean = 1775; std = 956.4;
       x_temp[0] = ( x_temp[0] - 1908.0 ) / 867.8;
       y_temp[0] = ( y_temp[0] - 1775.0 ) / 956.4;

       //Compute powers for x
       for (int i=1;i<5;i++)
       {
    	   x_temp[i] = x_temp[0]*x_temp[i-1];
       }

       //Compute powers for x
          for (int i=1;i<5;i++)
          {
       	   y_temp[i] = y_temp[0]*y_temp[i-1];
          }




       // z position in mm
       double z;

       // Reconstruction for steel extension rods
       z = 11.21 - 15.4*x_temp[0] + 2.096*y_temp[0] + \
                     26.79*x_temp[1] - 0.4647*x_temp[0]*y_temp[0] + 9.836*y_temp[1] + \
                     9.042*x_temp[2] - 8.766*x_temp[1]*y_temp[0] + 12.65*x_temp[0]*y_temp[1] + 4.61*y_temp[2] - \
                     6.593*x_temp[3] + 5.215*x_temp[2]*y_temp[0] - 6.138*x_temp[1]*y_temp[1] + 0.004965*x_temp[0]*y_temp[2] - 0.8649*y_temp[3] - \
                     2.562*x_temp[4] + 2.868*x_temp[3]*y_temp[0] - 7.218*x_temp[2]*y_temp[1] + 1.287*x_temp[1]*y_temp[2] - 2.176*x_temp[0]*y_temp[3] - 0.615*y_temp[4];


       return z*MM_TO_M;
}

/*double measure_theta_M2(void)
{
       double x_temp[3];
       double y_temp[3];

       static int init = 1;
       static filter_data_type x_filter;
       static filter_data_type y_filter;

       if (init)
       {
    	   init_filter(&x_filter,1,LP_FILTER_A_500Hz,LP_FILTER_B_500Hz);
    	   init_filter(&y_filter,1,LP_FILTER_A_500Hz,LP_FILTER_B_500Hz);

    	   init = 0;
       }

       first_ord_filter_val((double)(measure_pos3s2_raw()), &x_filter);
       first_ord_filter_val((double)(measure_pos3s1_raw()), &y_filter);

       //x_temp[0] = x_filter.last_out_vec.data[0];
       //y_temp[0] = y_filter.last_out_vec.data[0];

       x_temp[0] = x_filter.last_in_vec.data[0];
       y_temp[0] = y_filter.last_in_vec.data[0];

       // x normalisation: mean = 1908; std = 867.8;
       // y normalisation: mean = 1775; std = 956.4;
       x_temp[0] = ( x_temp[0] - 1942.0) / 878.6 / 8;  //IIR Filter Gain of 8!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
       y_temp[0] = ( y_temp[0] - 1997 ) / 901 / 8;

       //Compute powers for x
       for (int i=1;i<3;i++)
       {
    	   x_temp[i] = x_temp[0]*x_temp[i-1];
       }

       //Compute powers for x
          for (int i=1;i<3;i++)
          {
       	   y_temp[i] = y_temp[0]*y_temp[i-1];
          }




       // z position in mm
       double th;

       th = -0.7766- 1.877*x_temp[0] + 1.875*y_temp[0] + \
                     1.406*x_temp[1] - 0.1418*x_temp[0]*y_temp[0] + 0.3011*y_temp[1] + \
					0.6371*x_temp[2] - 1.161*x_temp[1]*y_temp[0] + 0.8224*x_temp[0]*y_temp[1] - 0.1204*y_temp[2];

       return th;
}*/


double measure_z_method2(void)
{
       double z_temp;


       z_temp = atan2((double)(measure_pos3s2_raw()),(double)(measure_pos3s1_raw()))*I_M_PI*0.015;

       // z normalisation
       z_temp = ( z_temp - 0.00388 ) / 0.002115;


       return (0.007815*z_temp+0.03322);
}

// Measures the y position for the module 1
// Module 1 has the hardware sensor 2!
//Factor of 0.125 is the gain of the hardware filter of 8
double measure_m1_y(void)
{
	return ((double)(measure_pos1s2_raw())*0.125-POS_MEA_M1_Y0)*POS_MEA_M1_DY*MM_TO_M;

}

double measure_m1_x(void)
{
	return ((double)(measure_pos2s2_raw())*0.125-POS_MEA_M1_X0)*POS_MEA_M1_DX*MM_TO_M;

}

double measure_m2_y(void)
{
	return ((double)(measure_pos1s1_raw())*0.125-POS_MEA_M2_Y0)*POS_MEA_M2_DY*MM_TO_M;

}

double measure_m2_x(void)
{
	return ((double)(measure_pos2s1_raw())*0.125-POS_MEA_M2_X0)*POS_MEA_M2_DX*MM_TO_M;

}

/*
// Read current values raw
uint32_t read_i1_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I1_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}


uint32_t read_i2_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I2_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i3_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I3_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i4_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I4_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i5_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I5_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i6_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I6_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i7_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I7_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i8_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I8_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i9_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I9_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i10_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I10_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i11_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I11_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i12_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I12_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i13_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I13_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i14_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I14_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i15_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I15_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i16_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I16_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i17_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I17_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i18_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I18_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i19_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I19_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i20_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I20_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i21_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I21_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i22_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I22_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i23_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I23_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}

uint32_t read_i24_raw(void)
{
	volatile uint32_t* const dat_register = (void*)ADCCTRL_I24_ADDR;
	uint32_t val;
	val = *dat_register;
	return val;
}
*/








