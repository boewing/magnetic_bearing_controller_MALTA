/*
 * iir.c
 *
 *  Created on: 25 May 2018
 *      Author: smiric
 */


#include "iir.h"

void irr_write_coeff(const int32_t irr_coeff_sos[][5], int32_t outgain, uint32_t numsec);
void strobe_irr_coeff_valid(void);

/*
* Coefficients for the IIR filter
* filter1
*/
/*
#define IIR_NUMSEC  3
static const int32_t IIR_Coeff[3][5] = {
        {32768,65536,32768,47974,23614},
        {24421,48843,24421,39131,13221},
        {24646,24646,0,18278,0}};
static const int32_t IIR_OutGain = 23542;
*/

/*
* Coefficients for the IIR filter
* filter2
*/
/*#define IIR_NUMSEC  4
static const int32_t IIR_Coeff[4][5] = {
        {32768,-52573,32768,-58369,29280},
        {11817,-17371,11817,-52275,23090},
        {4400,-4315,4400,-46991,17901},
        {1578,1484,1578,-43638,14667}};
static const int32_t IIR_OutGain = 3572;*/
//***************** 300Hz Filter ********************
/*
* Coefficients for the IIR filter
*/
/*#define IIR_NUMSEC  1
static const int32_t IIR_Coeff[1][5] = {
        {334,668,334,-64663,31906}};
static const int32_t IIR_OutGain = 282;*/

//*************** No Filter Dynamics ***************
/*
* Coefficients for the IIR filter
*/

#define IIR_NUMSEC  1
static const int32_t IIR_Coeff[1][5] = {
        {32768,0,0,0,0}};
static const int32_t IIR_OutGain = 32768;



void set_up_iir(void){
	// write the coeff of the iir filter
	irr_write_coeff(IIR_Coeff, (int32_t)IIR_OutGain, (uint32_t)IIR_NUMSEC);
}

/*
 * Strobe function. IIR filter implementation read values when you strobe.
 */
void strobe_irr_coeff_valid(void){
	volatile uint32_t* const cfg_register = (void*)IRR_COEFF_VALID_ADDR;
	// strobe the register address
	// set the bit
	*cfg_register = (uint32_t)1;
	// reset the bit
	*cfg_register = (uint32_t)0;
}


/*
 * This function writes iir filter coeff to the sos of the filter.
 */
void irr_write_coeff(const int32_t irr_coeff_sos[][5], int32_t outgain, uint32_t numsec){
	volatile uint32_t* const axi_irr_sect = (void*)IRR_SECT_ADDR;
	volatile uint32_t* const axi_irr_coeff = (void*)IRR_COEFF_ADDR;
	volatile int32_t* const axi_irr_data = (void*)IRR_DAT_ADDR;
	uint32_t active_section;
	uint32_t active_coeff;

	// iterate through the sections
	for (uint32_t i = 0; i<numsec; i++){
		active_section = i;
		//printf("Write Section\n");
		// iterate through the coefficients
		for (uint32_t k=0; k<5; k++){
			active_coeff = k;

			// write to AXI section address
			*axi_irr_sect = active_section;
			// write to AXI coeff address
			*axi_irr_coeff = active_coeff;
			// write the data/value of the coeff
			*axi_irr_data = irr_coeff_sos[i][k];
			// write the coeff
			strobe_irr_coeff_valid();
			//printf("Write Coef\n");
		}
	}
	// write to AXI section address
	*axi_irr_sect = numsec;
	// write to AXI coeff address
	*axi_irr_coeff = (uint32_t)6;
	// write the data/value of the coeff
	*axi_irr_data = outgain;
	// write the coeff
	strobe_irr_coeff_valid();
}








