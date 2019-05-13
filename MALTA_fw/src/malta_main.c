/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "platform.h"
#include "xil_printf.h"
#include "hw_cfg.h"
#include "pwm.h"
#include "adc.h"
#include "cli.h"
#include "adc.h"
#include "iir.h"


#include "interrupt.h"
#include "malta_configure.h"
#include "matrix_operations.h"
#include "2dq_transformation.h"
#include "malta_statemachine.h"
#include "malta_current_control.h"
#include "malta_position_control.h"
#include "sine_lookup.h"
#include "test2.h"
#include "softstart.h"
#include "step_optimizer.h"




int main()
{
    // Event handler setup
	volatile int local_events = 0;   //Volatile is needed otherwise some events are missed
	uint32_t test = 0;

	theta = 0;

	int plot_cnt = 0;
	int plot_cnt_max = 1;
	//int plot_cnt_max = 5;

	int test_dq = 0;
	int ang = 0;

	double theta_freq_cc=1;
	double drive_cur_cc=2;

	double temp_th = 0;

	int step_count = 0;
	int step_time = 100;



	//************* Current controller setup *******************//

	init_malta_current_control(M1_CUR_CONT,1);
	init_malta_current_control(M2_CUR_CONT,13);

	ref_data_type data_M1;
	ref_data_type data_M2;
	Init_Matrix(&data_M1.currents,2,2);
	Init_Matrix(&data_M2.currents,2,2);


	// Dummy Matrix for current reference
    mat_double cur_ref;
    Init_Matrix(&cur_ref,2,2);

	//************* Reference generation for position control *******************//
	init_reference_generation();
	init_softstart(100,40000);   //Disables current controller for the first N steps, so that the filters can settle from their initialization
    //************* Position controller setup *******************//
    pos_cont_data_type Position_Controller;
    init_malta_position_control(&Position_Controller);


    mat_double pos_ref;
    Init_Matrix(&pos_ref,10,1);
    //pos_ref.data[4] = 0;

    //z->theta calibration stuff
    int ang_stepsize = 10;

    //Step limits in pi/100
    int ang_steps_max_M1 = 100;
    int ang_steps_min_M1 = -80;
    int ang_steps_max_M2 = 100;
    int ang_steps_min_M2 = -80;
    float ang_cnt = 0;
    int ang_cnt_dir = 1;

    printf("\n\n\n\n\n");
	printf("MALTA firmware is running.\n\n\n");
	printf("Starting Init.\n\n");

	init_platform();

    gic_init();
    init_first_interrupt();
    //init_second_interrupt();

    //Initialise MALTA hardware
    malta_initialise();
    // Initialise HW Recorder
    rec_write_baseaddr();

    //enable_dead_time();
    //enable_pwm();

    //*******************Variable Recorder Setup*****************************
    //If the number of recorded variables change "NUM_VAR_RECORDER" in "variable_recorder.h" needs to be changed as well
    //If variables of the current controller are required, change the readout of the in the code below (After position control)
    int N_Data_Points = 5000;

    // Measured positions
     init_var_recoder(&(Position_Controller.mea_vec.data[0]),N_Data_Points,"x1");
	 init_var_recoder(&(Position_Controller.mea_vec.data[1]),N_Data_Points,"y1");
	 init_var_recoder(&(Position_Controller.mea_vec.data[2]),N_Data_Points,"x2");
	 init_var_recoder(&(Position_Controller.mea_vec.data[3]),N_Data_Points,"y2");
	 init_var_recoder(&(Position_Controller.mea_vec.data[4]),N_Data_Points,"z");
	 init_var_recoder(&(Position_Controller.z_laser),N_Data_Points, "z_laser");
//	 init_var_recoder(&(Position_Controller.z_disturbance),N_Data_Points, "d");
//	 init_var_recoder(&(Position_Controller.u_dist_rej),N_Data_Points, "u_dist_rej");
//	 init_var_recoder(&(Position_Controller.force_vec.data[4]),N_Data_Points, "fd");
   //Computed velocities
//      init_var_recoder(&(Position_Controller.control_vec.data[5]),N_Data_Points,"dx1");
//      init_var_recoder(&(Position_Controller.control_vec.data[6]),N_Data_Points,"dy1");
//      init_var_recoder(&(Position_Controller.control_vec.data[7]),N_Data_Points,"dx2");
//      init_var_recoder(&(Position_Controller.control_vec.data[8]),N_Data_Points,"dy2");
//      init_var_recoder(&(Position_Controller.control_vec.data[9]),N_Data_Points,"dz");
// Computed control forces including feedforward
//
//      init_var_recoder(&(Position_Controller.force_steady_vec.data[0]),N_Data_Points,"fx1");
//      init_var_recoder(&(Position_Controller.force_steady_vec.data[1]),N_Data_Points,"fy1");
//      init_var_recoder(&(Position_Controller.force_steady_vec.data[2]),N_Data_Points,"fx2");
//      init_var_recoder(&(Position_Controller.force_steady_vec.data[3]),N_Data_Points,"fy2");
//      init_var_recoder(&(Position_Controller.force_steady_vec.data[4]),N_Data_Points,"fd");
//// DQ Currents of M1 and M2
      double mea_current_dq[2];
      init_var_recoder(&(mea_current_dq[0]),N_Data_Points,"i_dq1");
      init_var_recoder(&(mea_current_dq[1]),N_Data_Points,"i_dq2");
//      init_var_recoder(&(Position_Controller.M2dq.currents.data[2]),N_Data_Points,"i_dq2_pos_con");
      double ref_current_dq[2];
      init_var_recoder(&(ref_current_dq[0]),N_Data_Points,"i_dq1_ref");
      init_var_recoder(&(ref_current_dq[1]),N_Data_Points,"i_dq2_ref");
 // Z position reference
      if(PC_STATE_CONTROL){
//    	  init_var_recoder(&(Position_Controller.ref_vec.data[0]),N_Data_Points,"x_ref");
//    	  init_var_recoder(&(Position_Controller.ref_vec.data[1]),N_Data_Points,"y_ref");
    	  init_var_recoder(&(Position_Controller.ref_vec.data[2]),N_Data_Points,"z_ref");
//    	  //init_var_recoder(&(Position_Controller.ref_vec.data[3]),N_Data_Points,"a_ref");
//    	  //init_var_recoder(&(Position_Controller.ref_vec.data[4]),N_Data_Points,"b_ref");
//    	  init_var_recoder(&(Position_Controller.ref_vec.data[5]),N_Data_Points,"dx_ref");
//    	  init_var_recoder(&(Position_Controller.ref_vec.data[6]),N_Data_Points,"dy_ref");
//    	  init_var_recoder(&(Position_Controller.ref_vec.data[7]),N_Data_Points,"dz_ref");
//    	  //init_var_recoder(&(Position_Controller.ref_vec.data[8]),N_Data_Points,"da_ref");
//    	  //init_var_recoder(&(Position_Controller.ref_vec.data[9]),N_Data_Points,"db_ref");
      }else{
//          init_var_recoder(&(Position_Controller.ref_vec.data[0]),N_Data_Points,"x1_ref");
//          init_var_recoder(&(Position_Controller.ref_vec.data[1]),N_Data_Points,"y1_ref");
//          init_var_recoder(&(Position_Controller.ref_vec.data[2]),N_Data_Points,"x2_ref");
//          init_var_recoder(&(Position_Controller.ref_vec.data[3]),N_Data_Points,"y2_ref");
//          init_var_recoder(&(Position_Controller.ref_vec.data[4]),N_Data_Points,"z_ref");
      }
//// DQ Voltages
  	double mea_voltage_dq[4];
//      init_var_recoder(&(mea_voltage_dq[0]),N_Data_Points,"V_dq1");
//      init_var_recoder(&(mea_voltage_dq[1]),N_Data_Points,"V_dq2");
      init_var_recoder(&(mea_voltage_dq[2]),N_Data_Points,"V_dq3");
//      init_var_recoder(&(mea_voltage_dq[3]),N_Data_Points,"V_dq4");
// // DC Link voltages
    double mea_dc_link;
//    init_var_recoder(&mea_dc_link,N_Data_Points,"V_DC");
//	init_var_recoder(&(Position_Controller.state_est.data[0]),N_Data_Points,"x_est");
//    init_var_recoder(&(Position_Controller.state_est.data[1]),N_Data_Points,"y_est");
    init_var_recoder(&(Position_Controller.state_est.data[2]),N_Data_Points,"z_est"); //z state estimate
//    init_var_recoder(&(Position_Controller.state_est.data[3]),N_Data_Points,"a_est");
//    init_var_recoder(&(Position_Controller.state_est.data[4]),N_Data_Points,"b_est");
//    init_var_recoder(&(Position_Controller.state_est.data[5]),N_Data_Points,"dx_est");
//    init_var_recoder(&(Position_Controller.state_est.data[6]),N_Data_Points,"dy_est");
//    init_var_recoder(&(Position_Controller.state_est.data[7]),N_Data_Points,"dz_est");
//    init_var_recoder(&(Position_Controller.state_est.data[8]),N_Data_Points,"da_est");
//    init_var_recoder(&(Position_Controller.state_est.data[9]),N_Data_Points,"db_est");

   //************************** End of variable recorder ***********************************

    set_up_iir();

    init_optimizer(get_z_rec());

    while(1)
    {



    	//printf( "pos1 = %d\n; ", (int)measure_pos1s1_raw() );


//****************** Malta Event Handler ********************
//Don use local copy of event handler. If used the controller skips randomly the execution of set events
//Insted use the read_events() function to check for set events.

//printf("Event Handler \n");
// Clears read Events from global
//copy_events_to_local(&local_events);

    	//****************** Control Block ********************
    	//Keep all functions in the execution order in the if block. Otherwise the controller starts mixing up the execution sequence during operation.
    	//Handle downsampling of execution times directly in the if block if required.
           if ((read_events() & CONTROL_EVENT) != 0)
           {

        	   clear_event(CONTROL_EVENT);
        	   //*** Position Measurement update Event ***
             if (POSITION_MEA_EN)
             {

          	  //set_time_pin();

              //printf("Position Measurement Event\n");
          	  update_mea_malta_position_control(&Position_Controller);

              //clear_time_pin();

              //Clear_Event_Loc(&local_events,POSITION_MEA_EVENT);
              }

             //*** Position Control Event ***
             if (POSITION_CONTROL_EN==1)
                   {

                      	//printf("Position Control Event\n");

                      	//**************** Reference selection ********************
                      	 //Upate position reference
                      	  if (PC_STATE_CONTROL == 1) {
                      	  pos_ref.data[2] = get_reference(); //Sets z position reference for state feedback
                      	  pos_ref.data[7] = get_vel_reference();
                          }
                      	  else{
                      	  pos_ref.data[4] = get_reference(); //Sets z position reference for output feedback

                      	  }
                          //printf("%f\n",pos_ref.data[4] );

                      	update_ref_malta_position_control(&Position_Controller,&pos_ref);

                      	compute_forces_malta_position_control(&Position_Controller);

                      	//set_time_pin();
                      	//Overwrite forces for manual control
                        if (MANUAL_FORCE_CONTROL_M1_EN) {
                        	Position_Controller.force_steady_vec.data[0] = cM1fx;
							Position_Controller.force_steady_vec.data[1] = cM1fy;
							Position_Controller.force_steady_vec.data[4] = cM1fz;
                       }
                        if (MANUAL_FORCE_CONTROL_M2_EN) {
                        	Position_Controller.force_steady_vec.data[2] = cM2fx;
							Position_Controller.force_steady_vec.data[3] = cM2fy;
							Position_Controller.force_steady_vec.data[4] = cM1fz;
                       }


                      	compute_dq_action_malta_position_control(&Position_Controller);

                      	//clear_time_pin();

                      	//clear_event(POSITION_CONTROL_EVENT);
                      	//Clear_Event_Loc(&local_events,POSITION_CONTROL_EVENT);
                   }

             //temp_th = Position_Controller.M1dq.theta_z;

              //*** Overwrite Currents Commands for manual control ***
              if (MANUAL_CONTROL_M1_EN==1) {
              Position_Controller.M1dq.theta_z=ctheta;
              Position_Controller.M1dq.currents.data[0] = cdrive_cur;
              Position_Controller.M1dq.theta_b=cthetab;
              Position_Controller.M1dq.currents.data[1] = cbear_cur;
             }
             if (MANUAL_CONTROL_M2_EN==1) {
             Position_Controller.M2dq.theta_z=ctheta;
             Position_Controller.M2dq.currents.data[0] = cdrive_cur;
             Position_Controller.M2dq.theta_b=cthetab;
             Position_Controller.M2dq.currents.data[1] = cbear_cur;
            }


             //*** Current Control Block ***
              if (CURRENT_CONTROL_EN == 1)
                {
            	  if(check_softstart() == 0 && SOFTSTART_EN){
            		  step_count = 0;
            	  }
            	  else{

            	  	  //set_time_pin();
                     execute_current_controller(&Position_Controller.M1dq,&Position_Controller.M2dq);
                     //clear_time_pin();

                     if (step_count>=step_time){
                    	 if (STEP_BACK_EN == 1){
                	    cdrive_cur = 0;}
                        }
                     else{
                        step_count ++; }
            	  }

            	  advance_softstart();

                }

               // Clear event
               //clear_event(CURRENT_CONTROL_EVENT);
               //Clear_Event_Loc(&local_events,CURRENT_CONTROL_EVENT);

              //****************** Variable recorder ********************
               if ((read_events() & VAR_SAMPLE) != 0)
                                           {
                             	//set_time_pin();
                                              //Copy necessary variables to be recorded from current controller
                                         	 /*for (int j=0;j<9;j++)    {
                                         		  mea_current_abc[j] = read_current_cc(j,1); }
             */
                                         	  for (int k=0;k<2;k++)  {
                                         	  mea_current_dq[k] = read_current_dq_cc(k+1,1);  }

                                         	  //for (int k=0;k<2;k++)  {
                                         	  //mea_current_dq[k+2] = read_current_dq_cc(k+1,2);  }

                                         	  for (int k=0;k<4;k++) {
                                         	  mea_voltage_dq[k] = read_voltage_dq_cc(k,1);  }

                                         	  for (int k=0;k<2;k++) {
                                         	  ref_current_dq[k] = read_ref_current_dq_cc(k+1,1);  }

                                         	 mea_dc_link = read_voltage_dc_cc(2);

                                         	//z_laser = read_laser();

                                         	  sample_var_recorder();
                                         	  clear_event(VAR_SAMPLE);

                                         	  //Clear_Event_Loc(&local_events,VAR_SAMPLE);
                                         	  //clear_time_pin();
                                           }

                            //******************END Variable recorder ********************

          	  advance_timer();

               clear_time_pin();
           }

           //******************END Control Block ********************



                if ((read_events() & POLL_UART) != 0 || read_control_system_state()==0)
                {
            	/*
                	* Check, if UART0 has a received byte. If yes, send it to the command line tool.
                	*/

                	poll_uart0_execute_cli();
                	clear_event(POLL_UART);
                	//printf("%f\n",temp_th);
                	//printf("%f%f\n",read_current_dq_cc(2,1),read_current_dq_cc(2,2));
                	//Clear_Event_Loc(&local_events,POLL_UART);
                	//printf("%f\n",Position_Controller.mea_vec.data[4] );
                }


                //********************* Calibration procedure ***************************************************
                // Disable position control and position measurement and only activate the current control of the module to use
                // Change to M1 or M2 in the procedure below
                // Start system and set dcur to desired value
                // Start calibration with "startcalibrateDAng"
                // The system will then automatically step through the drive angles and report the z-position etc.

                                         if ((read_events() & CALIBRATE_THETA_D) != 0)
                                         {
                                          if (wait_time()==1)
                                           {
                                             printf("%lu,%lu,%f,%f\n",measure_pos3s2_raw(),measure_pos3s1_raw(),Position_Controller.M2dq.theta_z,measure_z());

                                            toggle_time_pin();
                                            Position_Controller.M2dq.theta_z = ang_cnt*M_PI/100*ang_stepsize;
                                            Position_Controller.M2dq.currents.data[0] = cdrive_cur;
                                            ang_cnt += ang_cnt_dir;
                                            //printf("%f \n",Position_Controller.M2dq.theta_z);
                                            if (ang_cnt*ang_stepsize>ang_steps_max_M2)
                                            {
                                             ang_cnt_dir = -1;
                                            }
                                             if (ang_cnt*ang_stepsize<ang_steps_min_M2)
                                             {
                                             ang_cnt_dir = 1;
                                             }

                                             }
                                             set_event(CALIBRATE_THETA_D);
                                             //Clear_Event_Loc(&local_events,CALIBRATE_THETA_D);
                                             //clear_event(CALIBRATE_THETA_D);
                                         }




           //local_events = 0;
    }

    cleanup_platform();
    return 0;
}
