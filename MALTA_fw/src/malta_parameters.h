/*
 * malta_parameters.h
 *
 *  Created on: 15.05.2018
 *      Author: fabiand
 */

#ifndef SRC_MALTA_PARAMETERS_H_
#define SRC_MALTA_PARAMETERS_H_

//************************ Control Parameters **************************
// Controllers

//Enables manual control via "ctheta"/"cdrive_cur"/"cbear_cur" by overwriting the position control actions
#define MANUAL_CONTROL_M1_EN 0
#define MANUAL_CONTROL_M2_EN 0

// Enables manual control of the bearing and driving forces by overwriting them after the position control with
// cM1fx/cM1fy/cM1fz...
#define MANUAL_FORCE_CONTROL_M1_EN 0
#define MANUAL_FORCE_CONTROL_M2_EN 0

// Controller config flags
#define SOFTSTART_EN 1  // Enables softstart to initialize the filters before the current controller runs
#define CURRENT_CONTROL_EN 1  //Enables the current controller
#define POSITION_CONTROL_EN 1 //Enables the position controller

// If enabled the firmware steps the position back at a given interval.
// Is not required with the double step reference function
#define STEP_BACK_EN 0


// Position Controller
#define PC_INTEGRAL_EN 0    //Enables disables Integral Action
#define PC_STATE_CONTROL 1  //Enables the state observer !!Observer Computation is currently commented in "malta_position_control.c" to reduce computation time!! and state feedback controller

//Enables disables the Bearing or parts of it
#define PC_BEARING_EN 1
#define PC_BEARING_M1_EN 1
#define PC_BEARING_M2_EN 1

//Enable for position control axes
#define PC_Z_POS_EN 1
#define PC_Y_POS_EN 1
#define PC_X_POS_EN 1

#define POSITION_MEA_EN 1 //Position Measurement enable
#define EN_MEA_FILTER 0  //General Filter for the measurement
#define EN_Z_MEA_FILTER 1 //Filter only for the z position
#define EN_DIV_FILTER 1   //Derivation Filter
#define EN_OUT_FILTER 0   //default off | Does not work realy well

// Current Controller
#define EN_MEA_FILTER_CC 0  //Current Measurement Filter default off

//Current controller enable
#define CURRENT_CONTROL_M1 1
#define CURRENT_CONTROL_M2 1



//
#define PLOT_CNT_EN 0
#define TIMING_PIN_EN 1

#define TEST_SEC_EN 0

//************************ Measurement Constants ***********************

// Radial Position
#define POS_MEA_M1_DX 0.00113255033557047 //Conversion constant gives mm
#define POS_MEA_M1_X0 2156.0
#define POS_MEA_M1_DY 0.000955414012738854 //Conversion constant gives mm
#define POS_MEA_M1_Y0 2483.5

#define POS_MEA_M2_DX 0.00100259933160045
#define POS_MEA_M2_X0 2318.75
#define POS_MEA_M2_DY -0.00091031692515172
#define POS_MEA_M2_Y0 2353.5


//************************ Conversion Constants ***********************
#define MM_TO_M 0.001

#endif /* SRC_MALTA_PARAMETERS_H_ */
