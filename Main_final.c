//////////////////////////////////////////////////////////////////////////////////
// Main.c
// Version 11
// John Bezzina & Brendan Yates, 2018
// Expanded upon code by:
// 	-Finn Andersen, 2014
// 	-Daniel Hranilovic & Qi Xin(Bob) Yang, 2017
// Last Modified 13/10/18
//////////////////////////////////////////////////////////////////////////////////

// Header Files
#include <stdio.h>
#include "includes.h"
#include "system.h"
#include "altera_avalon_pio_regs.h"
#include "alt_types.h"
#include "priv/alt_legacy_irq.h"
#include "sys/alt_irq.h" //needed only if using interrupts
#include "math.h"
#include "float.h"
#include "stdlib.h"

//////////////////////////////////////////////////////////////////////////////////
// Parameters
//////////////////////////////////////////////////////////////////////////////////
#define puck_radius 15 			// puck radius in pixels
#define paddle_pix_pos0 638 	// pixel coordinates of slider
#define paddle_pix_pos1 2 		// pixel coordinates of slider
#define table_border_high 453 	// pixel coordinates of table border
#define table_border_low 32 	// pixel coordinates of table border
#define slider_offset 9870 		//offset to hit puck
#define slider_start_enc 9275 	// start of encoder counts
#define slider_end_enc 114597 	// end of encoder counts
#define rest0 115000 			// output to paddle pwm
#define backL_hitR0 101700 		// output to paddle pwm
#define backR_hitL0 124700 		// output to paddle pwm
#define rest1 120000 			// output to paddle pwm
#define backL_hitR1 107200 		// output to paddle pwm
#define backR_hitL1 130000 		// output to paddle pwm
#define centre_enc 71852 		// centre encoder count for slider
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
//RTOS initialisations
//////////////////////////////////////////////////////////////////////////////////
//If there is a semaphore wait operation, a message will be printed
INT8U err2;
#define CE(x) if ((err2 = x) != OS_NO_ERR) printf("Runtime error: %d line %d - see ucos_ii.h\n", err2, __LINE__)

// Definition of Task Stacks
#define TASK_STACKSIZE 2048
OS_STK ResetKalman_stk[TASK_STACKSIZE];
OS_STK KalmanFilterX_stk[TASK_STACKSIZE];
OS_STK KalmanFilterY_stk[TASK_STACKSIZE];
OS_STK ResponseControl_stk[TASK_STACKSIZE];
OS_STK MotionControl_stk[TASK_STACKSIZE];

// Definition of Task Priorities
#define ResponseControl_PRIORITY 1
#define ResetKalman_PRIORITY 2
#define KalmanFilterX_PRIORITY 3
#define KalmanFilterY_PRIORITY 4
#define MotionControl_PRIORITY 5

// Semaphore
OS_EVENT *start_motion;
OS_EVENT *protect_encoder;
OS_EVENT *protect_global;
OS_EVENT *protect_location;
OS_EVENT *KalmanX_begin_sem;
INT8U err_kalmanx_begin;
OS_EVENT *KalmanY_begin_sem;
INT8U err_kalmany_begin;
OS_EVENT *Kalman_reset_sem;
INT8U err_kalman_reset;
OS_EVENT *KalmanX_finish_sem;
INT8U err_kalmanx_finish;
OS_EVENT *KalmanY_finish_sem;
INT8U err_kalmany_finish;
//////////////////////////////////////////////////////////////////////////////////

// Functions
void* context;
void matrix_multi_2_2(float A[2][2],float B[2][2],float C[2][2]);
void matrix_add_2_2(float A[2][2],float B[2][2],float C[2][2]);
void matrix_inv_2_2(float A[2][2],float invA[2][2]);
void DistortCompensate(int x_raw, int y_raw, int *x_fix, int *y_fix);

// Kalman array definitions
#define kdt 1;
int kR= 1;
float kA[2][2] = {{1, 1}, {0, 1}};
float kA_t[2][2]= {{1, 0}, {1 , 1}};
int kH[1][2]= {{1 , 0}};
int kH_t[2][1] = {{1}, {0}};
float kQ[2][2] = {{1, 0}, {0, 0.5}};
float kPx[2][2] = {{1, 0}, {0, 1}}; //Initial prediction covariance
float kPy[2][2] = {{1, 0}, {0, 1}}; //Initial prediction covariance
float kX[2]; //X-coordinate state matrix
float kY[2]; //Y-coordinate state matrix

// Globals for Kalman filtering/predicting endpoint.
int intersect_pos0,intersect_pos1; 	// intersect position in pixels
int final_pos0 = 0; 				// intersect position in encoder counts
int final_pos1 = 0; 				// intersect position in encoder counts
float int_time0,int_time1; 			// time to intersection in frames
int puck_measured_x; 				// global x-pos of puck
int puck_measured_vx; 				// global vx of puck
int puck_measured_y; 				// global y-pos of puck
int puck_measured_vy; 				// global vy of puck
int x_velocity; 					// global vx of puck after kalman filtering
float strike_time; 					// global strike time for strike control

//////////////////////////////////////////////////////////////////////////////////
// 							INTERRUPTS
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// New frame interrupt
// When new frame is ready:
// *Read puck coordinates
// *Begin Kalman filtering or reset filter if movement direction changed
//////////////////////////////////////////////////////////////////////////////////
static void NewFrameISR(){
	int puck_x_prev, puck_y_prev, puck_vx_prev,puck_vy_prev;
	int puck_x_raw, puck_y_raw;
	int puck_x_fix, puck_y_fix;
	if(IORD(DETECT_VALID_BASE,0)){
		// save previous values
		puck_x_prev = puck_measured_x;
		puck_y_prev = puck_measured_y;
		puck_vx_prev = puck_measured_vx;
		puck_vy_prev = puck_measured_vy;
		// read in puck coordinates
		puck_x_raw = IORD(X_POS_BASE,0);
		puck_y_raw = IORD(Y_POS_BASE,0);
		// Perform Distortion Correction
		DistortCompensate(puck_x_raw, puck_y_raw, &puck_x_fix, &puck_y_fix);
		puck_measured_x = puck_x_fix;
		puck_measured_y = puck_y_fix;
		// Calc change in pos (rough velocity estimate)
		puck_measured_vx = puck_measured_x - puck_x_prev;
		puck_measured_vy = puck_measured_y - puck_y_prev;
		// Reset Kalman on change direction otherwise cont.
		if((puck_vx_prev*puck_measured_vx<0)||(puck_vy_prev*puck_measured_vy<0)){
			OSSemPost(Kalman_reset_sem);
		}
		else{
			// Otherwise begin filtering
			OSSemPost(KalmanX_begin_sem);
			OSSemPost(KalmanY_begin_sem);
		}
	}
	//Clear edgecapture register
	IOWR(NEW_FRAME_BASE,3,0x1);
}

// ISR
static void resetISR()
{
	// stop motion
	IOWR(MOTOR0_POS_BASE, 0, 19145); // return to start
	IOWR(MOTOR1_POS_BASE, 0, 19145);
	IOWR(PWM_0_BASE, 0, 115000); // set to centre
	IOWR(PWM_1_BASE, 0, 120000);
	// reset ISR
	IOWR(RESET_ISR_BASE,3,0x1);
}

//////////////////////////////////////////////////////////////////////////////////
// 							TASKS
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Reset Kalman filter parameters
// Written by Finn Andersen, 2014
//////////////////////////////////////////////////////////////////////////////////
void ResetKalman(void* pdata){
	while (1) {
		//Stop and wait for next reset signal
		OSSemPend(Kalman_reset_sem,0,&err_kalman_reset);
		CE(err_kalman_reset);
		//Set initial states to those recently measured
		kX[0]=puck_measured_x;
		kX[1]=puck_measured_vx;
		kY[0]=puck_measured_y;
		kY[1]=puck_measured_vy;
		//Reset prediction covariance matrices
		kPx[0][0]=1;
		kPx[0][1]=0;
		kPx[1][0]=0;
		kPx[1][1]=1;
		kPy[0][0]=1;
		kPy[0][1]=0;
		kPy[1][0]=0;
		kPy[1][1]=1;
		//Let filters run
		CE(OSSemPost(KalmanX_begin_sem));
		CE(OSSemPost(KalmanY_begin_sem));
	}
}
//////////////////////////////////////////////////////////////////////////////////
// Handle Kalman filtering for x-dimension of puck
// Written by Finn Andersen, 2014
//////////////////////////////////////////////////////////////////////////////////
void KalmanFilterX(void* pdata){
	//Intermediate resultant matrices
	float S; //Residual covariance
	float P_pred[2][2]; //Prediction covariance
	float K[2]; //Kalman gain
	float kX_predict[2]; //Predicted state
	float residual;
	//Intermediate matrices for matrix operations
	float A_P[2][2];
	float A_P_At[2][2];
	float I_K_H[2][2];
	//Let the filter run indefinitely
	while (1) {
		//Wait until coordinates have been measured from next frame before filtering
		OSSemPend(KalmanX_begin_sem,0,&err_kalmanx_begin);
		CE(err_kalmanx_begin);
		//Calculate P_predicted
		matrix_multi_2_2(kA,kPx,A_P); //A*P
		matrix_multi_2_2(A_P,kA_t,A_P_At); //A*P*A'
		matrix_add_2_2(A_P_At,kQ,P_pred); //P1=A*P*A'+Q;
		//Calculate S
		S = P_pred[0][0] + kR;
		//Calculate K
		K[0] = P_pred[0][0]/S; //P1/S
		K[1] = P_pred[1][0]/S; //P3/S
		//Calculate I - KH
		I_K_H[0][0] = 1-K[0]; //I_K_H = [1 - K1, 0; -K2, 1]
		I_K_H[0][1] = 0;
		I_K_H[1][0] = -K[1];
		I_K_H[1][1] = 1;
		//Calculate P = (I-KH)P_pred
		matrix_multi_2_2(I_K_H,P_pred,kPx); //A*P
		//Calculate predicted state from previous state
		kX_predict[0] = kX[0] + kX[1]*kdt; //Predicted position
		kX_predict[1] = kX[1]; //Predicted velocity
		//Calculate residual (difference between predicted and measured)
		residual = puck_measured_x - kX_predict[0];
		//Calculate/update new true state estimate
		kX[0] = kX_predict[0] + residual*K[0];
		kX[1] = kX_predict[1] + residual*K[1];
		//Signal that Kalman filtering is complete, begin response control
		CE(OSSemPost(KalmanX_finish_sem));
	}
}
//////////////////////////////////////////////////////////////////////////////////
// Handle Kalman filtering for y-dimension of puck
// Written by Finn Andersen, 2014
//////////////////////////////////////////////////////////////////////////////////
void KalmanFilterY(void* pdata){
	//Intermediate resultant matrices
	float S; //Residual covariance
	float P_pred[2][2]; //Prediction covariance
	float K[2]; //Kalman gain
	float kY_predict[2]; //Predicted state
	float residual;
	//Intermediate matrices for matrix operations
	float A_P[2][2];
	float A_P_At[2][2];
	float I_K_H[2][2];
	//Let the filter run indefinitely
	while (1) {
		//Wait until coordinates have been measured from next frame before filtering
		OSSemPend(KalmanY_begin_sem,0,&err_kalmany_begin);
		CE(err_kalmany_begin);
		//Calculate P_predicted
		matrix_multi_2_2(kA,kPy,A_P); //A*P
		matrix_multi_2_2(A_P,kA_t,A_P_At); //A*P*A'
		matrix_add_2_2(A_P_At,kQ,P_pred); //P1=A*P*A'+Q;
		//Calculate S
		S = P_pred[0][0] + kR;
		//Calculate K
		K[0] = P_pred[0][0]/S; //P1/S
		K[1] = P_pred[1][0]/S; //P3/S
		//Calculate I - KH
		I_K_H[0][0] = 1-K[0]; //I_K_H = [1 - K1, 0; -K2, 1]
		I_K_H[0][1] = 0;
		I_K_H[1][0] = -K[1];
		I_K_H[1][1] = 1;
		//Calculate P = (I-KH)P_pred
		matrix_multi_2_2(I_K_H,P_pred,kPy); //A*P
		//Calculate predicted state from previous state
		kY_predict[0] = kY[0] + kY[1]*kdt; //Predicted position
		kY_predict[1] = kY[1]; //Predicted velocity
		//Calculate residual (difference between predicted and measured)
		residual = puck_measured_y - kY_predict[0];
		//Calculate/update new true state estimate
		kY[0] = kY_predict[0] + residual*K[0];
		kY[1] = kY_predict[1] + residual*K[1];
		//Signal that Kalman filtering is complete, begin response control
		CE(OSSemPost(KalmanY_finish_sem));
	}
}

//////////////////////////////////////////////////////////////////////////////////
// Matrix Code
// Written 2017
// Modified by John Bezzina, 2018
// - changed to float to remove errors in using integers
//////////////////////////////////////////////////////////////////////////////////
void matrix_multi_2_2(float A[2][2],float B[2][2],float C[2][2]){
	C[0][0] = (A[0][0]*B[0][0]) + (A[0][1]*B[1][0]);
	C[0][1] = (A[0][0]*B[0][1]) + (A[0][1]*B[1][1]);
	C[1][0] = (A[1][0]*B[0][0]) + (A[1][1]*B[1][0]);
	C[1][1] = (A[1][0]*B[0][1]) + (A[1][1]*B[1][1]);
}
void matrix_add_2_2(float A[2][2],float B[2][2],float C[2][2]){
	C[0][0] = A[0][0] +B[0][0];
	C[0][1] = A[0][1] +B[0][1];
	C[1][0] = A[1][0] +B[1][0];
	C[1][1] = A[1][1] +B[1][1];
}
void matrix_inv_2_2(float A[2][2],float invA[2][2]){
	float det;
	//check first
	det = (A[0][0]*A[1][1] - A[0][1]*A[1][0]);
	if (det){
		invA[0][0]=((A[1][1])/det);
		invA[0][1]=(-(A[0][1])/det);
		invA[1][0]=(-(A[1][0])/det);
		invA[1][1]=((A[0][0])/det);
	}else{
		printf("error in inverse/n");//should not error with values used
	}
}

//////////////////////////////////////////////////////////////////////////////////
// Distortion Correction
// Corrects pucks location to account for barrel distortion in camera
// Inputs: raw x/y data
// Outputs: corrected x/y data
// Written by John Bezzina & Brendan Yates, 2018
//////////////////////////////////////////////////////////////////////////////////
void DistortCompensate(int x_raw, int y_raw, int *x_fix, int *y_fix) {
	// Local Variables
	int dx, dy;
	float rad2, rad4;
	int x_centre = 335;
	int y_centre = 230;
	float k0, k2,k4;
	// Constants
	k0 = 9.031051091863599e-01; k2 = 8.306538179679109e-07; k4 = 7.481889186072878e-12;
	// Calculate pixel distance from centre
	dx = x_raw - x_centre;
	dy = y_raw - y_centre;
	// calculate r^2 and r^4
	rad2 = dx*dx + dy*dy;
	rad4 = rad2*rad2;
	// corrected pixel = centre + (k0+k2*r^2+k4*r^4)*(pixel distance from centre)
	*x_fix = x_centre + (k0+k2*rad2+k4*rad4)*dx;
	*y_fix = y_centre + (k0+k2*rad2+k4*rad4)*dy;
}

//////////////////////////////////////////////////////////////////////////////////
// Response control
// * processes puck motion into required response
// When Kalman filtering is complete:
// 	Determines position and direction of puck (from Kalman filters) and predicts
// 		intersection location
// 	Calculates target slider position based on puck intersection location
// 	Calculates intersection if rebounds occurs
// Written by John Bezzina & Brendan Yates, 2018
//////////////////////////////////////////////////////////////////////////////////
void ResponseControl(void* pdata){
	// response variables
	float kalman_x, kalman_vx, kalman_y, kalman_vy; // x,vx,y,vy after filtering
	// holds for previous intersection points
	int prev_pos=0,prev2_pos=0,prev3_pos=0,prev_pos1=0,prev2_pos1=0,prev3_pos1=0;
	INT8U err;
	// rebound variables
	int rebound_vx,rebound_vy,rebound_xpos,rebound_ypos,b4_hit_vx,b4_hit_vy,int_time2,rebound_time;
	float time0,time1; // time to intersect in frames
	int count=0; // stop rebound loop getting stuck
	int output; // decides which end to calculate rebound intersect
	int intersect_pos; // intersect position for rebounds
	while (1)
	{
		//Wait until Kalman filtering has finished
		OSSemPend(KalmanX_finish_sem,0,&err_kalmanx_finish);
		CE(err_kalmanx_finish);
		OSSemPend(KalmanY_finish_sem,0,&err_kalmany_finish);
		CE(err_kalmany_finish);
		//Disable new frame interrupts until processing completed
		IOWR(NEW_FRAME_BASE, 2, 0x0);
		//copy to local
		kalman_x = kX[0];
		kalman_vx = kX[1];
		kalman_y = kY[0];
		kalman_vy = kY[1];
		// find time to intersect using time = (x_pos of Puck)-(dist to slider)/(x velocity) (t=d/v)
		if(kalman_vx){
			time0 = (float)(paddle_pix_pos0-kalman_x-puck_radius)/kalman_vx;
			time1 = (float)(paddle_pix_pos1+kalman_x-puck_radius)/-kalman_vx;
		}
		// find intersect using intersect = y_pos + t*vy (d=vt)
		intersect_pos0=kalman_y+time0*kalman_vy;
		intersect_pos1=kalman_y+time1*kalman_vy;

		///////////////////////////////
		// REBOUNDS
		///////////////////////////////
		// y is table width and x is length
		// load rebound variables
		b4_hit_vx	= kalman_vx; // velocities before hitting wall
		b4_hit_vy	= kalman_vy;
		rebound_xpos	= kalman_x; // begins as current pos becomes pos at wall hit
		rebound_ypos	= kalman_y;
		rebound_vx		= 0; // velocities at hitting wall
		rebound_vy		= 0;
		// run different rebounds based on which way puck is travelling
		if(kalman_vx>0){ // compute rebounds on left side
			intersect_pos = intersect_pos0;
			output =0;
		}else if (kalman_vx<0){ // compute rebounds on right side
			intersect_pos = intersect_pos1;
			output=1;
		}
		count = 0; // reset loop counter
		// check if intersect is out side table's frame coordinates
		// dont run more than 5 times - was occasionally getting stuck in loop in early versions
		while ((intersect_pos>(table_border_high) || intersect_pos<(table_border_low))&&(count<5)){
			// check if distortion has caused position to go outside y boundaries
			if(rebound_ypos>table_border_high)
			rebound_ypos = table_border_high - 1;
			if(rebound_ypos<table_border_low)
			rebound_ypos = table_border_low + 1;
			// update velocities considering inelastic collisions
			// if a straight rebound on wall 30% decrease
			if (abs(b4_hit_vy) > abs((3*b4_hit_vx))){
				rebound_vx=0.667*b4_hit_vx;
				rebound_vy=-0.667*b4_hit_vy;
				// if only glancing the wall 15% decrease
			}else if (abs(b4_hit_vx) > abs((3*b4_hit_vy))) {
				rebound_vx=0.85*b4_hit_vx;
				rebound_vy=-0.85*b4_hit_vy;
				// else 45deg hit and 25% decrease
			}else{
				rebound_vx=0.75*b4_hit_vx;
				rebound_vy=-0.75*b4_hit_vy;
			}
			// compute new intersect location considering updated velocities
			if(intersect_pos>table_border_high){
				rebound_time = (table_border_high-(rebound_ypos+puck_radius))/b4_hit_vy; // time to hit wall
				rebound_ypos = table_border_high-puck_radius; // y-position at wall
			}else{
				rebound_time = (table_border_low-(rebound_ypos+puck_radius))/b4_hit_vy; // time to hit wall
				rebound_ypos = table_border_low-puck_radius; // y-position at wall
			}
			rebound_xpos = rebound_xpos+(rebound_time*b4_hit_vx); // x-position at wall
			if(output==0){
				// find time to intersect after rebound using x-dist remaining/vx (t=d/v)
				int_time2 = (paddle_pix_pos0-(rebound_xpos+puck_radius))/rebound_vx; // time to hit paddle
				// find intersect using intersect = y_pos + t*vy (d=vt)
				intersect_pos = rebound_ypos+(int_time2*rebound_vy); // pixel location of intersect
				if((intersect_pos<(table_border_high) && intersect_pos>(table_border_low))){ // only update if valid
					intersect_pos0 = intersect_pos;
					// intersect time is time to rebound + time from rebound to end
					time0 = rebound_time+int_time2;
				}
			}else if(output==1){
				// find time to intersect after rebound using x-dist remaining/vx (t=d/v)
				int_time2 = (paddle_pix_pos1+(rebound_xpos-puck_radius))/rebound_vx; // time to hit paddle
				// find intersect using intersect = y_pos + t*vy (d=vt)
				intersect_pos = rebound_ypos+(int_time2*rebound_vy); // pixel location of intersect
				if((intersect_pos<(table_border_high) && intersect_pos>(table_border_low))){ // only update if valid
					intersect_pos1 = intersect_pos;
					// intersect time is time to rebound + time from rebound to end
					time1 = rebound_time+int_time2;
				}
			}
			// update variables in case of another rebound
			b4_hit_vx	= rebound_vx; // velocities before hitting wall
			b4_hit_vy	= rebound_vy;
			count = count+1;
		}// loop if another rebound occurs
		// END REBOUNDS //

		// protect location
		OSSemPend(protect_location, 0, &err);
		// check that value has converged to expected value
		// NOTE: Values changed to decrease range on damaged end //
		if((abs(intersect_pos0 - (prev3_pos+prev2_pos+prev_pos)/3)<10)){
			// check if out of bounds and calculate encoder counts
			if(intersect_pos0<96){
				final_pos0 = slider_start_enc; // go to end position
			}
			else if(intersect_pos0>389){
				final_pos0 = slider_end_enc; // go to end position
			}
			else{
				final_pos0 = (291.08)*intersect_pos0-8508.05; // else calc encoder count
			}
		}
		if((abs(intersect_pos1 - (prev3_pos1+prev2_pos1+prev_pos1)/3)<10)){
			// check if out of bounds and calculate encoder counts
			if(intersect_pos1<96){
				final_pos1 = slider_end_enc; // go to end position
			}
			else if(intersect_pos1>389){
				final_pos1 = slider_start_enc; // go to end position
			}
			else{
				final_pos1 = (-291.08)*intersect_pos1+132085.88; // else calc encoder count
			}
		}
		// Update prev values
		prev3_pos=prev2_pos;
		prev2_pos=prev_pos;
		prev_pos=intersect_pos0;
		prev3_pos1=prev2_pos1;
		prev2_pos1=prev_pos1;
		prev_pos1=intersect_pos1;
		// Update globals
		int_time0 = time0;
		int_time1 = time1;
		x_velocity = kalman_vx;

		///////////////////////////////
		// Strike control
		///////////////////////////////
		// NOTE: target decreased for testing with only one side 
		float a,b,c,Rvxvy;
		int P =20; // Velocity increase from paddle strike
		float ATm =-9.0153; // Gradient of angle -> time conversion
		float ATc =12.0023; // Intercept of angle -> time conversion
		float strike_angle;
		if(kalman_x>(paddle_pix_pos1 + 35) && kalman_x < (paddle_pix_pos0-35)){
			// Calculate required return trajectory to reach other end goal
			if(kalman_vx>0){ // compute rebounds on left side
				Rvxvy = ((float)(intersect_pos0-242.5)/(float)(paddle_pix_pos0-paddle_pix_pos1+100));
			}else if (kalman_vx<0){ // compute rebounds on right side
				Rvxvy = ((float)(242.5-intersect_pos1)/(float)(paddle_pix_pos0-paddle_pix_pos1+100));
			}
			// Calculate quadratic coefficients
			a = -((5/3)*( kalman_vy + Rvxvy*kalman_vx) + 0.5*Rvxvy*P);
			b = 2* kalman_vx + P - 2*Rvxvy*kalman_vy;
			c = kalman_vy + Rvxvy*(kalman_vx + P);
			// Calculate strike angle from quadratic solution
			strike_angle = (a==0) ? 0 : (-b + sqrt(b*b - 4*a*c))/(2*a);
			// Calculate corresponding strike time for desired angle
			//strike_time = ATm*fabs(strike_angle) + ATc;
			strike_time = ATm*strike_angle + ATc;
		}
		// END STRIKE CONTROL //

		// Enable motion semaphore
		OSSemPost(start_motion);
		// Disable location semaphore
		OSSemPost(protect_location);
		// Re-enable frame interrupts
		IOWR(NEW_FRAME_BASE, 2, 0x1);
	}
}

//////////////////////////////////////////////////////////////////////////////////
// Motion Control
// Controls sliders and paddles for both sides
// variables with 0 mean left side of table
// variables with 1 mean right side of table
// from the point of view from emergency stop side
// 0 - GPIO_0	1 - GPIO_1
// Written by John Bezzina & Brendan Yates, 2018
//////////////////////////////////////////////////////////////////////////////////
void MotionControl(void* pdata){
	INT8U err;
	int slider_pos0,slider_pos1; // positions for sliders see above for 1/0 meanings
	int paddle_pos0, paddle_pos1; // positions for paddles
	int offset0,offset1;// used to offset slider to side for hitting
	offset0 = slider_offset; offset1 = slider_offset;
	int state0=1; // state for paddle control
	int state1=1;
	while (1)
	{
		OSSemPend(start_motion, 0, &err);
		///////////////////////////////
		// Motor Control
		//////////////////////////////
		// work out offset to control slider
		if(intersect_pos0>0&&intersect_pos0<480) {
			if(intersect_pos0<160) {
				offset0 = slider_offset;//
			}
			else if(intersect_pos0>290){
				offset0 = -slider_offset; // hit from right side
			}
		}
		if(intersect_pos1>0&&intersect_pos1<480) {
			if(intersect_pos1<160) {
				offset1 = -slider_offset;// hit on right  side
			}
			else if(intersect_pos1>290){
				offset1 = slider_offset; // hit on left side
			}
		}
		// Output target position
		int magic = rand()% 6 + 1; // does the magic
		slider_pos0 = final_pos0+offset0+magic;
		slider_pos1 = final_pos1+offset1+magic;
		// write to motors
		if (slider_pos0>(slider_start_enc+slider_offset)&&slider_pos0<(slider_end_enc)){
			if(x_velocity>0) // go to intersect
			IOWR(MOTOR0_POS_BASE, 0, slider_pos0);
			else if((x_velocity<0||x_velocity==0)&&state0==1) // go to centre
			IOWR(MOTOR0_POS_BASE, 0, (centre_enc-offset0+magic));
		}
		if (slider_pos1>(slider_start_enc+slider_offset)&&slider_pos1<(slider_end_enc)){
			if(x_velocity<0) // go to intersect
			IOWR(MOTOR1_POS_BASE, 0, slider_pos1);
			else if((x_velocity>0||x_velocity==0)&&state1==1)  // go to centre
			IOWR(MOTOR1_POS_BASE, 0, (centre_enc-offset1+magic));
		}
		// END MOTOR CONTROL //

		//////////////////////////////
		// Paddle Control
		//////////////////////////////
		switch(state0){
		case 1: // movement state
			paddle_pos0 = rest0;
			if((int_time0 > 0 && int_time0 <= 40))
			state0 = 2;
			break;
		case 2: // backswing paddle state
			if(offset0<0){
				paddle_pos0 = backL_hitR0;
			}else{
				paddle_pos0 = backR_hitL0;
			}
			if(int_time0 <= strike_time) {
				state0 = 3;
			}
			break;
		case 3: // hit puck state
			if(offset0<0) {
				paddle_pos0 = backR_hitL0;
			}else {
				paddle_pos0 = backL_hitR0;
			}
			if(int_time0<=-5){
				state0 = 1; // moving away from intersect
			}
			break;
		default: // rest paddle
			paddle_pos0 = rest0;
			state0 = 1;
			break;
		}
		switch(state1){
		case 1: // movement state
			paddle_pos1 = rest1;
			if((int_time1 > 0 && int_time1 <= 40))
			state1 = 2;
			break;
		case 2: // backswing paddle state
			if(offset1<0){
				paddle_pos1 = backL_hitR1;
			}else{
				paddle_pos1 = backR_hitL1;
			}
			if(int_time1 <= strike_time) {
				state1 = 3;
			}
			break;
		case 3: // hit puck state
			if(offset1<0) {
				paddle_pos1 = backR_hitL1;
			}else {
				paddle_pos1 = backL_hitR1;
			}
			if(int_time1<=-5){
				state1 = 1; // moving away from intersect
			}
			break;
		default: // rest paddle
			paddle_pos1 = rest1;
			state1 = 1;
			break;
		}
		// write to PWM
		IOWR(PWM_0_BASE, 0, paddle_pos0);
		IOWR(PWM_1_BASE, 0, paddle_pos1);
		// END PADDLE CONTROL //

		// disable motion semaphore
		OSSemPost(start_motion);
	}
}

//////////////////////////////////////////////////////////////////////////////////
//								MAIN
//////////////////////////////////////////////////////////////////////////////////
int main(void)
{
	// clear any pending interrupts
	IOWR(RESET_ISR_BASE,3,0x1);
	// registers and enable interrupt
	alt_irq_register(RESET_ISR_IRQ, context, resetISR);
	// enable interrupt mask for bit 0 and 1
	IOWR(RESET_ISR_BASE,2,0x1);

	// Regiser IRQ, enable interupts, clear edge capture
	IOWR(NEW_FRAME_BASE,3,0);
	alt_irq_register(NEW_FRAME_IRQ,context,NewFrameISR);
	IOWR(NEW_FRAME_BASE,2,0x1);
	IOWR(NEW_FRAME_BASE,3,0x0);

	// Semaphore intialise
	KalmanX_begin_sem = OSSemCreate(0); //Don't begin Kalman filtering until coordinates have been read
	KalmanY_begin_sem = OSSemCreate(0);
	Kalman_reset_sem = OSSemCreate(1); //Reset at in initialisation
	KalmanX_finish_sem = OSSemCreate(0); //Don't begin response control until filtering finished
	KalmanY_finish_sem = OSSemCreate(0);
	start_motion = OSSemCreate(0);
	protect_location = OSSemCreate(1);

	// Create Tasks
	OSTaskCreateExt(ResponseControl,
		NULL,
		(void *)&ResponseControl_stk[TASK_STACKSIZE-1],
		ResponseControl_PRIORITY,
		ResponseControl_PRIORITY,
		ResponseControl_stk,
		TASK_STACKSIZE,
		NULL,
		0);
	OSTaskCreateExt(ResetKalman,
		NULL,
		(void *)&ResetKalman_stk[TASK_STACKSIZE-1],
		ResetKalman_PRIORITY,
		ResetKalman_PRIORITY,
		ResetKalman_stk,
		TASK_STACKSIZE,
		NULL,
		0);
	OSTaskCreateExt(KalmanFilterX,
		NULL,
		(void *)&KalmanFilterX_stk[TASK_STACKSIZE-1],
		KalmanFilterX_PRIORITY,
		KalmanFilterX_PRIORITY,
		KalmanFilterX_stk,
		TASK_STACKSIZE,
		NULL,
		0);
	OSTaskCreateExt(KalmanFilterY,
		NULL,
		(void *)&KalmanFilterY_stk[TASK_STACKSIZE-1],
		KalmanFilterY_PRIORITY,
		KalmanFilterY_PRIORITY,
		KalmanFilterY_stk,
		TASK_STACKSIZE,
		NULL,
		0);
	OSTaskCreateExt(MotionControl,
		NULL,
		(void *)&MotionControl_stk[TASK_STACKSIZE-1],
		MotionControl_PRIORITY,
		MotionControl_PRIORITY,
		MotionControl_stk,
		TASK_STACKSIZE,
		NULL,
		0);
	// Start OS
	OSStart();
	return 0;
}
//////////////////////////////////////////////////////////////////////////////////
