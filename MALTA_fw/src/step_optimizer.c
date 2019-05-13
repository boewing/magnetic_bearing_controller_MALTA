#include "step_optimizer.h"
#include "reference_generation.h"
#include "commands.h"
#include "optimizer_coordinate_transform.h"
#include "matrix_operations.h"

#define STORE_LEN 10

static cost_function middle_cost_function;
static cost_function right_cost_function[4];
static cost_function left_cost_function[4];
static cost_function visited[STORE_LEN];
static variable_rec_data_type varrec;
static mat_double G_mat;
static mat_double G_inv_mat;
static int counter;
static int epoch;
static int enable;
static int epochs;
static int i;
static int store_i;
mat_double x;
mat_double h_mat;
double h[4] = {0.0, 0.4, 0.02, 0.1};

void init_optimizer(variable_rec_data_type recorder){
	set_mat(&G[0][0],3,3,&G_mat);
	set_mat(&G_inv[0][0],3,3,&G_inv_mat);
	set_mat(&h[1], 3, 1, &h_mat);
	Init_Matrix(&x, 3, 1);
	printMat(&h_mat);
	varrec = recorder;
	reset_optimizer();
}

double get_cost(){

	double error = 0;
	double equilibrium = 0;

	//the equilibrium is the average over the last 100 steps
	for(int i = get_step_time2() - 100; i < get_step_time2(); ++i)
		equilibrium += varrec.data_store[i];
	equilibrium = equilibrium/100;
	//equilibrium = get_step_size() + manual_position_reference();
	printf("equilibrium = %f \n", equilibrium);
	for(int i = get_t4() + get_step_time1(); i < get_step_time2()-50*20; ++i){
		error += pow(1e3*(varrec.data_store[i] - equilibrium),2);
		//printf("current error = %f \n", error);
	}
	//printf("The current error is %f \n", error);
	return error;
}

double find_1d_gradient(double vl, double vr){
	return (vr - vl)/2;
}

void activate_optimizer(){
	enable = 1;
}

void reset_optimizer(){
	counter = 0;
	epoch = 1;
	epochs = 2;
	enable = 0;
	i = 1;
	store_i = 0;
	middle_cost_function.t[1] = get_t(1);
	middle_cost_function.t[2] = get_t(2);
	middle_cost_function.t[3] = get_t(3);
	middle_cost_function.value = 0;
	right_cost_function[1].value = 0;
	left_cost_function[1].value = 0;
	right_cost_function[2].value = 0;
	left_cost_function[2].value = 0;
	right_cost_function[3].value = 0;
	left_cost_function[3].value = 0;

	for(int k = 0; k < STORE_LEN; ++k){
		visited[k].value = 1e30;
		visited[k].t[1] = 0;
		visited[k].t[2] = 0;
		visited[k].t[3] = 0;
	}
}

void advance_timer(){
	if(enable){
		if(counter < 60000){
			if(counter == 0){
				cdstep_with_varrec(NULL,NULL);
			}
			if(counter == 10000){
				middle_cost_function.t[1] = get_t(1);
				middle_cost_function.t[2] = get_t(2);
				middle_cost_function.t[3] = get_t(3);
				for(int i = 1; i <= 3; ++i){
					for(int ii = 1; ii <= 3; ++ii){
						right_cost_function[i].t[ii] = get_t(ii);
						left_cost_function[i].t[ii] = get_t(ii);
					}
				}
				middle_cost_function.value += get_cost();
				printf("t_beg=[%d %d %d]\n", middle_cost_function.t[1], middle_cost_function.t[2], middle_cost_function.t[3]);
				//printf("x before update = [%.2f, %.2f, %.2f]\n", x.data[0], x.data[1], x.data[2]);
			}

			if(counter == 20000){
				//t_to_x(middle_cost_function.t, x);
				//x.data[i-1] += STEPW;
				//x_to_t(x, right_cost_function[i].t);
				right_cost_function[i].t[i] = middle_cost_function.t[i] + STEPW;
				set_all_t(right_cost_function[i].t);
				cdstep_with_varrec(NULL,NULL);
			}
			if(counter == 30000){
				right_cost_function[i].value += get_cost();
				printf("t_right=[%d %d %d]\n", right_cost_function[i].t[1], right_cost_function[i].t[2], right_cost_function[i].t[3]);
				//printf("x right is [%.2f %.2f %.2f]\n",x.data[1-1], x.data[2-1], x.data[3-1]);

			}
			if(counter == 40000){
				//t_to_x(middle_cost_function.t, x);
				//x.data[i-1] -= STEPW;
				//x_to_t(x, left_cost_function[i].t);
				left_cost_function[i].t[i] = middle_cost_function.t[i] - STEPW;
				set_all_t(left_cost_function[i].t);
				cdstep_with_varrec(NULL,NULL);
			}
			if(counter == 50000){
				left_cost_function[i].value += get_cost();
				printf("t_left=[%d %d %d]\n", left_cost_function[i].t[1], left_cost_function[i].t[2], left_cost_function[i].t[3]);
				set_all_t(middle_cost_function.t);

				if(i < 3){
					counter = 10000;
					++i;
				}
			}
			counter++;
		}
		else{
			counter = 0;
			i = 1;
			if (epoch < epochs)
				epoch++;
			else{
				epoch = 1;
				i = 1;
				counter = -20000;

				//gradient with respect to t
				mat_double gradient;
				mat_double gradient_x;
				mat_double update;
				mat_double update_x;
				Init_Matrix(&update,3,1);
				Init_Matrix(&update_x,3,1);
				Init_Matrix(&gradient,1,3);
				Init_Matrix(&gradient_x, 1 ,3);

				gradient.data[0] = find_1d_gradient(left_cost_function[1].value, right_cost_function[1].value)/epochs;
				gradient.data[1] = find_1d_gradient(left_cost_function[2].value, right_cost_function[2].value)/epochs;
				gradient.data[2] = find_1d_gradient(left_cost_function[3].value, right_cost_function[3].value)/epochs;

				matmul(&gradient,&G_inv_mat, &gradient_x);

				//printf("t = [%d %d %d] samples \n", middle_cost_function.t[1], middle_cost_function.t[2], middle_cost_function.t[3]);
				printf("grad(f) = [%.2f, %.2f, %.2f]\n", gradient.data[0], gradient.data[1], gradient.data[2]);
				printf("grad_x(f) = [%.2f, %.2f, %.2f]\n", gradient_x.data[0], gradient_x.data[1], gradient_x.data[2]);

				//printf("f_cost(t1 = %d) = %.2f \n", left_cost_function[1].t[1], left_cost_function[1].value);
				//printf("f_cost(t1 = %d) = %.2f \n", middle_cost_function.t[1], middle_cost_function.value);
				//printf("f_cost(t1 = %d) = %.2f \n", right_cost_function[1].t[1], right_cost_function[1].value);

				//update middle cost function
				//printf("update_x = \n");

				//transposing the gradient in x
				gradient_x.dcol = 1;
				gradient_x.drow = 3;

				matmul_element(&h_mat, &gradient_x, &update_x);
				//printMat(&update_x);
				matmul(&G_inv_mat, &update_x, &update);
				printf("update_t = \n");
				printMat(&update);

				for(int k = 1; k <= 3; ++k){
					//x.data[k-1] -= (h[k]*gradient_x.data[k-1]);
					middle_cost_function.t[k] = middle_cost_function.t[k] - (int) (update.data[k-1]);
				}
				//printf("t after update = [%d %d %d] samples \n", middle_cost_function.t[1], middle_cost_function.t[2], middle_cost_function.t[3]);


				//gradient descent does no update anymore -> go for steepest descent
				if(abs_d(update.data[1-1]) < 1 && abs_d(update.data[2-1]) < 1 && abs_d(update.data[3-1]) < 1){
					// do steepest descent
					t_to_x(middle_cost_function.t, &x);
					if (h[1]*abs_d(gradient_x.data[1-1]) >= h[2]*abs_d(gradient_x.data[2-1]) && h[1]*abs_d(gradient_x.data[1-1]) >= h[3]*abs_d(gradient_x.data[3-1])){
						x.data[1-1] = x.data[1-1] - sign(gradient_x.data[1-1]);
					}else if(h[2]*abs_d(gradient_x.data[2-1]) >= h[3]*abs_d(gradient_x.data[3-1])){
						x.data[2-1] = x.data[2-1] - sign(gradient_x.data[2-1]);
					}else{
						x.data[3-1] = x.data[3-1] - sign(gradient_x.data[3-1]);
					}
					x_to_t(&x, middle_cost_function.t);
					printf("t after small updt = [%d %d %d] samples \n", middle_cost_function.t[1], middle_cost_function.t[2], middle_cost_function.t[3]);

					//stop if already visited
					for(int k = 0; k < STORE_LEN; ++k){
						if(middle_cost_function.t[1] == visited[k].t[1] && middle_cost_function.t[2] == visited[k].t[2] && middle_cost_function.t[3] == visited[k].t[3]){
							//already visited -> optimum is the minimum of all values
							double min = 1e30;
							int min_i;
							for(int kk = 0; kk < STORE_LEN; ++kk){
								if(visited[kk].value < min){
									min = visited[kk].value;
									min_i = kk;
								}
							}
							printf("optimum at t = [%d, %d, %d]\n", visited[min_i].t[1], visited[min_i].t[2], visited[min_i].t[3]);
							enable = 0;
							counter = 0;
							break;
						}
					}
					//double stepsize = get_step_size();
					printf("small stp mode\nt_new = [%d, %d, %d]\n", middle_cost_function.t[1], middle_cost_function.t[2], middle_cost_function.t[3]);
				}else{
					printf("t_new = [%d, %d, %d] \n", middle_cost_function.t[1], middle_cost_function.t[2], middle_cost_function.t[3]);
				}

				//store visited values
				visited[store_i] = middle_cost_function;
				store_i = (store_i + 1)%STORE_LEN;

				//reset the gradient and the cost functions
				middle_cost_function.value = 0;
				for(int k = 1; k <= 3; ++k){
					right_cost_function[k].value = 0;
					left_cost_function[k].value = 0;
					set_t(k, middle_cost_function.t[k]);
				}
			}
		}
	}

}

//coordinate transformation to increase the gradient descent performance
void x_to_t(mat_double *x, int *t){

	mat_double t_mat;
	Init_Matrix(&t_mat, 3, 1);
	matmul(&G_inv_mat, x, &t_mat);

	t[1] = t_mat.data[0];
	t[2] = t_mat.data[1];
	t[3] = t_mat.data[2];

	//t[2] = 2*x[1] + 2*x[2];
	//t[3] = 2*x[1] + 2*x[2] + x[3];
}

void t_to_x(int *t, mat_double *x_in){
	mat_double t_mat;
	double t_d[3];
	//printf("t in conversion = [%d %d %d] samples \n", t[1], t[2], t[3]);
	t_d[0] = (double) t[1];
	t_d[1] = (double) t[2];
	t_d[2] = (double) t[3];
	//printf("t in conversion = [%f %f %f] samples \n", t_d[0], t_d[1], t_d[2]);
 	set_mat(t_d, 3, 1, &t_mat);
 	//printMat(&t_mat);
 	//printMat(&G_mat);
	matmul(&G_mat, &t_mat, x_in);
	//printMat(x_in);
}

int sign(double val){
	return (val > 0) - (val < 0);
}

double abs_d(double val){
	if (val >= 0){
		return val;
	}
	return -val;
}
