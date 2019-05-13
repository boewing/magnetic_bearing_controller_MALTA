#include "variable_recorder.h"
#include "matrix_operations.h"

#define STEPW 1

typedef struct{
	double value;
	int t[4];
}cost_function;

double get_cost();
double find_1d_gradient(double vl, double vr);
void descent_step();
void advance_timer();
void init_optimizer(variable_rec_data_type varrec);
void activate_optimizer();
void reset_optimizer();
double abs_d(double val);
void t_to_x(int *t, mat_double *x_in);
void x_to_t(mat_double *x, int *t);
