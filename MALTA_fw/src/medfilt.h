/**
* Moving Median Filter.
* @author CHEVALLIER Yves <ycr@csem.ch>
*
* This algorithm is iterative. Each call will compute the next point.
* In the example below, the kernel has a size of 3. Notice that the
* values in the kernel are alway sorted. The left value is therefore
* the minimum in the kernel, the center value is the median and the
* right value is the maximum value.
*
* Input data:         0 1 2 5 4 8 3
* Kernel:      |0 0 0|            .   (min=0, med=0, max=0)
*                |0 0 0|          .   (min=0, med=0, max=0)
*                  |0 0 1|        .   (min=0, med=0, max=1)
*                    |0 1 2|      .   (min=0, med=1, max=2)
*                      |1 2 5|    .   (min=1, med=2, max=5)
*                        |2 4 5|  .   (min=2, med=4, max=5)
*                          |4 5 8|.   (min=4, med=5, max=8)
*                            |3 4 8|  (min=3, med=4, max=8)
*/
#pragma once
#include <stdlib.h>

typedef struct MedfiltNode {
	double value;
	size_t index; // Node index in the sorted table
	struct MedfiltNode* parent;
	struct MedfiltNode* sorted;
} MedfiltNode;

typedef struct MedfiltData {
	MedfiltNode *kernel; // Working filter memory
	MedfiltNode *oldest; // Reference to the oldest value in the kernel
	size_t length; // Number of nodes
} MedfiltData;

void medfilt_init(MedfiltData *data, MedfiltNode *nodes, size_t length, double init);
void medfilt(MedfiltData *data, double input, double *median);
