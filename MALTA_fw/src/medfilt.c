/**
 * Moving Median Filter.
 * @author CHEVALLIER Yves <ycr@csem.ch>
 */
#include <stdint.h>
#include <stdlib.h>
#include "medfilt.h"

static void swap(MedfiltNode **a, MedfiltNode **b)
{
    // Swap two node references in the sorted table.
    MedfiltNode *temp = *a;
    *a = *b;
    *b = temp;

    // Preserve index. Used to retrive the node position in the sorted table.
    size_t index = (*a)->index;
    (*a)->index = (*b)->index;
    (*b)->index = index;
}

void medfilt(MedfiltData *data, double input, double *median)
{
    // New value replaces the oldest
    MedfiltNode *n = data->kernel;
    MedfiltNode *node = data->oldest;
    node->value = input;
    data->oldest = node->parent;

#   define VAL(x) (n[x].sorted->value)

    // Sort the kernel
    for (size_t i = node->index; i < data->length - 1 && VAL(i) > VAL(i + 1); i++) {
        swap(&n[i].sorted, &n[i + 1].sorted);
    }

    for (size_t i = node->index; i > 0 && VAL(i) < VAL(i - 1); i++) {
        swap(&n[i].sorted, &n[i - 1].sorted);
    }

    // Get kernel information from sorted entries
    //*min = VAL(0);
    //*max = VAL(data->length - 1);
    *median = n[(data->length / 2)].sorted->value;

#   undef VAL
}

void medfilt_init(MedfiltData *data, MedfiltNode *nodes, size_t length, double init)
{
    data->kernel = nodes;
    data->length = length;

    // Linked list initialization
    data->oldest = &data->kernel[length - 1];
    for (size_t i = 0; i < length; i++) {
        data->kernel[i] = (MedfiltNode) {
            .value = init,
            .parent = data->oldest,
            .index = i,
            .sorted = &data->kernel[i]
        };
        data->oldest = &data->kernel[i];
    }
}
