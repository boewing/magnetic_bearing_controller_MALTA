#ifndef position_control_observer_matrizes_h #define position_control_observer_matrizes_h const double AmLC[10][10]= {{0.82549, 0, 0, 0, 0.00014714, 4.5637e-05, 0, 0, 0, 3.6784e-09},                              {0, 0.80558, -7.039e-08, 0.00055745, 0, 0, 4.514e-05, -1.7598e-12, 1.3936e-08, 0},                              {0, 0.00014209, 0.94684, -0.00012483, 0, 0, 3.5523e-09, 4.8671e-05, -3.1207e-09, 0},                              {0, 0.096272, -1.0655e-05, 0.80237, 0, 0, 2.4068e-06, -2.6637e-10, 4.5059e-05, 0},                              {0.025384, 0, 0, 0, 0.82266, 6.3461e-07, 0, 0, 0, 4.5566e-05},                              {-301.9298, 0, 0, 0, 0.51501, 0.99245, 0, 0, 0, 1.2875e-05},                              {0, -375.5832, -0.00016815, 2.1728, 0, 0, 0.99061, -4.2038e-09, 5.4319e-05, 0},                              {0, 0.06441, -28.2549, -0.048845, 0, 0, 1.6102e-06, 0.99929, -1.2211e-06, 0},                              {0, 376.3513, 0.083963, -388.8605, 0, 0, 0.0094088, 2.0991e-06, 0.99028, 0},                              {89.1832, 0, 0, 0, -312.6069, 0.0022296, 0, 0, 0, 0.99218}}; const double Bobs[10][5]= {{3.7605e-09, 0, 3.7605e-09, 0, 0},                              {0, 3.7605e-09, 0, 3.7605e-09, 0},                              {0, -3.0361e-16, 0, 3.0361e-16, 7.5209e-09},                              {0, 4.9518e-08, 0, -4.9518e-08, 1.4372e-14},                              {-4.9518e-08, 0, 4.9518e-08, 0, 0},                              {0.00015042, 0, 0.00015042, 0, 0},                              {0, 0.00015042, 0, 0.00015042, 0},                              {0, -1.2144e-11, 0, 1.2144e-11, 0.00030083},                              {0, 0.0019807, 0, -0.0019807, 5.7488e-10},                              {-0.0019807, 0, 0.0019807, 0, 0}}; const double Cobs[10][10]= {{1, 0, 0, 0, -0.078752, 2.5001e-05, 0, 0, 0, -1.9688e-06},                              {0, 1, 1.5049e-07, 0.078752, 0, 0, 2.5001e-05, 3.7622e-12, 1.9688e-06, 0},                              {1, 0, 0, 0, 0.078752, 2.5001e-05, 0, 0, 0, 1.9688e-06},                              {0, 1, -1.5049e-07, -0.078752, 0, 0, 2.5001e-05, -3.7622e-12, -1.9688e-06, 0},                              {0, 0, 1, -6.1314e-09, 0, 0, 0, 2.5e-05, -1.5328e-13, 0},                              {1.253, 0, 0, 0, -0.084591, 1, 0, 0, 0, -0.10558},                              {0, 1.253, 0.00807, 0.084591, 0, 0, 1, 2.0175e-07, 0.10558, 0},                              {1.253, 0, 0, 0, 0.084591, 1, 0, 0, 0, 0.10558},                              {0, 1.253, -0.00807, -0.084591, 0, 0, 1, -2.0175e-07, -0.10558, 0},                              {0, 0, -4.6866e-10, -0.00024525, 0, 0, 0, 1, -6.1314e-09, 0}}; const double L[10][5]= {{0.08822, 0, 0.086351, 0, 0},                           {0, 0.093698, 0, 0.10078, 7.1456e-08},                           {0, 0.00072143, 0, -0.00086352, 0.05316},                           {0, 1.2069, 0, -1.3031, 1.4099e-05},                           {-1.1389, 0, 1.1135, 0, 0},                           {155.483, 0, 148.9433, 0, 0},                           {0, 175.2436, 0, 202.8338, 0.0001723},                           {0, 0.2748, 0, -0.33921, 28.2549},                           {0, 2290.9105, 0, -2667.25, 0.068168},                           {-2039.5304, 0, 1950.35, 0, 0}}; const double AmLC_no_z[10][10]= {{0.82549, 0, 0, 0, 0.00014714, 4.5637e-05, 0, 0, 0, 3.6784e-09},                                   {0, 0.80558, 1.0653e-09, 0.00055745, 0, 0, 4.514e-05, 2.6631e-14, 1.3936e-08, 0},                                   {0, 0.00014209, 1, -0.00012483, 0, 0, 3.5523e-09, 5e-05, -3.1208e-09, 0},                                   {0, 0.096272, 3.4442e-06, 0.80237, 0, 0, 2.4068e-06, 8.6105e-11, 4.5059e-05, 0},                                   {0.025384, 0, 0, 0, 0.82266, 6.3461e-07, 0, 0, 0, 4.5566e-05},                                   {-301.9298, 0, 0, 0, 0.51501, 0.99245, 0, 0, 0, 1.2875e-05},                                   {0, -375.5832, 4.152e-06, 2.1728, 0, 0, 0.99061, 1.038e-10, 5.4319e-05, 0},                                   {0, 0.06441, -9.3339e-08, -0.048845, 0, 0, 1.6102e-06, 1, -1.2211e-06, 0},                                   {0, 376.3513, 0.15213, -388.8605, 0, 0, 0.0094088, 3.8033e-06, 0.99028, 0},                                   {89.1832, 0, 0, 0, -312.6069, 0.0022296, 0, 0, 0, 0.99218}}; const double L_no_z[10][5]= {{0.08822, 0, 0.086351, 0, 0},                                {0, 0.093698, 0, 0.10078, 0},                                {0, 0.00072143, 0, -0.00086352, 0},                                {0, 1.2069, 0, -1.3031, 0},                                {-1.1389, 0, 1.1135, 0, 0},                                {155.483, 0, 148.9433, 0, 0},                                {0, 175.2436, 0, 202.8338, 0},                                {0, 0.2748, 0, -0.33921, 0},                                {0, 2290.9105, 0, -2667.25, 0},                                {-2039.5304, 0, 1950.35, 0, 0}};#endif