#include "car_info.h"
int16 sensor[4];  // 归一化后电感值，用于judge
int16 ad[4] = {0, 0, 0, 0};
int16 induc_ref[4] = {347, 347, 80, 45};
CarInfo car_info = {0, {0, 0}, {0, 0, 0, 0}, TAKE_OFF, 0, 0};
RingState ring_state = NOT_A_RING;
RingDir ring_dir = NOT_A_RING;

int16 test[4] = {0, 0, 0, 0};