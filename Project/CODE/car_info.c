#include "car_info.h"
int16 sensor[4];  // 归一化后电感值，用于judge
int16 ad[5] = {0, 0, 0, 0, 0};
int16 induc_ref[5] = {100, 100, 80, 45, 256};
CarInfo car_info = {0, {0, 0}, {0, 0, 0, 0}, TAKE_OFF, 0, 0};
RingState ring_state = NOT_A_RING;
RingDir ring_dir = NOT_A_RING;
TakeOffState take_off_state = STAND_UP;
StopState stop_state = TURN_READY;

int16 test[4] = {0, 0, 0, 0};
float angle_test = 0;