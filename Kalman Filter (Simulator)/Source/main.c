#include <stdio.h>

#include "kalman_filter.h"

int main()
{
	KalmanFilter kf = {.q = 0, .rl = 0, .x = 0, .p = 0, .k = 0};
	kalman(&kf, 0);
	printf("Hello World!\n");
	return 0;
}
