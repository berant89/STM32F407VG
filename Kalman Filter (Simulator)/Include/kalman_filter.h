#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct
{
	float q;
	float rl;
	float x;
	float p;
	float k;
}KalmanFilter;

float kalman(KalmanFilter* kf, float measurement);

#endif
