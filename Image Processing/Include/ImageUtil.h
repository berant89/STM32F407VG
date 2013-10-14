#ifndef IMAGEUTIL_H
#define IMAGEUTIL_H

typedef unsigned char BYTE;

double* BYTE2double(BYTE* im, int rows, int cols);
BYTE* double2BYTE(double* im, int rows, int cols);
double* gaussKernel(int kernel_size, double sigma);
void print_kernel(double* kernel, int size);
double* gaussian_blur(double* kernel, int kernel_size, double* image, int rows, int cols);
double* sobel_edge(const int*, const int*, const double*, int, int);
double* line_detect(const int*, const double*);
int* hough_transform(const double*);
double* hough_lines(int, int, int, const int*);
BYTE* threshold(int, const BYTE*);

#endif
