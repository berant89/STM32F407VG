#include "main.h"
#include "ImageUtil.h"
#include "RAWUtil.h"

#include <stdlib.h>

static __IO uint32_t uwTimingDelay; //Amount of time remaining

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

int main()
{
	//Delay(100);
	//printf("Hello World!\n");
	double* kernel;
	int kernel_size = 5;
	double sigma = 1.4;
	kernel = gaussKernel(kernel_size, sigma);
	print_kernel(kernel, kernel_size);
	free(kernel);
	
	//Test opening the file
	int size = 79*200;
	BYTE* image;
	image = load_raw("image_small.raw", size);
	free(image);
	return 0;
}
