#include "RAWUtil.h"

#include <stdio.h>
#include <stdlib.h>

#include "rt_sys.h"

/**
 * @brief Loads a 8-bit RAW image from a given filename.
 * @param input The name of the file to be read into memory.
 * @param size The size of the file to be read.
 * @return Pointer to the image.
 */
BYTE* load_raw(const char* input, int size)
{
	printf("%d\n", size);
	//Create buffer.
	BYTE* buffer;
	buffer  = (BYTE*) malloc(size);
	if(!buffer)
	{
		printf("Couldn't allocate buffer in load_raw\n");
		exit(1);
	}
	
	//Attempt to open file.
	//FILE* fp = fopen(input, "rb");
	FILEHANDLE fp = _sys_open(input, OPEN_R+OPEN_B);
	if(fp > 0)
	{
		_sys_read(fp, buffer, size, OPEN_B);
		_sys_close(fp);
		//fread(buffer, size, 1, fp);
		//fclose(fp);
	}
	else
	{
		free(buffer); //Delete the buffer and it's content.
		printf("Failed to open file: %s\n", input);
		exit(1);
	}

	return buffer;
}

/**
 * @brief Creates and writes an 8-bit RAW image with a given filename.
 * @param filename The name of the RAW output.
 * @param image Pointer to the image.
 * @param size The size of the image.
 */
void write_raw(const char* filename, BYTE* image, int size)
{
	//Create buffer.
	BYTE* buffer;
	buffer = (BYTE*) malloc(size);
	if(!buffer)
	{
		printf("Couldn't allocate buffer in load_raw\n");
		exit(1);
	}
	
	//Attempt to create the file with the given filename.
	//FILE *fp = fopen(filename, "wb");
	FILEHANDLE fp = _sys_open(filename, OPEN_W+OPEN_B);
	//Check if it opened.
	if(fp > 0)
	{
		_sys_write(fp, buffer, size, OPEN_B);
		_sys_close(fp);
		//fwrite(buffer, size, 1, fp);
		//fclose(fp);
		free(buffer);
	}
	else
	{
		printf("%s failed to open to be written to.\n", filename);
		free(buffer);
		exit(1);
	}
}
