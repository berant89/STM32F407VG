#ifndef RAWUTIL_H
#define RAWUTIL_H

typedef unsigned char BYTE;

BYTE* load_raw(const char* input, int size);
void write_raw(const char* filename, BYTE* image, int size);

#endif
