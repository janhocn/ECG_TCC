#include <stdio.h>
#include <stdlib.h>
#include "bufferPeak.h"

void initBufferPeak(int32_t size, buffPeak * buffer)
{
    buffer->data = calloc(size, sizeof(*(buffer->data)));
    buffer->head = size - 1;
    buffer->size = size;
    buffer->inserts = 0;
}

void cleanupBufferPeak(buffPeak * buffer)
{
    free(buffer->data);
}

void insertToBufferPeak(Peak_t data, buffPeak * buffer)
{
    movePointerBufferPeak(buffer);
    buffer->data[buffer->head] = data;
    buffer->inserts++;
}

void movePointerBufferPeak(buffPeak * buffer)
{
    buffer->head += 1;
    if (buffer->head >= buffer->size)
    {
        buffer->head = buffer->head % buffer->size;
    }
}

Peak_t getPreviousPeak(int32_t previousN, buffPeak * buffer)
{
    int32_t index = ((buffer->head + buffer->size) - (previousN % buffer->size)) % buffer->size;
    return buffer->data[index];
}

Peak_t getHeadPeak(buffPeak * buffer)
{
    return getPreviousPeak(0, buffer);
}
