#include <stdio.h>
#include <stdlib.h>
#include "buffer.h"

void initBuffer(int32_t size, buff * buffer)
{
    buffer->data = calloc(size, sizeof(*(buffer->data)));
    buffer->head = size - 1;
    buffer->size = size;
    buffer->inserts = 0;
}

void cleanupBuffer(buff * buffer)
{
    free(buffer->data);
}

void insertToBuffer(int32_t data, buff * buffer)
{
    buffer->inserts++;
    movePointerBuffer(buffer);
    buffer->data[buffer->head] = data;
}

void movePointerBuffer(buff * buffer)
{
    buffer->head += 1;
    if (buffer->head >= buffer->size)
    {
        buffer->head = buffer->head % buffer->size;
    }
}

int32_t getPreviousBuffer(int32_t previousN, buff * buffer)
{
    int32_t index = ((buffer->head + buffer->size) - (previousN % buffer->size)) % buffer->size;
    return buffer->data[index];
}

int32_t getHeadBuffer(buff * buffer)
{
    return getPreviousBuffer(0, buffer);
}

int32_t getAvgBuffer(buff * buffer)
{
    int32_t count = buffer->size;
    int32_t actual = 0;
    int32_t sum = 0;
    int32_t i;

    for (i = 0; i < count; i++)
    {
        int add = getPreviousBuffer(i, buffer);
        if (add == 0)
            break;
        sum += add;
        actual++;
    }

    if (actual == 0)
        return 0;
    return sum / actual;
}
