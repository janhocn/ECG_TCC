#ifndef USER_BUFFER_H_
#define USER_BUFFER_H_
#include "arm_math.h"

typedef struct
{
    uint32_t head;
    uint32_t size;
    uint32_t inserts;
    int32_t * data;
} buff;

void initBuffer(int32_t size, buff * buffer);
void cleanupBuffer(buff * buffer);
void insertToBuffer(int32_t data, buff * buffer);
void movePointerBuffer(buff * buffer);
int32_t getPreviousBuffer(int32_t previousN, buff * buffer);
int32_t getHeadBuffer(buff * buffer);
int32_t getAvgBuffer(buff * buffer);

#endif /* USER_BUFFER_H_ */
