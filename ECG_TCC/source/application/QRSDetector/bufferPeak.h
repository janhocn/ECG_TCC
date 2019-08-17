#ifndef USER_BUFFERPEAK_H_
#define USER_BUFFERPEAK_H_
#include "Peak.h"
#include "arm_math.h"
typedef struct{
    uint32_t head;
    uint32_t size;
    uint32_t inserts;
    Peak_t * data;
} buffPeak;

void initBufferPeak(int32_t size, buffPeak * buffer);
void cleanupBufferPeak(buffPeak * buffer);
void insertToBufferPeak(Peak_t data, buffPeak * buffer);
void movePointerBufferPeak(buffPeak * buffer);
Peak_t getPreviousPeak(int32_t previousN, buffPeak * buffer);
Peak_t getHeadPeak(buffPeak * buffer);
int32_t getAverageRRPeak(buffPeak * buffer, int32_t goback);


#endif /* USER_BUFFERPEAK_H_ */
