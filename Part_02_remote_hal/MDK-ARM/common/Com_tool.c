#include "Com_tool.h"


//限幅函数
int16_t Com_Limit(int16_t value, int16_t min, int16_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
