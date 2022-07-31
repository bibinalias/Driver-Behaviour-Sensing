#ifndef __HARSH_ACCELERATION_BRAKE_FUNCTION_WAKEUP_THRESHOLD_OPTIMIZATION_H
#define __HARSH_ACCELERATION_BRAKE_FUNCTION_WAKEUP_THRESHOLD_OPTIMIZATION_H
#include "stdint.h"
#include <math.h>

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)
uint8_t USER_Harsh_Acceleration_Brake_Function_Wakeup_Threshold_Optimize(void);
#endif
