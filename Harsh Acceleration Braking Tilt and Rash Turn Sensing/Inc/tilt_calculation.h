#ifndef __TILT_CALCULATION_H
#define __TILT_CALCULATION_H


#include <math.h>


enum Tilt USER_Tilt_Calculation(void);

#define TILT_ITRATION_TRESHOLD 5
#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

#endif
