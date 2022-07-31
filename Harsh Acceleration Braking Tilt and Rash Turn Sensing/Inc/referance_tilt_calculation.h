#ifndef __REFERANCE_TILT_CALCULATION_H
#define __REFERANCE_TILT_CALCULATION_H


#include <math.h>


void USER_Referance_Tilt_Calculation(void);
void USER_Referance_Tilt_Calculation_without_Forward_axes(void);
void USER_Referance_Tilt_Calculation_with_Forward_axes(void);
void USER_Referance_Tilt_Calculation_without_Gravity_axes(void);


#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

#endif
