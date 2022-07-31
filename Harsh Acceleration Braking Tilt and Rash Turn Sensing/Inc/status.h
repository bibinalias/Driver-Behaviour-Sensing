#ifndef __STATUS_H
#define __STATUS_H

#include "stm32f0xx_hal.h"
#include "math.h"

enum Rash_Turn{
	NO_RASH_TURN,
	RASH_TURN_RIGHT,
	RASH_TURN_LEFT,
};
enum Tilt{
	NO_TILT,
	TILT,
	TILT_SIDE_AXIS,
	TILT_FORWARD_AXIS,
	TILT_GRAVITY_AXIS

};
enum Referance_Tilt_function
{
	NO_TILT_FUNCTION,
	NO_GRAVITY_AXES,
	ONLY_GRAVITY_AXES,
	GRAVITY_AXES_PLUS_FORWARD_AXES
};
enum Acceleration_Brake {
	NOTHING,
	HARSH_ACCELERATION,
	HARSH_BRAKING

};
enum AXES {X,
	Y,
	Z,
	AXES_NOT_SET
};
enum Wakeup_Device_Mode{
	SLEEP,
	FORWARD_AXIS,
	ACCELERATION,
	ORIENTATION
};




enum Function_Exicution{


	NOT_EXICUTED,
	EXICUTED

};


enum Sign{
	MINUS=-1,
	NO_SIGN,
	PLUS,
	SIGN_NOT_SET


};

struct out_xl
	{
		int32_t x;
		int32_t y;
		int32_t z;

	};

struct out_xl_mg
	{
		float_t x;
		float_t y;
		float_t z;

	};

struct out_g
	{
		int32_t x;
		int32_t y;
		int32_t z;

	};

struct axes_with_sign
{
	enum AXES axes;
	enum Sign sign;
};




#endif
