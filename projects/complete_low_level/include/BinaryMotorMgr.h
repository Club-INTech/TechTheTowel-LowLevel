/**
 * BinaryMotorMgr.h
 *
 */

#ifndef __BINARYMOTORMGR_h__
#define __BINARYMOTORMGR_h__

#include "stm32f4xx.h"
#include "safe_enum.hpp"
#include "utils.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "Motor.h"

class BinaryMotorMgr {
private:
	;

public:
	BinaryMotorMgr();
	void run(Direction, Side);
	void stop(Direction, Side);
};

#endif
