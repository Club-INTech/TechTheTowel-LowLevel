/**
 * BinaryMotorMgr.hpp
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
#include "Singleton.hpp"



class BinaryMotorMgr : public Singleton<BinaryMotorMgr> {

public:
	BinaryMotorMgr();

	void runAxisLeft();
	void runAxisRight();
	void stopAxisLeft();
	void stopAxisRight();

	void runForwardLeft();
	void runBackwardLeft();
	void runForwardRight();
	void runBackwardRight();

	void stopLeftDoor();
	void stopRightDoor();


};

#endif
