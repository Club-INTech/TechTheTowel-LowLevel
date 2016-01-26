/*
 * BinaryMotor.cpp
 *
 * Classe de gestion des moteurs sans PWM
 *
 */

#include "BinaryMotorMgr.h"

BinaryMotorMgr::BinaryMotorMgr() {

		/**
		 * Configuration des pins :
		 * Porte Gauche ouverture : PE9
		 * Porte gauche fermeture : PE11
		 * Porte Droite ouverture : PE13
		 * Porte droite fermeture : PE15
		 *
		 */

	GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_StructInit(&GPIO_InitStruct); //Remplit avec les valeurs par défaut
		// Active l'horloge du port D
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);


			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_Init(GPIOD, &GPIO_InitStruct);

			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_Init(GPIOD, &GPIO_InitStruct);


			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_Init(GPIOD, &GPIO_InitStruct);

			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_Init(GPIOD, &GPIO_InitStruct);




}

void BinaryMotorMgr::run(Direction dir, Side side) {
	if (side == Side::LEFT) {
		if (dir == Direction::FORWARD) {
			GPIO_SetBits(GPIOD, GPIO_Pin_9);
		} else {
			GPIO_SetBits(GPIOD, GPIO_Pin_11);
		}
	} else {
		if (dir == Direction::FORWARD) {
			GPIO_SetBits(GPIOD, GPIO_Pin_13);

		} else {
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
		}
	}
}

void BinaryMotorMgr::stop(Direction dir, Side side) {
	if (side == Side::LEFT) {
		if (dir == Direction::FORWARD) {
			GPIO_ResetBits(GPIOD, GPIO_Pin_9);
		} else {
			GPIO_ResetBits(GPIOD, GPIO_Pin_11);
		}
	} else {
		if (dir == Direction::FORWARD) {
			GPIO_ResetBits(GPIOD, GPIO_Pin_13);

		} else {
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		}
	}
}
