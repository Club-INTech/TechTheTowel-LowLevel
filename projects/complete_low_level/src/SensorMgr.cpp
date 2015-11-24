#include "SensorMgr.h"

/*		PINS DES CAPTEURS
 *
 * 	ULTRASONS:
 * 		Avant Droit   :	PA6
 * 		Avant Gauche  :	PA4
 * 		Arrière Droit :	PA7
 * 		Arrière Gauche:	PB1
 *
 * 	CONTACTEURS:
 * 		Monte-plot		: PC15
 * 		Gobelet Droit	: PD9
 * 		Gobelet Gauche	: PD11
 */


SensorMgr::SensorMgr():
	ultrason()
{
	lastRefreshTime = 0;
	refreshDelay = 13;//(ms)

	/* Set variables used */
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/*
	 * Initialisation des pins des capteurs de contact
	 */

	GPIO_StructInit(&GPIO_InitStruct); //Remplit avec les valeurs par défaut

	//Jumper (PC9)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//Active l'horloge du port C

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);




/*     ________________________________
	 *|								   |*
	 *|Initialisation des interruptions|*
	 *|________________________________|*
*/


	/*
	 * Capteur de Test : PA6
	 */

	/* Activation de l'horloge du port GPIOA */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Activation de l'horloge du SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*Réglages de la pin*/
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Tell system that you will use PA6 for EXTI_Line6 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);

	/* PA6 is connected to EXTI_Line6 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line6;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = DISABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);

	/* Add IRQ vector to NVIC */
	/* PA6 is connected to EXTI_Line6, which has EXTI9_5_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);

	ultrason.init(GPIOA, GPIO_InitStruct, EXTI_InitStruct);//On donne les paramètres de la pin et de l'interruption au capteur pour qu'il puisse les modifier sans faire d'erreur

}

/*
 * Fonction de mise à jour des capteurs à ultrason
 */
void SensorMgr::refresh()
{
	currentTime = Millis();

	if(currentTime - lastRefreshTime >= refreshDelay)
	{
		ultrason.refresh();
		lastRefreshTime = currentTime;
	}
}


/*
 * Fonctions d'interruption des capteurs à ultrason
 */

void SensorMgr::sensorInterrupt(){
	ultrason.interruption();
}


/*
 * Fonctions de récupération de la distance mesurée
 */

int SensorMgr::getSensorDistance() {
	return ultrason.value();
}



/*
 * Fonctions de récupération de l'état des capteurs de contact et du jumper
 */

bool SensorMgr::isJumperOut() const{
	return !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9);
}
