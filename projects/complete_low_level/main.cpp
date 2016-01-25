#include "Uart.hpp"
#include "MotionControlSystem.h"
#include "delay.h"
#include "ActuatorsMgr.hpp"
#include "SensorMgr.h"
#include "voltage_controller.hpp"



int main(void)
{
	Delay_Init();
	Uart<1> serial;
	Uart<2> serial_ax;
	serial.init(115200);
	serial_ax.init(9600);

	MotionControlSystem* motionControlSystem = &MotionControlSystem::Instance(); // motionControlSystem est tout simplement un pointeur vers une référence d'un objet de type MotionControlSystem #TRIVIAL #USELESS
	motionControlSystem->init();
	ActuatorsMgr* actuatorsMgr = &ActuatorsMgr::Instance();
	SensorMgr* sensorMgr = &SensorMgr::Instance();
	//voltage_controller* voltage = &voltage_controller::Instance();

	char order[64];//Permet le stockage du message reçu par la liaison série

	bool translation = true;//permet de basculer entre les réglages de cte d'asserv en translation et en rotation

	while(1)
	{
		sensorMgr->refresh();

		uint8_t tailleBuffer = serial.available();

		if (tailleBuffer && tailleBuffer < RX_BUFFER_SIZE - 1)
		{
			serial.read(order);
			serial.printfln("_");//Acquittement

			if(!strcmp("?",order))				//Ping
			{
				serial.printfln("0");
			}
			else if(!strcmp("f",order))			//Indiquer l'état du mouvement du robot
			{
				serial.printfln("%d", motionControlSystem->isMoving());//Robot en mouvement ou pas ?
				serial.printfln("%d", motionControlSystem->isMoveAbnormal());//Cet état du mouvement est il anormal ?
			}
			else if(!strcmp("?xyo",order))		//Indiquer la position du robot (en mm et radians)
			{
				serial.printfln("%f", motionControlSystem->getX());
				serial.printfln("%f", motionControlSystem->getY());
				serial.printfln("%f", motionControlSystem->getAngleRadian());
			}
			else if(!strcmp("d", order))		//Ordre de déplacement rectiligne (en mm)
			{
				int deplacement = 0;
				serial.read(deplacement);
				serial.printfln("_");//Acquittement
				motionControlSystem->orderTranslation(deplacement);
			}
			else if(!strcmp("t", order))		//Ordre de rotation via un angle absolu (en radians)
			{
				float angle = motionControlSystem->getAngleRadian();
				serial.read(angle);
				serial.printfln("_");//Acquittement
				motionControlSystem->orderRotation(angle, MotionControlSystem::FREE);
			}

			else if(!strcmp("tor", order))  // Ordre de rotation seulement à DROITE (pour ne pas perdre le sable)
			{
				float angle = motionControlSystem->getAngleRadian();
				serial.read(angle);
				serial.printfln("_");//Acquittement
				motionControlSystem->orderRotation(angle, MotionControlSystem::ANTITRIGO);
			}



			else if(!strcmp("tol", order))  // Ordre de rotation seulement à GAUCHE (pour ne pas perdre le sable)
			{
				float angle = motionControlSystem->getAngleRadian();
				serial.read(angle);
				serial.printfln("_");//Acquittement
				motionControlSystem->orderRotation(angle, MotionControlSystem::TRIGO);
			}


			else if(!strcmp("t3", order))		//Ordre de rotation via un angle relatif (en radians)
			{
				float angle_actuel = motionControlSystem->getAngleRadian(), delta_angle = 0;
				serial.read(delta_angle);
				serial.printfln("_");
				motionControlSystem->orderRotation(angle_actuel + delta_angle, MotionControlSystem::FREE);
			}
			else if(!strcmp("r", order))		//Ordre de rotation via un angle relatif (en degrés)
			{
				float angle_actuel = motionControlSystem->getAngleRadian()*180/PI, delta_angle = 0;
				serial.read(delta_angle);
				serial.printfln("_");
				motionControlSystem->orderRotation((angle_actuel + delta_angle)*PI/180, MotionControlSystem::FREE);
			}
			else if(!strcmp("stop",order))		//Ordre d'arrêt (asservissement à la position actuelle)
			{
				motionControlSystem->stop();
			}
			else if(!strcmp("us",order))		//Indiquer la distance mesurée par le capteur à ultrason
			{
				serial.printfln("%d", sensorMgr->getSensorDistance());//en mm
			}
			else if(!strcmp("j",order))			//Indiquer l'état du jumper (0='en place'; 1='dehors')
			{
				serial.printfln("%d", sensorMgr->isJumperOut());
			}
			else if(!strcmp("ct0",order))		//Désactiver l'asservissement en translation
			{
				motionControlSystem->enableTranslationControl(false);
			}
			else if(!strcmp("ct1",order))		//Activer l'asservissement en translation
			{
				motionControlSystem->enableTranslationControl(true);
			}
			else if(!strcmp("cr0",order))		//Désactiver l'asservissement en rotation
			{
				motionControlSystem->enableRotationControl(false);
			}
			else if(!strcmp("cr1",order))		//Activer l'asservissement en rotation
			{
				motionControlSystem->enableRotationControl(true);
			}
			else if(!strcmp("cx",order))		//Régler la composante x de la position (en mm)
			{
				float x;
				serial.read(x);
				serial.printfln("_");//Acquittement
				motionControlSystem->setX(x);
			}
			else if(!strcmp("cy",order))		//Régler la composante y de la position (en mm)
			{
				float y;
				serial.read(y);
				serial.printfln("_");//Acquittement
				motionControlSystem->setY(y);
			}
			else if(!strcmp("co",order))		//Régler l'orientation du robot (en radians)
			{
				float o;
				serial.read(o);
				serial.printfln("_");//Acquittement
				motionControlSystem->setOriginalAngle(o);
			}
			else if(!strcmp("ctv",order))   //Régler la vitesse de translation
			{
				float speed = 0; // unité de speed : mm/s
				serial.read(speed);
				serial.printfln("_");
				motionControlSystem->setTranslationSpeed(speed);
			}
			else if(!strcmp("crv",order))  //Régler la vitesse de rotation
			{
				float speedRotation = 0;
				serial.read(speedRotation);
				serial.printfln("_");
				motionControlSystem->setRotationSpeed(speedRotation);
			}





/*			 __________________
 * 		   *|                  |*
 *		   *|COMMANDES DE DEBUG|*
 *		   *|__________________|*
 */
			else if(!strcmp("!",order))//Test quelconque
			{

			}
			else if(!strcmp("oxy",order))
			{
				serial.printfln("x=%f\r\ny=%f", motionControlSystem->getX(), motionControlSystem->getY());
				serial.printfln("o=%f", motionControlSystem->getAngleRadian());
			}
			else if (!strcmp("at", order))	// Commute l'asservissement en translation
			{
				static bool asservTranslation = false;
				motionControlSystem->enableTranslationControl(asservTranslation);
				serial.printfln("l'asserv en translation est désormais");
				if (asservTranslation)
				{
					serial.printfln("asservi en translation");
				}
				else
				{
					serial.printfln("non asservi en translation");
				}
				asservTranslation = !asservTranslation;
			}
			else if (!strcmp("ar", order)) // Commute l'asservissement en rotation
			{
				static bool asservRotation = false;
				motionControlSystem->enableRotationControl(asservRotation);
				serial.printfln("l'asserv en rotation est désormais");
				if (asservRotation)
				{
					serial.printfln("asservi en rotation");
				}
				else
				{
					serial.printfln("non asservi en rotation");
				}
				asservRotation = !asservRotation;
			}

			else if(!strcmp("rp",order))//Reset position
			{
				motionControlSystem->resetPosition();
				serial.printfln("Reset position");
			}


			else if(!strcmp("testSpeed",order))
			{
				motionControlSystem->testSpeed();
			}
			else if(!strcmp("dtest",order))
			{
				int distance = 0;
				serial.printfln("Distance du test : (mm)");
				serial.read(distance);
				motionControlSystem->distanceTest = distance;
			}

			else if(!strcmp("continualTest",order))//Test long
			{
				motionControlSystem->longTestSpeed();
			}
			else if(!strcmp("testPosition",order))
			{
				motionControlSystem->testPosition();
			}




/**
 * 	Réglage des constantes d'asservissement
 */
			else if(!strcmp("toggle",order))//Bascule entre le réglage d'asserv en translation et en rotation
			{
				translation = !translation;
				if(translation)
					serial.printfln("reglage de la transation");
				else
					serial.printfln("reglage de la rotation");
			}
			else if(!strcmp("display",order))
			{
				float
					kp_t, ki_t, kd_t,	// Translation
					kp_r, ki_r, kd_r,	// Rotation
					kp_g, ki_g, kd_g,	// Vitesse gauche
					kp_d, ki_d, kd_d;	// Vitesse droite
				motionControlSystem->getTranslationTunings(kp_t, ki_t, kd_t);
				motionControlSystem->getRotationTunings(kp_r, ki_r, kd_r);
				motionControlSystem->getLeftSpeedTunings(kp_g, ki_g, kd_g);
				motionControlSystem->getRightSpeedTunings(kp_d, ki_d, kd_d);
				serial.printfln("trans : kp= %g ; ki= %g ; kd= %g", kp_t, ki_t, kd_t);
				serial.printfln("rot   : kp= %g ; ki= %g ; kd= %g", kp_r, ki_r, kd_r);
				serial.printfln("gauche: kp= %g ; ki= %g ; kd= %g", kp_g, ki_g, kd_g);
				serial.printfln("droite: kp= %g ; ki= %g ; kd= %g", kp_d, ki_d, kd_d);
			}

			else if(!strcmp("autoasserv" ,order))// Commande pour le programme d'autoasserv (python)
			{
				float
					kp_g, kp_d, ki_g, ki_d, kd_g, kd_d;

				motionControlSystem->getLeftSpeedTunings(kp_g, ki_g, kd_g);
				motionControlSystem->getRightSpeedTunings(kp_d, ki_d, kd_d);

				motionControlSystem->printTracking();
				serial.printf("endtest");
			}


			else if(!strcmp("dts",order))//Delay To Stop
			{
				uint32_t delayToStop = 0;
				serial.printfln("Delay to stop ? (ms)");
				serial.read(delayToStop);
				motionControlSystem->setDelayToStop(delayToStop);
				serial.printfln("Delay to stop = %d", delayToStop);
			}

// ***********  TRANSLATION  ***********
			else if(!strcmp("kpt",order))
			{
				float kp, ki, kd;
				serial.printfln("kp_trans ?");
				motionControlSystem->getTranslationTunings(kp,ki,kd);
				serial.read(kp);
				motionControlSystem->setTranslationTunings(kp,ki,kd);
				serial.printfln("kp_trans = %g", kp);
			}
			else if(!strcmp("kdt",order))
			{
				float kp, ki, kd;
				serial.printfln("kd_trans ?");
				motionControlSystem->getTranslationTunings(kp,ki,kd);
				serial.read(kd);
				motionControlSystem->setTranslationTunings(kp,ki,kd);
				serial.printfln("kd_trans = %g", kd);
			}
			else if(!strcmp("kit",order))
			{
				float kp, ki, kd;
				serial.printfln("ki_trans ?");
				motionControlSystem->getTranslationTunings(kp,ki,kd);
				serial.read(ki);
				motionControlSystem->setTranslationTunings(kp,ki,kd);
				serial.printfln("ki_trans = %g", ki);
			}

// ***********  ROTATION  ***********
			else if(!strcmp("kpr",order))
			{
				float kp, ki, kd;
				serial.printfln("kp_rot ?");
				motionControlSystem->getRotationTunings(kp,ki,kd);
				serial.read(kp);
				motionControlSystem->setRotationTunings(kp,ki,kd);
				serial.printfln("kp_rot = %g", kp);
			}
			else if(!strcmp("kir",order))
			{
				float kp, ki, kd;
				serial.printfln("ki_rot ?");
				motionControlSystem->getRotationTunings(kp,ki,kd);
				serial.read(ki);
				motionControlSystem->setRotationTunings(kp,ki,kd);
				serial.printfln("ki_rot = %g", ki);
			}
			else if(!strcmp("kdr",order))
			{
				float kp, ki, kd;
				serial.printfln("kd_rot ?");
				motionControlSystem->getRotationTunings(kp,ki,kd);
				serial.read(kd);
				motionControlSystem->setRotationTunings(kp,ki,kd);
				serial.printfln("kd_rot = %g", kd);
			}

// ***********  VITESSE GAUCHE  ***********
			else if(!strcmp("kpg",order))
			{
				float kp, ki, kd;
				serial.printfln("kp_gauche ?");
				motionControlSystem->getLeftSpeedTunings(kp,ki,kd);
				serial.read(kp);
				motionControlSystem->setLeftSpeedTunings(kp,ki,kd);
				serial.printfln("kp_gauche = %g", kp);
			}
			else if(!strcmp("kig",order))
			{
				float kp, ki, kd;
				serial.printfln("ki_gauche ?");
				motionControlSystem->getLeftSpeedTunings(kp,ki,kd);
				serial.read(ki);
				motionControlSystem->setLeftSpeedTunings(kp,ki,kd);
				serial.printfln("ki_gauche = %g", ki);
			}
			else if(!strcmp("kdg",order))
			{
				float kp, ki, kd;
				serial.printfln("kd_gauche ?");
				motionControlSystem->getLeftSpeedTunings(kp,ki,kd);
				serial.read(kd);
				motionControlSystem->setLeftSpeedTunings(kp,ki,kd);
				serial.printfln("kd_gauche = %g", kd);
			}

// ***********  VITESSE DROITE  ***********
			else if(!strcmp("kpd",order))
			{
				float kp, ki, kd;
				serial.printfln("kp_droite ?");
				motionControlSystem->getRightSpeedTunings(kp,ki,kd);
				serial.read(kp);
				motionControlSystem->setRightSpeedTunings(kp,ki,kd);
				serial.printfln("kp_droite = %g", kp);
			}
			else if(!strcmp("kid",order))
			{
				float kp, ki, kd;
				serial.printfln("ki_droite ?");
				motionControlSystem->getRightSpeedTunings(kp,ki,kd);
				serial.read(ki);
				motionControlSystem->setRightSpeedTunings(kp,ki,kd);
				serial.printfln("ki_droite = %g", ki);
			}
			else if(!strcmp("kdd",order))
			{
				float kp, ki, kd;
				serial.printfln("kd_droite ?");
				motionControlSystem->getRightSpeedTunings(kp,ki,kd);
				serial.read(kd);
				motionControlSystem->setRightSpeedTunings(kp,ki,kd);
				serial.printfln("kd_droite = %g", kd);
			}



	/**
	 * 		Commandes de tracking des variables du système (débug)
	 */
			else if(!strcmp("trackAll",order))
			{
				motionControlSystem->printTrackingAll();
			}




/*			 ___________
 * 		   *|           |*
 *		   *|ACTIONNEURS|*
 *		   *|___________|*
 */

			else if(!strcmp("setallid",order))
			{
				actuatorsMgr->setAllID();
			}

			else if(!strcmp("fpr",order)) // Descente du bras droit aimanté (poissons)
			{
				actuatorsMgr->fishingRight();
			}

			else if(!strcmp("mpr",order))
			{
				actuatorsMgr->midPositionRight();
			}


			else if(!strcmp("rmd",order)) // rightMagnetsDown
			{
				actuatorsMgr->rightMagnetsDown();

			}
			else if(!strcmp("rfd",order))	// rightFingerDown
			{
				actuatorsMgr->rightFingerDown();

			}
			else if(!strcmp("rmu",order))	//rightMagnetsUp
			{
				actuatorsMgr->rightMagnetsUp();

			}
			else if(!strcmp("rfu",order))	//rightFingerUp
			{
				actuatorsMgr->rightFingerUp();

			}






			else if(!strcmp("fpl",order)) // Descente du bras gauche aimanté (poissons)
			{
				actuatorsMgr->fishingLeft();
			}

			else if(!strcmp("mpl",order))
			{
				actuatorsMgr->midPositionLeft();
			}

			else if(!strcmp("lmd",order)) // leftMagnetsDown
			{
				actuatorsMgr->leftMagnetsDown();

			}
			else if(!strcmp("lfd",order))	// leftFingerDown
			{
				actuatorsMgr->leftFingerDown();

			}
			else if(!strcmp("lmu",order))	//leftMagnetsUp
			{
				actuatorsMgr->leftMagnetsUp();

			}
			else if(!strcmp("lfu",order))	//leftFingerUp
			{
				actuatorsMgr->leftFingerUp();

			}

			else if(!strcmp("emr", order)) // permet de tester manuellement les positions des AX12
			{
				int position = 150;
				serial.printfln("Entrez angle");
				serial.read(position);
				serial.printfln("angle = %d", position);
				if(position >= 0 && position <= 300)
					actuatorsMgr->setAXposMagnetsRight(position);
				serial.printfln("done.");

			}

			else if(!strcmp("efr", order)) // permet de tester manuellement les positions des AX12
			{
				int position = 150;
				serial.printfln("Entrez angle");
				serial.read(position);
				serial.printfln("angle = %d", position);
				if(position >= 0 && position <= 300)
					actuatorsMgr->setAXposFreeRightFishes(position);
				serial.printfln("done.");

			}

			else if(!strcmp("eml", order)) // permet de tester manuellement les positions des AX12
			{
				int position = 150;
				serial.printfln("Entrez angle");
				serial.read(position);
				serial.printfln("angle = %d", position);
				if(position >= 0 && position <= 300)
					actuatorsMgr->setAXposMagnetsLeft(position);
				serial.printfln("done.");

			}

			else if(!strcmp("efl", order)) // permet de tester manuellement les positions des AX12
			{
				int position = 150;
				serial.printfln("Entrez angle");
				serial.read(position);
				serial.printfln("angle = %d", position);
				if(position >= 0 && position <= 300)
					actuatorsMgr->setAXposFreeLeftFishes(position);
				serial.printfln("done.");

			}

			else if(!strcmp("aif", order)) {
				actuatorsMgr->initialPositionFish();
			}

			else if(!strcmp("caxs", order)) { //commande de debug
				int speed = 100;
				serial.printfln("Entrez vitesse");
				serial.read(speed);
				actuatorsMgr->changeAXSpeed(speed);
				serial.printfln("Done");
			}

			else if(!strcmp("uoe",order))
			{ // test
				serial.printfln("Une fraise");
			}
			// Sinon, Ordre inconnu

			else
			{
				serial.printfln("Ordre inconnu");
			}

		}
#if DEBUG
		else if(tailleBuffer == RX_BUFFER_SIZE - 1)
		{
			serial.printfln("CRITICAL OVERFLOW !");
			motionControlSystem->enableTranslationControl(false);
			motionControlSystem->enableRotationControl(false);
			while(true)
				;
		}
#endif
	}
}

extern "C" {
//Interruption overflow TIMER4
void TIM4_IRQHandler(void) { //2kHz = 0.0005s = 0.5ms
	volatile static uint32_t i = 0, j = 0, k = 0;
	static MotionControlSystem* motionControlSystem = &MotionControlSystem::Instance();
	static voltage_controller* voltage = &voltage_controller::Instance();

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
		//Remise à 0 manuelle du flag d'interruption nécessaire
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		//Asservissement et mise à jour de la position
		motionControlSystem->updatePosition();
		motionControlSystem->control();

		if (i >= 10) { //5ms
			//Gestion de l'arrêt
			motionControlSystem->manageStop();
			i = 0;
		}

		if(j >= 5){ //2.5ms
			motionControlSystem->track();
			j=0;
		}

		if(k <= 2000)
		{
			//voltage->measure();
			k=0;
		}

		k++;
		i++;
		j++;
	}
}

void EXTI9_5_IRQHandler(void)
{
	static SensorMgr* sensorMgr = &SensorMgr::Instance();

	//Interruptions de l'ultrason de test
    if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
        sensorMgr->sensorInterrupt();

        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line6);
    }
}

}
