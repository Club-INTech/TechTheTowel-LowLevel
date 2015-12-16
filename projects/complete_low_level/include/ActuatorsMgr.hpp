#ifndef ACTUATORSMGR_HPP
#define ACTUATORSMGR_HPP

#include <ax12.hpp>
#include <Uart.hpp>
#include <Singleton.hpp>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

extern Uart<1> serial;

#define position1 100
#define position2 200



// on définit les différents angles utilisés pour le coté gauche et le coté droit
//TODO valeurs à modifier
#define fishingRightPosition 0
#define initialRightPosition 240
#define middleRightPosition 0
#define fingerRightInitialPosition 0
#define fingerRightFreePosition 0
#define passingRightPosition 0


// Coté gauche
#define fishingLeftPosition 155
#define initialLeftPosition 80
#define middleLeftPosition 130
#define fingerLeftInitialPosition 100
#define fingerLeftFreePosition 45
#define passingLeftPosition 100

class ActuatorsMgr : public Singleton<ActuatorsMgr>
{
private:
	typedef Uart<2> serial_ax; // On utilise le port série 2 de la stm32
	AX<serial_ax>* ax12MagnetsRight; // Bras pour pecher les poissons
	AX<serial_ax>* ax12FingerRightFishes; // Doigt pour décrocher les poissons
	AX<serial_ax>* ax12MagnetsLeft; // Bras gauche pour pecher les poissons
	AX<serial_ax>* ax12FingerLeftFishes; // Doigt gauche pour décrocher les poissons


public:
	ActuatorsMgr()
	{
		ax12MagnetsRight = new AX<serial_ax>(0,0,1023); // (ID, Angle_min, Angle_Max)
		ax12FingerRightFishes = new AX<serial_ax>(1,0,1023);
		ax12MagnetsRight->init();
		ax12MagnetsLeft = new AX<serial_ax>(2,0,1023); // (ID, Angle_min, Angle_Max)
		ax12FingerLeftFishes = new AX<serial_ax>(3,0,1023);
		ax12MagnetsLeft->init();
	}

	~ActuatorsMgr()
	{
		delete(ax12MagnetsRight);
		delete(ax12FingerRightFishes);
		delete(ax12MagnetsLeft);
		delete(ax12FingerLeftFishes);
	}

	void setAllID(){ //Permet de regler les différents AX12
		int i;
		serial.printfln("Reglage des ID des AX12");
		serial.printfln("(brancher un AX12 a la fois)");
		serial.printf("\n");

		serial.printfln("Brancher ax12MagnetsRight");
		serial.read(i);
		ax12MagnetsRight->initIDB(0);
		serial.printfln("done");

		serial.printfln("Brancher ax12FreeRightFishes");
		serial.read(i);
		ax12FingerRightFishes->initIDB(1);
		serial.printfln("done");

		serial.printfln("Brancher ax12MagnetsLeft");
		serial.read(i);
		ax12MagnetsLeft->initIDB(2);
		serial.printfln("done");

		serial.printfln("Brancher ax12FreeLeftFishes");
		serial.read(i);
		ax12FingerLeftFishes->initIDB(3);
		serial.printfln("done");

		serial.printfln("Fin du reglage");
	}




	void fishingRight() {
		ax12MagnetsRight->goTo(fishingRightPosition);
	}

	void fishingLeft() {
		ax12MagnetsLeft->goTo(fishingLeftPosition);
	}

	void midPositionRight() {
		ax12MagnetsRight->goTo(middleRightPosition);
	}

	void midPositionLeft() {
		ax12MagnetsLeft->goTo(middleLeftPosition);
	}

	void freeRightFishes() {
		ax12MagnetsRight->goTo(passingRightPosition);
		Delay(500);
		ax12FingerRightFishes->goTo(fingerRightFreePosition);
		Delay(500);
		ax12MagnetsRight->goTo(initialRightPosition);
		Delay(500);
		ax12FingerRightFishes->goTo(fingerRightInitialPosition);
		Delay(500);
	}

	void freeLeftFishes() {
		ax12MagnetsLeft->goTo(passingLeftPosition);
		Delay(500);
		ax12FingerLeftFishes->goTo(fingerLeftFreePosition);
		Delay(500);
		ax12MagnetsLeft->goTo(initialLeftPosition);
		Delay(500);
		ax12FingerLeftFishes->goTo(fingerLeftInitialPosition);
		Delay(500);
	}

	void initialPositionFish() { //pour remettre AX12 dans leurs positions initiales
		ax12MagnetsLeft->goTo(initialLeftPosition);
		ax12MagnetsRight->goTo(initialRightPosition);
		ax12FingerLeftFishes->goTo(fingerLeftInitialPosition);
		ax12FingerRightFishes->goTo(fingerRightInitialPosition);
	}

	void setAXposMagnetsRight(int position) { // pour définir manuellement 1 position
		ax12MagnetsRight->goTo(position);
	}
	void setAXposMagnetsLeft(int position) { // pour définir manuellement 1 position
		ax12MagnetsLeft->goTo(position);
	}
	void setAXposFreeRightFishes(int position) { // pour définir manuellement 1 position
		ax12FingerRightFishes->goTo(position);
	}
	void setAXposFreeLeftFishes(int position) { // pour définir manuellement 1 position
		ax12FingerLeftFishes->goTo(position);
	}

	void changeAXSpeed(int speed)
	{
		ax12MagnetsLeft->changeSpeed(speed);
		ax12FingerLeftFishes->changeSpeed(speed);
		ax12MagnetsRight->changeSpeed(speed);
		ax12FingerRightFishes->changeSpeed(speed);
	}
};

#endif /* ACTUATORSMGR_HPP */
