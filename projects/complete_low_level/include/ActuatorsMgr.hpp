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

/* Actionneurs poissons :
	 * ax12magnets : bras aimanté pour pecher
	 * 		Position initiale : 150
	 * 		Position peche : 240
	 * 		Position intermédiaire :200
	 *
	 * 	ax12freefishes : Doigt pour libérer les poissons
	 * 		Position initiale : 150
	 * 		Position ouverture : 75
	 *
	 */

#define fishingPosition 240
#define initialPosition 150
#define middlePosition 200
#define openingPosition 75


class ActuatorsMgr : public Singleton<ActuatorsMgr>
{
private:
	typedef Uart<2> serial_ax; // On utilise le port série 2 de la stm32
	AX<serial_ax>* ax12magnets; // Bras pour pecher les poissons
	AX<serial_ax>* ax12freefishes; // Doigt pour décrocher les poissons


public:
	ActuatorsMgr()
	{
		ax12magnets = new AX<serial_ax>(0,0,1023); // (ID, Angle_min, Angle_Max)
		ax12freefishes = new AX<serial_ax>(1,0,1023);
		ax12magnets->init();
	}

	~ActuatorsMgr()
	{
		delete(ax12magnets);
		delete(ax12freefishes);
	}

	void setAllID(){
		int i;
		serial.printfln("Reglage des ID des AX12");
		serial.printfln("(brancher un AX12 a la fois)");
		serial.printf("\n");

		serial.printfln("Brancher AX12magnets");
		serial.read(i);
		ax12magnets->initIDB(0);
		serial.printfln("done");

		serial.printfln("Brancher AX12freefishes");
		serial.read(i);
		ax12freefishes->initIDB(1);
		serial.printfln("done");

		serial.printfln("Fin du reglage");
	}

	void testMouvement() {
		ax12magnets->goTo(position1);
		Delay(1000);
		ax12magnets->goTo(position2);
		Delay(1000);
		ax12freefishes->goTo(position1);
		Delay(1000);
		ax12freefishes->goTo(position2);
	}



	void fishing() {
		ax12magnets->goTo(fishingPosition);
	}

	void midPosition() {
		ax12magnets->goTo(middlePosition);
	}

	void freefishes() {
		ax12freefishes->goTo(openingPosition);
		Delay(500);
		ax12magnets->goTo(initialPosition);
		Delay(500);
		ax12freefishes->goTo(initialPosition);
	}

	void setAXpos(int position) { // pour définir manuellement 1 position
		ax12magnets->goTo(position);
	}
};

#endif /* ACTUATORSMGR_HPP */
