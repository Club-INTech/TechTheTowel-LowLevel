#ifndef MOTION_CONTROL_H_
#define MOTION_CONTROL_H_

#define DEBUG	1

#include "Singleton.hpp"
#include "Motor.h"
#include "pid.hpp"
#include <math.h>
#include "delay.h"
#include "misc.h"
#include "Counter.h"
#include <Uart.hpp>
#include "average.hpp"

#define PI 3.14159265

// 65,5mm diametre des roues
// ~1000 ticks par tour de roue
// 17cm �cartement des roues

#define TICK_TO_MM 0.2077			// unit� : mm/ticks
#define TICK_TO_RADIAN 0.0014468	// unit� : radians/ticks

#define AVERAGE_SPEED_SIZE	25

#if DEBUG
#define TRACKER_SIZE 		1000
#else
#define TRACKER_SIZE 		1
#endif

enum MOVING_DIRECTION {FORWARD, BACKWARD, NONE};

extern Uart<1> serial;

class MotionControlSystem : public Singleton<MotionControlSystem>
{
private:
	Motor leftMotor;
	Motor rightMotor;

/*
 * 		D�finition des variables d'�tat du syst�me (position, vitesse, consigne, ...)
 *
 * 		Les unit�s sont :
 * 			Pour les distances		: ticks
 * 			Pour les vitesses		: ticks/seconde
 * 			Pour les acc�l�rations	: ticks/seconde^2
 * 			Ces unit�s seront vraies pour une fr�quence d'asservissement de 2kHz,
 * 			si l'on souhaite changer la fr�quence d'asservissement il faut adapter le calcul de la vitesse
 * 			autrement les unit�es ci-dessus ne seront plus valables.
 */


	//	Asservissement en vitesse du moteur droit
	PID rightSpeedPID;
	volatile int32_t rightSpeedSetpoint;	// ticks/seconde
	volatile int32_t currentRightSpeed;		// ticks/seconde
	volatile int32_t rightPWM;

	//	Asservissement en vitesse du moteur gauche
	PID leftSpeedPID;
	volatile int32_t leftSpeedSetpoint;		// ticks/seconde
	volatile int32_t currentLeftSpeed;		// ticks/seconde
	volatile int32_t leftPWM;

	//	Asservissement en position : translation
	PID translationPID;
	volatile int32_t translationSetpoint;	// ticks
	volatile int32_t currentDistance;		// ticks
	volatile int32_t translationSpeed;		// ticks/seconde

	//	Asservissement en position : rotation
	PID rotationPID;
	volatile int32_t rotationSetpoint;		// ticks
	volatile int32_t currentAngle;			// ticks
	volatile int32_t rotationSpeed;			// ticks/seconde

	//	Limitation de vitesse
	volatile int32_t maxSpeed;

	//	Limitation d'acc�l�ration
	volatile int32_t maxAcceleration;

	//	Pour faire de jolies courbes de r�ponse du syst�me, la vitesse moyenne c'est mieux !
	Average<int32_t, AVERAGE_SPEED_SIZE> averageLeftSpeed;
	Average<int32_t, AVERAGE_SPEED_SIZE> averageRightSpeed;

	// D�finit la vitesse � utiliser pour les tests d'asservissement (testSpeed et testSpeedReverse)
	volatile int32_t speedTest;


/*
 * 	Variables de positionnement haut niveau (exprimm�es en unit�s pratiques ^^)
 *
 * 	Toutes ces variables sont initialis�es � 0. Elles doivent donc �tre r�gl�es ensuite
 * 	par le haut niveau pour correspondre � son syst�me de coordonn�es.
 * 	Le bas niveau met � jour la valeur de ces variables mais ne les utilise jamais pour
 * 	lui m�me, il se contente de les transmettre au haut niveau.
 */
	volatile float x;				// Positionnement 'x' (mm)
	volatile float y;				// Positionnement 'y' (mm)
	volatile float originalAngle;	// Angle d'origine	  (radians)
	// 'originalAngle' repr�sente un offset ajout� � l'angle courant pour que nos angles en radians co�ncident avec la repr�sentation haut niveau des angles.


	// Variables d'�tat du mouvement
	volatile bool moving;
	volatile MOVING_DIRECTION direction;
	volatile bool moveAbnormal;

	// Variables d'activation des diff�rents PID
	volatile bool translationControlled;
	volatile bool rotationControlled;
	volatile bool leftSpeedControlled;
	volatile bool rightSpeedControlled;

	// Variables de r�glage de la d�tection de blocage physique
	unsigned int delayToStop;//En ms
	//Nombre de ticks de tol�rance pour consid�rer qu'on est arriv� � destination
	int toleranceTranslation;
	int toleranceRotation;


	/*
	 * Dispositif d'enregistrement de l'�tat du syst�me pour permettre le d�bug
	 * La valeur de TRACKER_SIZE d�pend de la valeur de DEBUG.
	 */
	struct trackerType
	{
		float x;
		float y;
		float angle;

		int consigneVitesseGauche;
		int vitesseGaucheCourante;
		int vitesseMoyenneGauche;
		int pwmGauche;

		int consigneVitesseDroite;
		int vitesseDroiteCourante;
		int vitesseMoyenneDroite;
		int pwmDroit;

		int consigneTranslation;
		int translationCourante;
		int consigneVitesseTranslation;

		int consigneRotation;
		int rotationCourante;
		int consigneVitesseRotation;
	};

	trackerType trackArray[TRACKER_SIZE];
	unsigned int trackerCursor;

	bool isPhysicallyStopped();//Indique si le robot est immobile.


public:
	MotionControlSystem();

	void init();

	void control();
	void updatePosition();
	void manageStop();

	void track();//Stock les valeurs de d�bug
	void printTrackingAll();//Affiche l'int�gralit� du tableau de tracking
	void printTracking(); // Envoie des donn�es pour l'asserv auto
	void printPosition();
	void resetTracking();// Reset le tableau de tracking

	int getPWMTranslation() const;
	int getPWMRotation() const;
	int getTranslationGoal() const;
	int getRotationGoal() const;
	int getLeftEncoder() const;
	int getRightEncoder() const;

	void enable(bool);
	void enableTranslationControl(bool);
	void enableRotationControl(bool);

	void orderTranslation(int32_t);
	void orderRotation(float);
	void orderRawPwm(Side,int16_t);
	void stop();
	static int32_t optimumAngle(int32_t,int32_t);

	void setTranslationTunings(float, float, float);
	void setRotationTunings(float, float, float);
	void setLeftSpeedTunings(float, float, float);
	void setRightSpeedTunings(float, float, float);
	void getTranslationTunings(float &,float &,float &) const;
	void getRotationTunings(float &,float &,float &) const;
	void getLeftSpeedTunings(float &, float &, float &) const;
	void getRightSpeedTunings(float &, float &, float &) const;

	float getAngleRadian() const;
	void setOriginalAngle(float);
	float getX() const;
	float getY() const;
	void setX(float);
	void setY(float);
	void resetPosition(void);
	float getBalance() const;
	void setBalance(float newBalance);
	int16_t getMaxPWMtranslation() const;
	int16_t getMaxPWMrotation() const;
	void setMaxPWMtranslation(int16_t);
	void setMaxPWMrotation(int16_t);
	void setDelayToStop(uint32_t);
	void setPWM();

	bool isMoving() const;
	bool isMoveAbnormal() const;
	MOVING_DIRECTION getMovingDirection() const;

	void setTestSpeed(int32_t);
	void testSpeed();
	void testSpeedReverse();
	void longTestSpeed();
	void testPosition();

};

#endif /* MOTION_CONTROL_H_ */
