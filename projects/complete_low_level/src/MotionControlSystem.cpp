#include "MotionControlSystem.h"

MotionControlSystem::MotionControlSystem(): leftMotor(Side::LEFT), rightMotor(Side::RIGHT),
	rightSpeedPID(&currentRightSpeed, &rightPWM, &rightSpeedSetpoint),
	leftSpeedPID(&currentLeftSpeed, &leftPWM, &leftSpeedSetpoint),
	translationPID(&currentDistance, &translationSpeed, &translationSetpoint),
	rotationPID(&currentAngle, &rotationSpeed, &rotationSetpoint),
	averageLeftSpeed(), averageRightSpeed()
{
	translationControlled = true;
	rotationControlled = true;
	leftSpeedControlled = true;
	rightSpeedControlled = true;

	originalAngle = 0.0;
	rotationSetpoint = 0;
	translationSetpoint = 0;
	leftSpeedSetpoint = 0;
	rightSpeedSetpoint = 0;
	x = 0;
	y = 0;
	moving = false;
	moveAbnormal = false;
	direction = NONE;

	leftSpeedPID.setOutputLimits(-255,255);
	rightSpeedPID.setOutputLimits(-255,255);

	maxSpeed =3000; //Vitesse maximum, des moteurs (avec une marge au cas où on s'amuse à faire forcer un peu la bestiole).
	maxAcceleration = 15;

	delayToStop = 100;
	toleranceTranslation = 50;
	toleranceRotation = 25;

	translationPID.setTunings(15, 0, 10);
	rotationPID.setTunings(16,0,10);
	leftSpeedPID.setTunings(0.01, 0.00005, 0.01);
	rightSpeedPID.setTunings(0.01, 0.00005, 0.01);

	speedTest = 1000;
}

void MotionControlSystem::init() {
/**
 * Initialisation moteurs et encodeurs
 */
	Motor::initPWM();
	Counter();


/**
 * Initialisation de la boucle d'asservissement (TIMER 4)
 */
	NVIC_InitTypeDef NVIC_InitStructure;
	//Configuration et activation de l'interruption
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//Activation de l'horloge du TIMER 4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//Configuration du timer
	//TIM4CLK = HCLK / 2 = SystemCoreClock /2 = 168MHz/2 = 84MHz
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 84 MHz Clock down to 1 MHz
	TIM_TimeBaseStructure.TIM_Period = 500 - 1; // 1 MHz down to 2 KHz : fréquence d'asservissement de 2kHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	//Active l'asservissement
	enable(true);
}

void MotionControlSystem::enable(bool enable) {
	if (enable) {
		TIM_Cmd(TIM4, ENABLE); //Active la boucle d'asservissement
	} else {
		TIM_Cmd(TIM4, DISABLE); //Désactive la boucle d'asservissement
		stop();
	}
}

void MotionControlSystem::enableTranslationControl(bool enabled) {
	translationControlled = enabled;
}
void MotionControlSystem::enableRotationControl(bool enabled) {
	rotationControlled = enabled;
}

void MotionControlSystem::control()
{
	// Pour le calcul de la vitesse instantanée :
	static int32_t previousLeftTicks = 0;
	static int32_t previousRightTicks = 0;

	// Pour le calcul de l'accélération intantanée :
	static int32_t previousLeftSpeedSetpoint = 0;
	static int32_t previousRightSpeedSetpoint = 0;

	/*
	 * Comptage des ticks de la roue droite
	 * Cette codeuse est connectée à un timer 16bit
	 * on subit donc un overflow/underflow de la valeur des ticks tous les 7 mètres environ
	 * ceci est corrigé de manière à pouvoir parcourir des distances grandes sans devenir fou en chemin (^_^)
	 */
	static int32_t lastRawRightTicks = 0;	//On garde en mémoire le nombre de ticks obtenu au précédent appel
	static int rightOverflow = 0;			//On garde en mémoire le nombre de fois que l'on a overflow (négatif pour les underflow)

	int32_t rawRightTicks = Counter::getRightValue();	//Nombre de ticks avant tout traitement

	if (lastRawRightTicks - rawRightTicks > 32768)		//Détection d'un overflow
		rightOverflow++;
	else if(lastRawRightTicks - rawRightTicks < -32768)	//Détection d'un underflow
		rightOverflow--;

	lastRawRightTicks = rawRightTicks;

	int32_t rightTicks = rawRightTicks + rightOverflow*65535;	//On calcule le nombre réel de ticks

	/*
	 * Comptage des ticks de la roue gauche
	 * ici on est sur un timer 32bit, pas de problème d'overflow sauf si on tente de parcourir plus de 446km...
	 */
	int32_t leftTicks = Counter::getLeftValue();


	currentLeftSpeed = (leftTicks - previousLeftTicks)*2000; // (nb-de-tick-passés)*(freq_asserv) (ticks/sec)
	currentRightSpeed = (rightTicks - previousRightTicks)*2000;

	previousLeftTicks = leftTicks;
	previousRightTicks = rightTicks;

	averageLeftSpeed.add(currentLeftSpeed);
	averageRightSpeed.add(currentRightSpeed);

	currentLeftSpeed = averageLeftSpeed.value(); // On utilise pour l'asserv la valeur moyenne des dernieres current Speed
	currentRightSpeed = averageRightSpeed.value();

	currentDistance = (leftTicks + rightTicks) / 2;
	currentAngle = (rightTicks - leftTicks) / 2;

	if(translationControlled)
		translationPID.compute();	// Actualise la valeur de 'translationSpeed'
	if(rotationControlled)
		rotationPID.compute();		// Actualise la valeur de 'rotationSpeed'

	leftSpeedSetpoint = translationSpeed - rotationSpeed;
	rightSpeedSetpoint = translationSpeed + rotationSpeed;


	// Limitation de l'accélération du moteur gauche
	if(leftSpeedSetpoint - previousLeftSpeedSetpoint > maxAcceleration)
	{
		leftSpeedSetpoint = previousLeftSpeedSetpoint + maxAcceleration;
	}
	else if(leftSpeedSetpoint - previousLeftSpeedSetpoint < -maxAcceleration)
	{
		leftSpeedSetpoint = previousLeftSpeedSetpoint - maxAcceleration;
	}

	// Limitation de l'accélération du moteur droit
	if(rightSpeedSetpoint - previousRightSpeedSetpoint > maxAcceleration)
	{
		rightSpeedSetpoint = previousRightSpeedSetpoint + maxAcceleration;
	}
	else if(rightSpeedSetpoint - previousRightSpeedSetpoint < -maxAcceleration)
	{
		rightSpeedSetpoint = previousRightSpeedSetpoint - maxAcceleration;
	}

	// Limitation de la vitesse
	if(leftSpeedSetpoint > maxSpeed)
		leftSpeedSetpoint = maxSpeed;
	else if(leftSpeedSetpoint < -maxSpeed)
		leftSpeedSetpoint = -maxSpeed;
	if(rightSpeedSetpoint > maxSpeed)
		rightSpeedSetpoint = maxSpeed;
	else if(rightSpeedSetpoint < -maxSpeed)
		rightSpeedSetpoint = -maxSpeed;

	previousLeftSpeedSetpoint = leftSpeedSetpoint;
	previousRightSpeedSetpoint = rightSpeedSetpoint;

	//serial.printfln("%d",(leftSpeedSetpoint - currentLeftSpeed));

	if(leftSpeedControlled)
		leftSpeedPID.compute();		// Actualise la valeur de 'leftPWM'
	if(rightSpeedControlled)
		rightSpeedPID.compute();	// Actualise la valeur de 'rightPWM'

	leftMotor.run(leftPWM);
	rightMotor.run(rightPWM);
}

bool MotionControlSystem::isPhysicallyStopped() {
	return (currentLeftSpeed == 0) && (currentRightSpeed == 0);
}

void MotionControlSystem::manageStop()
{
	static uint32_t time = 0;

	if (isPhysicallyStopped() && moving)
	{

		if (time == 0)
		{ //Début du timer
			time = Millis();
		}
		else
		{
			if ((Millis() - time) >= delayToStop)
			{ //Si arrêté plus de 'delayToStop' ms
				if (ABS(translationPID.getError()) <= toleranceTranslation && ABS(rotationPID.getError()) <= toleranceRotation)
				{ //Stopé pour cause de fin de mouvement
					stop();
					moveAbnormal = false;
				}
				else
				{ //Stoppé pour blocage
					stop();
					moveAbnormal = true;
				}
			}
		}
	}
	else
	{
		time = 0;
		if(moving)
			moveAbnormal = false;
	}
}

void MotionControlSystem::updatePosition() {
	static volatile int32_t lastDistance = 0;

	float deltaDistanceMm = (currentDistance - lastDistance) * TICK_TO_MM;
	lastDistance = currentDistance;

	x += (deltaDistanceMm * cos(getAngleRadian()));
	y += (deltaDistanceMm * sin(getAngleRadian()));
}


/**
 * Ordres
 */

void MotionControlSystem::orderTranslation(int32_t mmDistance) {
	translationSetpoint += (int32_t) mmDistance / TICK_TO_MM;
	if(!moving)
	{
		translationPID.resetErrors();
		moving = true;
	}
	if ( mmDistance >= 0) {
		direction = FORWARD;
	} else {
		direction = BACKWARD;
	}
	moveAbnormal = false;
}

void MotionControlSystem::orderRotation(float angleConsigneRadian) {

	static int32_t deuxPiTick = 2*PI / TICK_TO_RADIAN;
	static int32_t piTick = PI / TICK_TO_RADIAN;

	int32_t highLevelOffset = originalAngle / TICK_TO_RADIAN;

	int32_t angleConsigneTick = angleConsigneRadian / TICK_TO_RADIAN;
	int32_t angleCourantTick = currentAngle + highLevelOffset;

	int32_t rotationTick = (angleConsigneTick % deuxPiTick) - (angleCourantTick % deuxPiTick);

	if(rotationTick > piTick)
	{
		rotationTick -= deuxPiTick;
	}
	else if(rotationTick < -piTick)
	{
		rotationTick += deuxPiTick;
	}

	rotationSetpoint = angleCourantTick + rotationTick - highLevelOffset;

	if(!moving)
	{
		rotationPID.resetErrors();
		moving = true;
	}
	direction = NONE;
	moveAbnormal = false;
}

void MotionControlSystem::orderRawPwm(Side side, int16_t pwm) {
	if (side == Side::LEFT)
		leftMotor.run(pwm);
	else
		rightMotor.run(pwm);
}

void MotionControlSystem::stop() {
	translationSetpoint = currentDistance;
	rotationSetpoint = currentAngle;
	leftSpeedSetpoint = 0;
	rightSpeedSetpoint = 0;

	leftMotor.run(0);
	rightMotor.run(0);
	moving = false;
	translationPID.resetErrors();
	rotationPID.resetErrors();
	leftSpeedPID.resetErrors();
	rightSpeedPID.resetErrors();
}


void MotionControlSystem::track()
{
	this->trackArray[trackerCursor].x = x;
	this->trackArray[trackerCursor].y = y;
	this->trackArray[trackerCursor].angle = getAngleRadian();

	this->trackArray[trackerCursor].consigneVitesseGauche = leftSpeedSetpoint;
	this->trackArray[trackerCursor].vitesseGaucheCourante = currentLeftSpeed;
	this->trackArray[trackerCursor].vitesseMoyenneGauche = averageLeftSpeed.value();
	this->trackArray[trackerCursor].pwmGauche = leftPWM;

	this->trackArray[trackerCursor].consigneVitesseDroite = rightSpeedSetpoint;
	this->trackArray[trackerCursor].vitesseDroiteCourante = currentRightSpeed;
	this->trackArray[trackerCursor].vitesseMoyenneDroite = averageRightSpeed.value();
	this->trackArray[trackerCursor].pwmDroit = rightPWM;

	this->trackArray[trackerCursor].consigneTranslation = translationSetpoint;
	this->trackArray[trackerCursor].translationCourante = currentDistance;
	this->trackArray[trackerCursor].consigneVitesseTranslation = translationSpeed;

	this->trackArray[trackerCursor].consigneRotation = rotationSetpoint;
	this->trackArray[trackerCursor].rotationCourante = currentAngle;
	this->trackArray[trackerCursor].consigneVitesseRotation = rotationSpeed;

	trackerCursor = (trackerCursor+1)%(TRACKER_SIZE);
}

void MotionControlSystem::printTrackingAll()
{
	for(int i=0; i<TRACKER_SIZE; i++)
	{
		serial.printf("%f\t%f\t%f\t",
				trackArray[i].x, trackArray[i].y, trackArray[i].angle);
		serial.printf("%d\t%d\t%d\t%d\t",
				trackArray[i].consigneVitesseGauche, trackArray[i].vitesseGaucheCourante, trackArray[i].vitesseMoyenneGauche, trackArray[i].pwmGauche);
		serial.printf("%d\t%d\t%d\t%d\t",
				trackArray[i].consigneVitesseDroite, trackArray[i].vitesseDroiteCourante, trackArray[i].vitesseMoyenneDroite, trackArray[i].pwmDroit);
		serial.printf("%d\t%d\t%d\t",
				trackArray[i].consigneTranslation , trackArray[i].translationCourante , trackArray[i].consigneVitesseTranslation);
		serial.printf("%d\t%d\t%d\t",
				trackArray[i].consigneRotation, trackArray[i].rotationCourante, trackArray[i].consigneVitesseRotation);
		serial.printf("\r\n");
	}
}

void MotionControlSystem::printTracking() // Envoie les données nécessaires à l'analyse d'asserv / l'asserv auto (Python)
{
	for(int i=0; i<TRACKER_SIZE; i++)
						{
							serial.printf("%d\t%d\t%d\t%d\t", trackArray[i].vitesseGaucheCourante, trackArray[i].vitesseDroiteCourante, trackArray[i].vitesseMoyenneGauche, trackArray[i].vitesseMoyenneDroite);
							serial.printf("%d\t%d\t%d\t%d\t", trackArray[i].consigneVitesseGauche, trackArray[i].consigneVitesseDroite, trackArray[i].pwmGauche, trackArray[i].pwmDroit);
							serial.printf("\r\n");

						}
}

void MotionControlSystem::printPosition()
{
	for(int i=0; i<TRACKER_SIZE; i++)
	{
		serial.printf("%d\t%d\t%d\t",
				trackArray[i].consigneTranslation , trackArray[i].translationCourante , trackArray[i].consigneVitesseTranslation);
		serial.printf("%d\t%d\t%d\t",
				trackArray[i].consigneRotation, trackArray[i].rotationCourante, trackArray[i].consigneVitesseRotation);
		serial.printf("\r\n");
	}
}

void MotionControlSystem::resetTracking()
{
	trackerType zero;
	zero.angle = 0;
	zero.consigneRotation = 0;
	zero.consigneTranslation = 0;
	zero.consigneVitesseDroite = 0;
	zero.consigneVitesseGauche = 0;
	zero.consigneVitesseRotation = 0;
	zero.consigneVitesseTranslation = 0;
	zero.pwmDroit = 0;
	zero.pwmGauche = 0;
	zero.rotationCourante = 0;
	zero.translationCourante = 0;
	zero.vitesseDroiteCourante = 0;
	zero.vitesseGaucheCourante = 0;
	zero.vitesseMoyenneDroite = 0;
	zero.vitesseMoyenneGauche = 0;
	zero.x = 0;
	zero.y = 0;

	for(int i=0; i<TRACKER_SIZE; i++)
	{
		trackArray[i] = zero;
	}
	trackerCursor = 0;
}

void MotionControlSystem::setTestSpeed(int32_t speed) // set la valeur de la vitesse de test
{
	speedTest=speed;
}


void MotionControlSystem::testSpeed()
{
	translationControlled = false;
	rotationControlled = false;
	leftSpeedControlled = true;
	rightSpeedControlled = true;

	resetTracking();
	translationSpeed = speedTest;
	rotationSpeed = 0;
	Delay(1000);
	translationSpeed = 0;
	printTracking();
	serial.printf("endtest");
}

void MotionControlSystem::testPosition()
{
	translationControlled = true;
	rotationControlled = true;
	leftSpeedControlled = true;
	rightSpeedControlled = true;

	resetTracking();
	orderTranslation(200);
	while(moving)
	{;}
	printPosition();
	serial.printf("endtest");

}

void MotionControlSystem::testSpeedReverse()
{
	translationControlled = false;
	rotationControlled = false;
	leftSpeedControlled = true;
	rightSpeedControlled = true;

	resetTracking();
	translationSpeed = (-1)*speedTest;
	rotationSpeed = 0;
	Delay(1000);
	translationSpeed = 0;
	printTracking();
	serial.printf("endtest");
}

void MotionControlSystem::longTestSpeed()
{
	translationControlled = false;
	rotationControlled = false;
	leftSpeedControlled = true;
	rightSpeedControlled = true;

	resetTracking();
	translationSpeed = 200000;
	rotationSpeed = 0;
	Delay(200);
	translationSpeed = 0;
	printTracking();
	serial.printf("endtest");

}

/**
 * Getters/Setters des constantes d'asservissement en translation/rotation/vitesse
 */

void MotionControlSystem::getTranslationTunings(float &kp, float &ki, float &kd) const {
	kp = translationPID.getKp();
	ki = translationPID.getKi();
	kd = translationPID.getKd();
}
void MotionControlSystem::getRotationTunings(float &kp, float &ki, float &kd) const {
	kp = rotationPID.getKp();
	ki = rotationPID.getKi();
	kd = rotationPID.getKd();
}
void MotionControlSystem::getLeftSpeedTunings(float &kp, float &ki, float &kd) const {
	kp = leftSpeedPID.getKp();
	ki = leftSpeedPID.getKi();
	kd = leftSpeedPID.getKd();
}
void MotionControlSystem::getRightSpeedTunings(float &kp, float &ki, float &kd) const {
	kp = rightSpeedPID.getKp();
	ki = rightSpeedPID.getKi();
	kd = rightSpeedPID.getKd();
}
void MotionControlSystem::setTranslationTunings(float kp, float ki, float kd) {
	translationPID.setTunings(kp, ki, kd);
}
void MotionControlSystem::setRotationTunings(float kp, float ki, float kd) {
	rotationPID.setTunings(kp, ki, kd);
}
void MotionControlSystem::setLeftSpeedTunings(float kp, float ki, float kd) {
	leftSpeedPID.setTunings(kp, ki, kd);
}
void MotionControlSystem::setRightSpeedTunings(float kp, float ki, float kd) {
	rightSpeedPID.setTunings(kp, ki, kd);
}

void MotionControlSystem::setPWM() {
	leftMotor.run(leftPWM);
	rightMotor.run(rightPWM);
}


/*
 * Getters/Setters des variables de position haut niveau
 */
float MotionControlSystem::getAngleRadian() const {
	return (currentAngle * TICK_TO_RADIAN + originalAngle);
}

void MotionControlSystem::setOriginalAngle(float angle) {
	originalAngle = angle - (getAngleRadian() - originalAngle);
}

float MotionControlSystem::getX() const{
	return x;
}

float MotionControlSystem::getY() const{
	return y;
}

void MotionControlSystem::setX(float newX){
	this->x = newX;
}

void MotionControlSystem::setY(float newY){
	this->y = newY;
}

void MotionControlSystem::resetPosition()
{
	x = 0;
	y = 0;
	setOriginalAngle(0);
	stop();
}

void MotionControlSystem::setDelayToStop(uint32_t delayToStop)
{
	this->delayToStop = delayToStop;
}

bool MotionControlSystem::isMoving() const{
	return moving;
}

bool MotionControlSystem::isMoveAbnormal() const{
	return moveAbnormal;
}

MOVING_DIRECTION MotionControlSystem::getMovingDirection() const{
	return direction;
}
