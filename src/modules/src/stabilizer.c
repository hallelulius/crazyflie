/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "attitude_controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "param.h"
#include "sitaw.h"
#ifdef PLATFORM_CF1
#include "ms5611.h"
#else
#include "lps25h.h"
#endif
#include "num.h"
#include "position_estimator.h"
#include "position_controller.h"
#include "altitude_hold.h"
#include "led.h"

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define ATTITUDE_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

#define ALTHOLD_UPDATE_RATE_DIVIDER  5
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 100hz

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;   // Measured roll angle in deg
static float eulerPitchActual;  // Measured pitch angle in deg
static float eulerYawActual;    // Measured yaw angle in deg

uint16_t actuatorThrust;  // Actuator output for thrust base

uint32_t motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)

static bool isInit;
static bool isInitMain;
static bool isInitRef;
static bool isInitMode;

static uint16_t limitThrust(int32_t value);

//global variables for the tasks
static bool mode;
xSemaphoreHandle gatekeeperRef = 0;
xSemaphoreHandle gatekeeperMode = 0;
static void mainControlTask(void *arg);
static void referenceGeneratorTask(void *arg);
static void modeSwitchTask(void *arg);


// global variables for the control
const int NREF = 4;
float referenceSignal[4] = {0, 0, 0, 0};
float controlSignal[] = {0, 0, 0, 0};
float controlDebugThrust[] = {0, 0, 0, 0};
float controlDebugPWM[] = {0, 0, 0, 0};
float estimatedState[] = {0, 0, 0, 0, 0, 0};
float dt = ATTITUDE_UPDATE_DT; // 1/250


// state variables
float rollDot = 0;
float pitchDot = 0;
float yawDot = 0;

float prevRoll = 0;
float prevPitch = 0;
float prevYaw = 0;

float prevRollDot = 0;
float prevPitchDot = 0;
float prevYawDot = 0;

// tuning parameters
float paramGain = 0;
float paramD = 1;
float paramAlpha = 0.9;
float paramRef = 0;
float paramStep = 0.005;

// k parameters
float Ka = 0.0255;
float Kb = 0.0085;
float Kc = 0;
float Kd=  0;

float aggressiveK[] = {0.0224, 0.0074, 0.0005, 0.0025};
float normalK[] = {0.0255, 0.0085, 0, 0};

// model constants
float const m = 0.027;
// gain 0.002 ref 0.16 d 1 alpha 0.5
// gain 0.0014 ref 0.16 d 1 alpha 0.5
// how much thrust to hover? around 0.15 depending on battery
// change K matrix
float const g = 9.81;


float toRad(float degrees){
	return degrees * ((float) M_PI) / 180;
}

float thrustToPWM(float controlSignal){
	//old a -1.2205e6;
	//	  b	5.9683e5;
	//	  c	1.1357e3
	// new   8.1372e+05
	//  	 3.0676e+05
	//  	 659.2136
	double a = -1.2205e6;	// 8.1372e5;
	double b = 5.9683e5;	// 3.0676e5;
	double c = 1.1357e3;	// 659.2136;
	if (controlSignal > 0.15) controlSignal = 0.15;
	float pwm = (a * pow(controlSignal,2) + b * controlSignal + c);
	return pwm;
}

/* LQR controller
 * u = -K * x + Kr * r */
void LQRController(float stateEst[], float refSignal[], float paramK[]){
	int nCtrl = 4;
	int nRefs = 6;
	float a = paramK[0];
	float b = paramK[1];
	float c = paramK[2];
	float d = paramK[3];
	float K[4][6] = {{-a, -b,  a,  b,  c,  d},
			{-a, -b, -a, -b, -c, -d},
			{ a,  b, -a, -b,  c,  d},
			{ a,  b,  a,  b, -c, -d}};

	int i;
	int j;

	for (i = 0; i< nCtrl; i++){
		controlSignal[i] =  refSignal[i] *paramRef;
		for (j = 0; j<nRefs; j++){
			controlSignal[i]+= -K[i][j] * stateEst[j] *paramGain;
			controlDebugThrust[i] = controlSignal[i];
		}
		controlSignal[i] = thrustToPWM(controlSignal[i]);
		if (controlSignal[i] < 0){
			controlSignal[i] = 0;
		}
	}
}
void mainControlInit(void)
{
	if(isInitMain)
		return;

	ledInit();
	gatekeeperRef = xSemaphoreCreateMutex();
	gatekeeperMode = xSemaphoreCreateMutex();

	xTaskCreate(mainControlTask, MAIN_CONTROL_TASK_NAME,
			MAIN_CONTROL_TASK_STACKSIZE, NULL, MAIN_CONTROL_TASK_PRI, NULL);

	isInitMain = true;
}

bool mainControlTest(void)
{
	bool pass = true;
	return pass;
}

static void mainControlTask(void* param)
{
	uint32_t attitudeCounter = 0;
	uint32_t lastWakeTime;
	int currMode = 0;
	float currRef[NREF];

	vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

	//Wait for the system to be fully started to start stabilization loop
	systemWaitStart();

	lastWakeTime = xTaskGetTickCount ();

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

		// Magnetometer not yet used more then for logging.
		imu9Read(&gyro, &acc, &mag);

		if (imu6IsCalibrated())
		{
			// 250HZ
			if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
			{
				sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, ATTITUDE_UPDATE_DT);
				sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
				attitudeCounter = 0;

				// get states
				rollDot = (1-paramAlpha) * prevRollDot + paramAlpha * (toRad(eulerRollActual) - prevRoll)/dt;
				pitchDot = (1-paramAlpha) * prevPitchDot + paramAlpha * (toRad(eulerPitchActual) - prevPitch)/dt;
				yawDot = (1-paramAlpha) * prevYawDot + paramAlpha * (toRad(eulerPitchActual) - prevYaw)/dt;
				estimatedState[0] = toRad(eulerRollActual);
				estimatedState[1] = paramD * rollDot;
				estimatedState[2] = toRad(eulerPitchActual);
				estimatedState[3] = paramD * pitchDot;
				estimatedState[4] = toRad(eulerYawActual);
				estimatedState[5] = paramD * yawDot;

				if (currMode == 1){
					float K[] = {Ka, Kb, Kc, Kd};
					LQRController(estimatedState, currRef, K);
				}else{

					LQRController(estimatedState, currRef, normalK);
				}
				prevRoll = estimatedState[0];
				prevPitch = estimatedState[2];
				prevYaw = estimatedState[4];

				prevRollDot = estimatedState[1];
				prevPitchDot = estimatedState[3];
				prevYawDot = estimatedState[5];


				motorPowerM1 = limitThrust(fabs(controlSignal[0]));
				motorPowerM2 = limitThrust(fabs(controlSignal[1]));
				motorPowerM3 = limitThrust(fabs(controlSignal[2]));
				motorPowerM4 = limitThrust(fabs(controlSignal[3]));


				motorsSetRatio(MOTOR_M1, motorPowerM1);
				motorsSetRatio(MOTOR_M2, motorPowerM2);
				motorsSetRatio(MOTOR_M3, motorPowerM3);
				motorsSetRatio(MOTOR_M4, motorPowerM4);
			}
		}

		// get current mode and reference signal
		if (xSemaphoreTake(gatekeeperMode, 100))
		{
			currMode = mode;
			xSemaphoreGive(gatekeeperMode);
		}
		if (xSemaphoreTake(gatekeeperRef, 100))
		{
			int i;
			for (i = 0; i<NREF; i++)
			{
				currRef[i] = referenceSignal[i];
			}
			xSemaphoreGive(gatekeeperRef);
		}
	}
}


// reference generator
void referenceGeneratorInit(void)
{
	if(isInitRef)
		return;
	int i;
	for (i = 0; i<NREF; i++){
		referenceSignal[i] = 0;
	}

	xTaskCreate(referenceGeneratorTask, REF_GENERATOR_TASK_NAME,
			REF_GENERATOR_TASK_STACKSIZE, NULL, REF_GENERATOR_TASK_PRI, NULL);

	isInitRef = true;
}

bool referenceGeneratorTest(void)
{
	bool pass = true;

	return pass;
}

static void referenceGeneratorTask(void* param)
{
	bool toggle = true;
	bool up = true;
	systemWaitStart();

	while(1)
	{
		if(xSemaphoreTake(gatekeeperRef, 2000))
		{

			int i;
			float step = 0;
			if (toggle && up){
				step = paramStep;
				toggle = false;
				up = false;
			} else if(toggle && !up){
				step = -paramStep;
				toggle = false;
				up = true;
			} else {
				toggle = true;
			}
			for(i = 0; i<NREF; i++)
			{
				referenceSignal[i] = m*g/4.0 + step;
			}
			xSemaphoreGive(gatekeeperRef);
		}
		vTaskDelay(M2T(4000));
	}
}

void modeSwitchInit(void)
{
	if(isInitMode)
		return;
	mode = 0;
	xTaskCreate(modeSwitchTask, MODE_SWITCH_TASK_NAME,
			MODE_SWITCH_TASK_STACKSIZE, NULL, MODE_SWITCH_TASK_PRI, NULL);

	isInitMode = true;
}
bool modeSwitchTest(void)
{
	bool pass = true;
	return pass;
}

static void modeSwitchTask(void* param)
{
	//Wait for the system to be fully started
	systemWaitStart();
	while(1)
	{
		if(xSemaphoreTake(gatekeeperMode, 2000)){
			if (mode == 1){
				mode = 0;
				ledSet(0, 1); // which led is this? LED_BLUE_L
			}
			else{
				mode = 1;
				ledSet(0, 0);
			}
			xSemaphoreGive(gatekeeperMode);
		}
		vTaskDelay(M2T(10000)); //increase later
	}
}

static void stabilizerTask(void* param)
{
	//uint32_t attitudeCounter = 0;
	uint32_t lastWakeTime;

	vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

	//Wait for the system to be fully started to start stabilization loop
	systemWaitStart();

	lastWakeTime = xTaskGetTickCount ();

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

		// Magnetometer not yet used more then for logging.
		//imu9Read(&gyro, &acc, &mag);
	}
}

void stabilizerInit(void)
{
	if(isInit)
		return;

	motorsInit(motorMapDefaultBrushed);
	imu6Init();
	sensfusion6Init();
	attitudeControllerInit();

	xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
			STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= motorsTest();
	pass &= imu6Test();
	pass &= sensfusion6Test();
	pass &= attitudeControllerTest();

	return pass;
}

static uint16_t limitThrust(int32_t value)
{
	return limitUint16(value);
}

PARAM_GROUP_START(params)
PARAM_ADD(PARAM_FLOAT, gain, &paramGain)
PARAM_ADD(PARAM_FLOAT, alpha, &paramAlpha)
PARAM_ADD(PARAM_FLOAT, ref, &paramRef)
PARAM_ADD(PARAM_FLOAT, step, &paramStep)
LOG_ADD(PARAM_FLOAT, Ka, &Ka)
LOG_ADD(PARAM_FLOAT, Kb, &Kb)
LOG_ADD(PARAM_FLOAT, Kc, &Kc)
LOG_ADD(PARAM_FLOAT, Kd, &Kd)
PARAM_GROUP_STOP(params)


LOG_GROUP_START(debugdata)
LOG_ADD(LOG_INT8, mode, &mode)
LOG_ADD(LOG_FLOAT, roll, &estimatedState[0])
LOG_ADD(LOG_FLOAT, pitch, &estimatedState[2])
LOG_ADD(LOG_FLOAT, yaw, &estimatedState[4])
LOG_ADD(LOG_FLOAT, rollDot, &estimatedState[1])
LOG_ADD(LOG_FLOAT, pitchDot, &estimatedState[3])
LOG_ADD(LOG_FLOAT, yawDot, &estimatedState[5])
LOG_ADD(LOG_FLOAT, controlT1, &controlDebugThrust[0])
LOG_ADD(LOG_FLOAT, controlT2, &controlDebugThrust[1])
LOG_ADD(LOG_FLOAT, ref1, &referenceSignal[1])
//LOG_ADD(LOG_FLOAT, controlT3, &controlDebugThrust[2])
//LOG_ADD(LOG_FLOAT, controlT4, &controlDebugThrust[3])
LOG_GROUP_STOP(debugdata)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)
