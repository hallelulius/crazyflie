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
static bool isInit2;
static bool isInit3;
static bool isInit4;

static uint16_t limitThrust(int32_t value);

//global variables for the tasks
static bool mode;
const int NREF = 4;
float referenceSignal[4];
xSemaphoreHandle gatekeeperRef = 0;
xSemaphoreHandle gatekeeperMode = 0;
static void mainControlTask(void *arg);
static void referenceGeneratorTask(void *arg);
static void modeSwitchTask(void *arg);


// global variables for the control
float controlSignal[] = {0, 0, 0, 0};
float integratorOutput[] = {0, 0, 0, 0, 0, 0}; // store the integrator output
float estimatedState[] = {0, 0, 0, 0, 0, 0};
float dt = ATTITUDE_UPDATE_DT; // 1/500
float rollDot = 0;
float prevRoll = 0;
float pitchDot = 0;
float prevPitch = 0;
float yawDot = 0;
float prevYaw = 0;


// model constants
float const m = 0.027;
float const g = 9.8;

float toRad(float degrees){
	return degrees * ((float) M_PI) / 180;
}

float thrustToPWM(float controlSignal){
	// from polyfit-280.5110  572.9469    5.6284
	// tweaked -250.5110*T.^2 + 572.9469 * T

	// from table + polyfit   -7.6280e+04 1.4921e+05 1.1357e+03 no need to multiply with 256

	double a = -7.6280e4;
	double b = 1.4921e5;
	double c = 1.1357e3;

	float pwm = (a * pow(controlSignal,2) + b * controlSignal + c);
	return pwm;
}

/* LQR controller
 * u = -K * x + Kr * r */
void LQRController(float stateEst[], float refSignal[]){
	int nCtrl = 4;
	int nRefs = 6;
	int a = 15.8114;
	int b = 5.0002;
	int c = 0.0158;
	int d= 0.0158;
	float K[4][6] = {{-a, -b,  a,  b,  c,  d},
					 {-a, -b, -a, -b, -c, -d},
					 { a,  b, -a, -b,  c,  d},
					 { a,  b,  a,  b, -c, -d}};

	//float Kr[4][6] = {{-a, -b,  a,  b,  c,  d},
	//			{-a, -b, -a, -b, -c, -d},
	//			{ a,  b, -a, -b,  c,  d},
	//			{ a,  b,  a,  b, -c, -d}};
	int i;
	int j;

	for (i = 0; i< nCtrl; i++){
		controlSignal[i] = 0;
		for (j = 0; j<nRefs; j++){
			controlSignal[i]+= -K[i][j] * stateEst[j] + refSignal[i];
		}
		controlSignal[i] = thrustToPWM(controlSignal[i]);
		if(controlSignal[i]>40000.0){
			controlSignal[i] = 40000.0;
		}
		if (controlSignal[i] < 0){
			controlSignal[i] = 0;
		}
	}
}

/* LQR controller
 * u = -[K; Ki] * [x; x_i] + Kr * r /
double* LQIController(double* stateEst, double* refSignal, double* output){
    int nCtrl = 4;
    int nRefs = 8;
    double K[nCtrl][nRefs] = {{1, 2 ,3, 4, 5, 6, 7, 8},
                              {1, 2 ,3, 4, 5, 6, 7, 8},
                              {1, 2 ,3, 4, 5, 6, 7, 8},
                              {1, 2 ,3, 4, 5, 6, 7, 8}};
    double Ki[nCtrl][nRefs] = {{1, 2 ,3, 4, 5, 6, 7, 8},
                               {1, 2 ,3, 4, 5, 6, 7, 8},
                               {1, 2 ,3, 4, 5, 6, 7, 8},
                               {1, 2 ,3, 4, 5, 6, 7, 8}};

    int i;
    int j;
    double error;
    for (i = 0; i< nCtrl; i++){
      for (j = 0; j<nRefs; j++){
        error = refSignal[j] - output[j];
        integratorOutput[j] += error;
        controlSignal[i]+= -K[i][j] * stateEst[j] + -Ki[i][j] * integratorOutput[j];
      }
    }

    return controlSignal;
}

 */


void mainControlInit(void)
{
	if(isInit2)
		return;

	ledInit();
	gatekeeperRef = xSemaphoreCreateMutex();
	gatekeeperMode = xSemaphoreCreateMutex();

	xTaskCreate(mainControlTask, MAIN_CONTROL_TASK_NAME,
			MAIN_CONTROL_TASK_STACKSIZE, NULL, MAIN_CONTROL_TASK_PRI, NULL);

	isInit2 = true;
}

bool mainControlTest(void)
{
	// not sure what to add here
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

				rollDot = (prevRoll - toRad(eulerRollActual))/dt;
				pitchDot = (prevPitch - toRad(eulerPitchActual))/dt;
				yawDot = (prevYaw - toRad(eulerYawActual))/dt;
				estimatedState[0] = toRad(eulerRollActual);
				estimatedState[1] = rollDot;
				estimatedState[2] = toRad(eulerPitchActual);
				estimatedState[3] = pitchDot;
				estimatedState[4] = toRad(eulerYawActual);
				estimatedState[5] = yawDot;

				LQRController(estimatedState, currRef);
				prevRoll = estimatedState[0];
				prevPitch = estimatedState[2];
				prevYaw = estimatedState[4];

				// do more control stuff
				if (currMode == 1 || currMode == 0){

					motorPowerM1 = limitThrust(fabs(controlSignal[0]));
					motorPowerM2 = limitThrust(fabs(controlSignal[1]));
					motorPowerM3 = limitThrust(fabs(controlSignal[2]));
					motorPowerM4 = limitThrust(fabs(controlSignal[3]));
				}
				else
				{
					motorPowerM1 = limitThrust(0);
					motorPowerM2 = limitThrust(0);
					motorPowerM3 = limitThrust(0);
					motorPowerM4 = limitThrust(0);
				}

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
	if(isInit3)
		return;
	int i;
	for (i = 0; i<NREF; i++){
		referenceSignal[i] = 0;
	}

	xTaskCreate(referenceGeneratorTask, REF_GENERATOR_TASK_NAME,
			REF_GENERATOR_TASK_STACKSIZE, NULL, REF_GENERATOR_TASK_PRI, NULL);

	isInit3 = true;
}

bool referenceGeneratorTest(void)
{
	bool pass = true;

	return pass;
}

static void referenceGeneratorTask(void* param)
{
	//bool increase = true;
	systemWaitStart();

	while(1)
	{
		if(xSemaphoreTake(gatekeeperRef, 2000))
		{

			referenceSignal[0] = m*g/4;
			referenceSignal[1] = m*g/4;
			referenceSignal[2] = m*g/4;
			referenceSignal[3] = m*g/4;

			/*
			int i;

			for(i = 0; i<NREF; i++)
			{

				if (referenceSignal[i]< 1 && increase)
				{
					referenceSignal[i] += 0.01;
				}
				else if (referenceSignal[i] >= 0)
				{
					referenceSignal[i]-= 0.01;
					increase = false;
				}
				else
				{
					increase = true;
				}
			}
			 */
			xSemaphoreGive(gatekeeperRef);
		}
		vTaskDelay(M2T(250)); //increase later
	}
}

void modeSwitchInit(void)
{
	if(isInit4)
		return;
	mode = 0;
	xTaskCreate(modeSwitchTask, MODE_SWITCH_TASK_NAME,
			MODE_SWITCH_TASK_STACKSIZE, NULL, MODE_SWITCH_TASK_PRI, NULL);

	isInit4 = true;
}
bool modeSwitchTest(void)
{
	bool pass = true;
	//pass &= motorsTest();
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
		vTaskDelay(M2T(5000)); //increase later
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

		if (imu6IsCalibrated())
		{
			// 250HZ
			//if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
			//{
			//sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, ATTITUDE_UPDATE_DT);
			//sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
			// Set motors depending on the euler angles
			//motorPowerM1 = limitThrust(fabs(32000*eulerYawActual/180.0));
			//motorPowerM2 = limitThrust(fabs(32000*eulerPitchActual/180.0));
			//motorPowerM3 = limitThrust(fabs(32000*eulerRollActual/180.0));
			//motorPowerM4 = limitThrust(fabs(32000*eulerYawActual/180.0));

			//motorsSetRatio(MOTOR_M1, motorPowerM1);
			//motorsSetRatio(MOTOR_M2, motorPowerM2);
			//motorsSetRatio(MOTOR_M3, motorPowerM3);
			//motorsSetRatio(MOTOR_M4, motorPowerM4);

			//attitudeCounter = 0;
			//}
		}
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

//LOG_GROUP_START(tasks)
//LOG_ADD(LOG_INT8, mode, &mode)
//LOG_ADD(LOG_FLOAT, ref, &referenceSignal[0]) // does this work?
//LOG_GROUP_STOP(tasks)

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
