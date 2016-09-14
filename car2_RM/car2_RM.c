#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#define SONAR_SENSOR_ID NXT_PORT_S2
#define MOTOR_A_ID NXT_PORT_A
#define MOTOR_B_ID NXT_PORT_B
#define RUNTIME_CONNECTION
#define MOTOR_BASE_SPEED 50

//#define BT_PERIODIC_TASK

DeclareCounter(TimerCounter);
DeclareTask(MotorControlTask);
DeclareTask(GetCurrentSonarSensor);
DeclareResource(SonarSensor);
DeclareTask(Bluetooth_Receive);

/* struct sSonarResource_t
{
	U16 Black;
	U16 White;
	U16 Threshold;
	U16 Current;
}sLightResource; */
int generate_random_number()
{
	int iRandom;
		
	//srand(time(NULL));
	iRandom = rand() % 10;
	
	return iRandom;
}

int iSonarDistance = 0;
struct sMotorResource_t
{
	S32 sTurnDistance;
	S32 sSonarDistance;
	S32 sRightMotor;
	S32 sLeftMotor;
	S32 sCalibratedDistance;
}sMotorResource;


/* nxtOSEK hooks */
void ecrobot_device_initialize() {
	/* 
	Left motor
	Right Motor
	Ultra sonic sensor
	Bluetooth connection
	*/
	#ifndef RUNTIME_CONNECTION
		ecrobot_init_bt_slave("1234");
	#endif
	nxt_motor_set_speed(MOTOR_A_ID, 0, 1);
	nxt_motor_set_speed(MOTOR_B_ID, 0, 1);
	ecrobot_init_sonar_sensor(SONAR_SENSOR_ID);
}

void ecrobot_device_terminate() {
	nxt_motor_set_speed(MOTOR_A_ID, 0, 1);
	nxt_motor_set_speed(MOTOR_B_ID, 0, 1);
	ecrobot_term_sonar_sensor(SONAR_SENSOR_ID);
	ecrobot_term_bt_connection();
}

void user_1ms_isr_type2() 
{
	StatusType ercd;
	
	ercd = SignalCounter(TimerCounter);
	if(ercd != E_OK)
	{
		ShutdownOS(ercd);
	}
}

TASK(Bluetooth_Receive)
{

	static U8 bt_receive_buf[32];
	int iRandom;
	GetResource(SonarSensor);
	
	/*display_clear(0);
	display_goto_xy(0,0);
	display_string("BT received");
	display_update();*/
	display_clear(0);
	display_goto_xy(0,1);
	display_string("BT_receive");
	display_update();
	
	iRandom = generate_random_number();
	
	if(iRandom %2 == 0)
	{
		
	ecrobot_read_bt_packet(bt_receive_buf, 32);
	if(bt_receive_buf[0] == 1)
	{
		display_clear(0);
		display_goto_xy(0,1);
		display_string("msg received");
		display_update();
		sMotorResource.sSonarDistance = ecrobot_get_sonar_sensor(SONAR_SENSOR_ID);
		if(sMotorResource.sTurnDistance == 0)
		{
			sMotorResource.sTurnDistance = ecrobot_get_sonar_sensor(SONAR_SENSOR_ID);
		}
		if(bt_receive_buf[1] != MOTOR_BASE_SPEED)
		{
			sMotorResource.sLeftMotor = bt_receive_buf[1];
			display_goto_xy(0,2);
			display_int(bt_receive_buf[1],50);
			display_update();
		}
		if(bt_receive_buf[2] != MOTOR_BASE_SPEED)
		{
			sMotorResource.sRightMotor = bt_receive_buf[2];
			display_goto_xy(0,3);
			display_int(bt_receive_buf[2],50);
			display_update();
		}
	}	
	}
	
	ReleaseResource(SonarSensor);
	TerminateTask();
}

TASK(GetCurrentSonarSensor)
{
	//This task collects the light sensor data frequently and updates the same.
	int iRandom = generate_random_number();
	display_goto_xy(0,3);
	display_string("Sonar");
	display_update();
	GetResource(SonarSensor);
	
	if(iRandom % 2 == 0)
	{
		sMotorResource.sSonarDistance = ecrobot_get_sonar_sensor(SONAR_SENSOR_ID);
	}

	ReleaseResource(SonarSensor);
	TerminateTask();
}

TASK(MotorControlTask)
{
	GetResource(SonarSensor);
	S32 sLeftMotor, sRightMotor;
	U8	bt_send_buf[32];
	
	int iRandom = 0;
	
	bt_send_buf[0] = 1;
	display_goto_xy(0,1);
	display_int(sMotorResource.sSonarDistance, 0);
	//display_update();
	display_goto_xy(0,2);
	display_int(sMotorResource.sTurnDistance, 0);
	display_update();
	
	iRandom = generate_random_number();
	display_goto_xy(0,4);
	display_int(iRandom,50);
	display_update();
	if(iRandom % 2 == 0)
	{
		if(sMotorResource.sSonarDistance <= 225)
		{
			sLeftMotor 	= 0;
			sRightMotor = 0;
		}
		
		else if(sMotorResource.sTurnDistance - sMotorResource.sSonarDistance == 0)
		{
			//sMotorResource.sTurnDistance = 0;
			/*sLeftMotor 	= sMotorResource.sLeftMotor;
			sRightMotor = sMotorResource.sRightMotor;*/
			sLeftMotor 	= 0;
			sRightMotor = 0;
		}
		//else if(sMotorResource.sSonarDistance > sMotorResource.sCalibratedDistance)
		else if(sMotorResource.sSonarDistance >= 255)
		{
			bt_send_buf[0] = 0;
			ecrobot_send_bt_packet(bt_send_buf,32);
			/*LeftMotor 	= sMotorResource.sLeftMotor;
			sRightMotor = sMotorResource.sRightMotor;*/
			sLeftMotor  = sMotorResource.sLeftMotor;
			sRightMotor = sMotorResource.sRightMotor;		
		}
		else
		{
			sLeftMotor  = sMotorResource.sLeftMotor;
			sRightMotor = sMotorResource.sRightMotor;
		}
	}
	
	nxt_motor_set_speed(MOTOR_A_ID,sLeftMotor,1);
	nxt_motor_set_speed(MOTOR_B_ID,sRightMotor,1);
	
	systick_wait_ms(100);
		
	ReleaseResource(SonarSensor);
	
	TerminateTask();
}

/* IdleTask */
TASK(IdleTask)
{
  static SINT bt_status = BT_NO_INIT;

#ifndef BT_PERIODIC_TASK
  while(1)
  {
#endif
#ifdef RUNTIME_CONNECTION
    ecrobot_init_bt_slave("1234");
#endif
	
	if(ecrobot_get_bt_status() == BT_NO_INIT)
	{
		display_clear(0);
		display_goto_xy(0,0);
		display_string("Not initialized");
		display_update();
	}
	
    if (ecrobot_get_bt_status() == BT_STREAM && bt_status != BT_STREAM)
    {
      display_clear(0);
      display_goto_xy(0, 0);
      display_string("[BT]");
      display_update();
    }
    bt_status = ecrobot_get_bt_status();
#ifndef BT_PERIODIC_TASK
  }
#endif
}
