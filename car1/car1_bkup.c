#include <stdlib.h>
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#define TOUCH_PORT_ID NXT_PORT_S2
#define COLOR_PORT_ID NXT_PORT_S3
#define MOTOR_A_ID NXT_PORT_A
#define MOTOR_B_ID NXT_PORT_B
#define RUNTIME_CONNECTION

//Get Bluetooth address for car 2
const U8 bluetooth_slave_address[7] = {0x00, 0x16, 0x53, 0x1A, 0x2E, 0x6B, 0x00};

DeclareCounter(TimerCounter);
DeclareTask(MotorControlTask);
DeclareTask(GetCurrentLightSensor);
DeclareTask(CalibrateSensorsTask);
DeclareTask(StarterTask);
DeclareTask(IdleTask);
DeclareEvent(TouchBlackEvent);
DeclareEvent(TouchWhiteEvent);
DeclareEvent(ReadyEvent);
DeclareResource(LightSensor);


struct sLightResource_t
{
	U16 Black;
	U16 White;
	U16 Threshold;
	U16 Current;
}sLightResource;


/* nxtOSEK hooks */
void ecrobot_device_initialize() {
	ecrobot_init_nxtcolorsensor(COLOR_PORT_ID, NXT_LIGHTSENSOR_RED); // initialize a sensor
	nxt_motor_set_speed(MOTOR_A_ID, 0, 1);
	nxt_motor_set_speed(MOTOR_B_ID, 0, 1);
	ecrobot_set_nxtcolorsensor(COLOR_PORT_ID, NXT_LIGHTSENSOR_RED);
	ecrobot_get_touch_sensor(TOUCH_PORT_ID);
	#ifndef RUNTIME_CONNECTION
		ecrobot_init_bt_master(bluetooth_slave_address, "1234");
	#endif
}

void ecrobot_device_terminate() {
	ecrobot_term_nxtcolorsensor(COLOR_PORT_ID); // terminate a sensor
	nxt_motor_set_speed(MOTOR_A_ID, 0, 1);
	nxt_motor_set_speed(MOTOR_B_ID, 0, 1);
	ecrobot_get_touch_sensor(NXT_PORT_S2);
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

TASK(StarterTask)
{
	int iTouch = 0;
	U8 uPrevTouch = 0, uCurrentTouch = 0;
	
	while(iTouch < 3)
	{
		uCurrentTouch = ecrobot_get_touch_sensor(TOUCH_PORT_ID);
		if(uPrevTouch == 0 && uCurrentTouch == 1)
		{
			switch(iTouch)
			{
				case 0:
					SetEvent(CalibrateSensorsTask, TouchBlackEvent);
					break;
					
				case 1:
					SetEvent(CalibrateSensorsTask, TouchWhiteEvent);
					break;
					
				case 2:
					display_goto_xy(0,4);
					display_string("Third");
					display_update();
					SetEvent(MotorControlTask, ReadyEvent);
					break;
					
				default:
					break;
			}
		}
		else if(uPrevTouch == 1 && uCurrentTouch == 0)
		{
			iTouch++;
		}
		uPrevTouch = uCurrentTouch;
			
		systick_wait_ms(100);
	}
	TerminateTask();
}


TASK(CalibrateSensorsTask)
{
	display_clear(0);
	
	display_goto_xy(0,1);
	display_string("Calibrating...");
	
	display_update();
	ecrobot_process_bg_nxtcolorsensor();
	
	EventMaskType CalibrateEventMask = 0;
	int iBlack = 0, iWhite = 0;
		
 	while(1)
	{
		WaitEvent(TouchBlackEvent | TouchWhiteEvent);
		GetEvent(CalibrateSensorsTask, &CalibrateEventMask);
		if(CalibrateEventMask & TouchBlackEvent)
		{
			//Getting the value for the black color
			ClearEvent(TouchBlackEvent);
			ecrobot_process_bg_nxtcolorsensor();
			GetResource(LightSensor);
			sLightResource.Black = ecrobot_get_nxtcolorsensor_light(COLOR_PORT_ID);
			display_goto_xy(0,1);
			display_string("Black : ");
			display_int(sLightResource.Black, 0);
			ReleaseResource(LightSensor);
			display_update();
			iBlack = 1;
		}
		if(CalibrateEventMask & TouchWhiteEvent)
		{
			//Getting the value for white
			ClearEvent(TouchWhiteEvent);
			ecrobot_process_bg_nxtcolorsensor();
			GetResource(LightSensor);
			sLightResource.White = ecrobot_get_nxtcolorsensor_light(COLOR_PORT_ID);
			
			display_goto_xy(0,2);
			display_string("White : ");
			display_int(sLightResource.White, 0);
			ReleaseResource(LightSensor);
			display_update();
			iWhite = 1;
		}
		
		if(iBlack == 1 && iWhite == 1)
		{
			//Calculating the average.
			GetResource(LightSensor);
			sLightResource.Threshold = (sLightResource.Black + sLightResource.White)/ 2;

			display_goto_xy(0,3);
			display_string("Threshold: ");
			display_int(sLightResource.Threshold,0);
			ReleaseResource(LightSensor);

			display_update();
			break;	
		}
		
		systick_wait_ms(1000);
	} 
	TerminateTask();
}

TASK(GetCurrentLightSensor)
{
	//This task collects the light sensor data frequently and updates the same.
	GetResource(LightSensor);
	
	ecrobot_process_bg_nxtcolorsensor();
	sLightResource.Current = ecrobot_get_nxtcolorsensor_light(COLOR_PORT_ID);
	
	ReleaseResource(LightSensor);
	TerminateTask();
}
TASK(MotorControlTask)
{
	float 	fKp 		= 0.5;
	int 	iBaseSpeed 	= 50;
	int 	iError, iTurn;
	int 	iPowerA, iPowerB;
	U8		bt_send_buf[32];
	U8		bt_receive_buf[32];
	U32		uStatus;
	static int iPreviousError;
	
	display_goto_xy(0,4);
	display_string("Mctl");
	display_update();
	
	bt_receive_buf[0] = 1;
	
	bt_send_buf[0] = 0;
	bt_send_buf[1] = 50;
	bt_send_buf[2] = 50;
	
	GetResource(LightSensor);
	
	if(sLightResource.Threshold == 0)
	{
		WaitEvent(ReadyEvent);
	}
	
	//Calculate the Error
	iError = sLightResource.Threshold - sLightResource.Current;
	
	display_clear(0);
	display_goto_xy(0,1);
	display_string("Error: ");
	display_int(iError,0);
	display_update();
	
	display_goto_xy(0,2);
	display_string("Threshold: ");
	display_int(sLightResource.Threshold,0);
	display_update();
	
	iTurn = fKp * iError;
	
	ecrobot_read_bt_packet(bt_receive_buf, 32);
	if(bt_receive_buf[0] == 0)
	{
		iPowerA = 0;
		iPowerB = 0;
	}
	else
	{
	if(iError > 100)
	{
		//The car turns to the right
		iPowerA = iBaseSpeed - iTurn;
		iPowerB = iBaseSpeed + iTurn;

		if(iError > 130 && iPreviousError > 130)
		{
			bt_send_buf[0] = 1;
			bt_send_buf[1] = iPowerA;
			bt_send_buf[2] = iPowerB;
			uStatus = ecrobot_send_bt_packet(bt_send_buf,32);
			display_goto_xy(0,0);
			display_int(uStatus,0);
			display_update();
			
		}
		
		display_goto_xy(0,3);
		display_int(iPowerA,0);
		display_goto_xy(0,4);
		display_int(iPowerB,0);
		display_update();
	}
	else if(iError < -100)
	{
		//The car turns to the left
		iPowerA = iBaseSpeed - iTurn;
		iPowerB = iBaseSpeed + iTurn;
		
		if(iError < -130 && iPreviousError < -130)
		{
			bt_send_buf[0] = 1;
			bt_send_buf[1] = iPowerA;
			bt_send_buf[2] = iPowerB;
			uStatus = ecrobot_send_bt_packet(bt_send_buf,32);
			display_goto_xy(0,0);
			display_int(uStatus,0);
			display_update();
		}
		
	}	
	else
	{
		iPowerA = iBaseSpeed - iTurn;
		iPowerB = iBaseSpeed + iTurn;
	}
	}
 	nxt_motor_set_speed(MOTOR_A_ID, iPowerA, 1);
	nxt_motor_set_speed(MOTOR_B_ID, iPowerB, 1);  
	iPreviousError = iError;
	
	systick_wait_ms(100);
	
	ReleaseResource(LightSensor);
	
	TerminateTask();
}

TASK(IdleTask)
{
  static SINT bt_status = BT_NO_INIT;

  display_goto_xy(0,3);
  display_string("IDTa");
  display_update();
  
  while(1)
  {
#ifdef RUNTIME_CONNECTION
    ecrobot_init_bt_master(bluetooth_slave_address, "1234");
#endif
	if(ecrobot_get_bt_status() == BT_NO_INIT)
	{
	  //display_clear(0);
      display_goto_xy(0, 0);
      display_string("[IT]");
      display_update();
	}
	
	if(ecrobot_get_bt_status() == BT_CONNECTED)
	{
	  //display_clear(0);
      display_goto_xy(0, 0);
      display_string("[CTD]");
      display_update();
	}
	
	
    if (ecrobot_get_bt_status() == BT_STREAM && bt_status != BT_STREAM)
    {
      //display_clear(0);
      display_goto_xy(0, 0);
      display_string("[BT]");
      display_update();
    }
    bt_status = ecrobot_get_bt_status();
  }
}
