#include "gd32f10x.h"
#include "systick.h"
#include <stdio.h>
#include "led_driver.h"
#include "key_driver.h"
#include "oled_driver.h"
#include "tim_driver.h"
#include "enc_driver.h"
#include "uart_driver.h"
#include "pwm_driver.h"
#include "Photoelectric.h"

/*!
	\brief      main function
	\param[in]  none
	\param[out] none
	\retval     none
*/
int main(void)
{
	uint8_t sensor_Value[4];
	char txt[20];
	systick_config();
	rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1); // AHB��Ƶ��1��Ƶ
	systick_config();						  // ϵͳ��Ƶ72MHZ,�����ⲿ����
	rcu_periph_clock_enable(RCU_AF);		  // �ܽŸ���ʱ��alternate function clockʹ��

	led_init(); // LED��ʼ��
	key_init();
	OLED_Init(); // OLED��ʼ��
	delay_1ms(200);
	OLED_CLS();
	sensor_init();
	Motor_Init();	
  int32_t duty = 0;	//����ռ�ձȵı���	
	int S_Value [4] = {0,0,0,0};
	
	while (1)
	{
		sensor_Value[0] = Read_sensor(sensor1);
		sensor_Value[1] = Read_sensor(sensor2);
		sensor_Value[2] = Read_sensor(sensor3);
		sensor_Value[3] = Read_sensor(sensor4);
		
		sprintf(txt, "%d %d %d %d", sensor_Value[0], sensor_Value[1], sensor_Value[2], sensor_Value[3]);
		OLED_P6x8Str(0, 2, txt); // �ַ���
		led_toggle();
		//delay_1ms(200);
		int speed_slow = 1300;
		int speed_medium = 1800;
		int speed_quick = 2100;
		int rotate = 1600;
		OLED_P6x8Str(0, 1, "PWM_Test."); // �ַ���
		
		//straight
		if (sensor_Value[1] == 1&&
		sensor_Value[2] == 1&&
		sensor_Value[0]==sensor_Value[3]
		)
		{
			duty = speed_quick;
			MotorCtrl3W(0, -duty, duty);
		}
		
		//left run
		else if (sensor_Value[0] == 1&&
		sensor_Value[1] == 1&&
		sensor_Value[2] == 1&&
		sensor_Value[3] == 0)
		{
			/*duty = rotate;*/
			MotorCtrl3W(0,-speed_quick, speed_medium );
		}// same 1 = 1 2 = 1
		
		//right run
		else if (sensor_Value[0] == 0&&
		sensor_Value[1] == 1&&
		sensor_Value[2] == 1&&
		sensor_Value[3] == 1)
		{
			//duty = rotate;
			MotorCtrl3W(0,-speed_medium ,speed_quick );
		}//same 1 ==1 2==1
		
		//left
		else if (sensor_Value[0] == 1&&
		sensor_Value[2] == 0&&
		sensor_Value[3] == 0)
		{
			MotorCtrl3W(0,-rotate, speed_slow );
			S_Value[0]=sensor_Value[0];
		  S_Value[1]=sensor_Value[1];
		  S_Value[2]=sensor_Value[2];
		  S_Value[3]=sensor_Value[3];//record the value of sensor at last time 
			
		}
		
		//right
		else if (sensor_Value[0] == 0&&
		sensor_Value[1] == 0&&
		sensor_Value[3] == 1)
		{
			MotorCtrl3W(0,-speed_slow ,rotate );
			S_Value[0]=sensor_Value[0];
		  S_Value[1]=sensor_Value[1];
		  S_Value[2]=sensor_Value[2];
		  S_Value[3]=sensor_Value[3];//record the value of sensor at last time 
		}
		
		//pa
		else
		{
			//delay_1ms(300);
			if(S_Value[0] == 1)
			{
			duty = -1000;
			MotorCtrl3W(duty , duty, duty);//left
			}
			else
			{
			duty = 1000;
			MotorCtrl3W(duty , duty, duty);
			}
		}
		
		/*if (Read_key(KEY2) == 1)
		{
			duty = 0;
		}
		if (Read_key(KEY4) == 1)
		{
			duty -= 100;
		}
		MotorCtrl3W(0, -duty, duty);*/
		sprintf(txt, "Duty:%05d", duty);
		OLED_P8x16Str(10, 5, txt); // ��ʾ�ַ���
		led_toggle();
		delay_1ms(10);
	}
}
