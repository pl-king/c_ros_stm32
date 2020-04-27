/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: MPU6050������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "usart/bsp_debug_usart.h"
#include "lcd/bsp_lcd.h"
#include "stdlib.h"
#include "spiflash/bsp_spiflash.h"
#include "i2c/bsp_i2c.h"
#include "mpu6050/mpu6050.h"
#include "led/bsp_led.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define TASK_ENABLE 0

/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
extern unsigned int Task_Delay[NumOfTask];

/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{
  short Accel[3];
	short Gyro [3];
	short Temp;
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();
 
   /* ��ʼ��3.5��TFTҺ��ģ�飬һ�������ڵ��Դ��ڳ�ʼ�� */
  BSP_LCD_Init(); 
  /*LED��ʼ��*/
  LED_GPIO_Init();
  /*I2C��ʼ��*/
  I2C_InitGPIO();
  
  /*MPU6050��ʼ��*/
  MPU6050_Init();  
  LCD_Clear(0,0,LCD_DEFAULT_WIDTH,LCD_DEFAULT_HEIGTH,BLACK);
  HAL_Delay(1000);
  /* ������ */
  LCD_BK_ON();
  
  LCD_DispString_EN_CH(70,50,(uint8_t *)"YS-F4Pro������",BLACK,BLUE,USB_FONT_24);
	if (MPU6050ReadID() == 0)
	{	
    LCD_DispString_EN_CH(20,150,(uint8_t *)"��ⲻ��MPU6050��ͣ��",BLACK,BLUE,USB_FONT_24);
    printf("��ⲻ��MPU6050��ͣ��");
	  while(1);
  }
  
  /* ����ѭ�� */
  while (1)
  {
    if(Task_Delay[0]==TASK_ENABLE)
    {
      LED1_TOGGLE;
      Task_Delay[0]=1000;
    }
    
    if(Task_Delay[1]==0)
    {
      MPU6050ReadAcc(Accel);			
      printf("\r\n���ٶȣ� %8d%8d%8d    ",Accel[0],Accel[1],Accel[2]);
      MPU6050ReadGyro(Gyro);
      printf("�����ǣ� %8d%8d%8d    ",Gyro[0],Gyro[1],Gyro[2]);
      
      MPU6050_ReturnTemp(&Temp);
      printf("�¶ȣ� %d",Temp);
      
      Task_Delay[1]=100;//��ֵÿ1ms���1������0�ſ������½����������ִ�е�������100ms
    }    
  }
}


/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
