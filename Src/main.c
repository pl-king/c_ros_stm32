/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: MPU6050陀螺仪
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "usart/bsp_debug_usart.h"
#include "lcd/bsp_lcd.h"
#include "stdlib.h"
#include "spiflash/bsp_spiflash.h"
#include "i2c/bsp_i2c.h"
#include "mpu6050/mpu6050.h"
#include "led/bsp_led.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define TASK_ENABLE 0

/* 私有变量 ------------------------------------------------------------------*/
uint8_t main_sta=0; //用作处理主函数各种if，去掉多余的flag（1打印里程计）（2调用计算里程计数据函数）（3串口接收成功）（4串口接收失败）
uint8_t aRxBuffer;
uint8_t aRxBuffer2;
/* 扩展变量 ------------------------------------------------------------------*/
extern unsigned int Task_Delay[NumOfTask];

/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{
  short Accel[3];
	short Gyro [3];
	short Temp;
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init();
 
   /* 初始化3.5寸TFT液晶模组，一般优先于调试串口初始化 */
  BSP_LCD_Init(); 
  /*LED初始化*/
  LED_GPIO_Init();
  /*I2C初始化*/
  I2C_InitGPIO();
  
  /*MPU6050初始化*/
  MPU6050_Init();  
  LCD_Clear(0,0,LCD_DEFAULT_WIDTH,LCD_DEFAULT_HEIGTH,BLACK);
  HAL_Delay(1000);
  /* 开背光 */
  LCD_BK_ON();
  
  LCD_DispString_EN_CH(70,50,(uint8_t *)"YS-F4Pro开发板",BLACK,BLUE,USB_FONT_24);
	if (MPU6050ReadID() == 0)
	{	
    LCD_DispString_EN_CH(20,150,(uint8_t *)"检测不到MPU6050，停机",BLACK,BLUE,USB_FONT_24);
    printf("检测不到MPU6050，停机");
	  while(1);
  }
    /* 使能接收，进入中断回调函数 */
  HAL_UART_Receive_IT(&husart_debug,&aRxBuffer,1);
	HAL_UART_Transmit_IT(&husart_debug, (uint8_t *)&"123",3);
  /* 无限循环 */
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
      //printf("\r\n加速度： %8d%8d%8d    ",Accel[0],Accel[1],Accel[2]);
      MPU6050ReadGyro(Gyro);
      //printf("陀螺仪： %8d%8d%8d    ",Gyro[0],Gyro[1],Gyro[2]);
      
      MPU6050_ReturnTemp(&Temp);
			aRxBuffer2=123;
     //printf("温度： %d",Temp);
			HAL_UART_Transmit_IT(&husart_debug, (uint8_t *)&"123",3);
      Task_Delay[1]=100;//此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是100ms
    }
    
  }
}
/**
  * 函数功能: 串口接收完成回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

  HAL_UART_Transmit(&husart_debug,&aRxBuffer,1,0);
  HAL_UART_Receive_IT(&husart_debug,&aRxBuffer,1);
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
