/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define Green_LED_Pin GPIO_PIN_0
#define Green_LED_GPIO_Port GPIOB
#define RGB_On_Pin GPIO_PIN_14
#define RGB_On_GPIO_Port GPIOF
#define RGB_LED2_Pin GPIO_PIN_11
#define RGB_LED2_GPIO_Port GPIOE
#define RGB_LED3_Pin GPIO_PIN_13
#define RGB_LED3_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define VCOM_TX_Pin GPIO_PIN_8
#define VCOM_TX_GPIO_Port GPIOD
#define VCOM_RX_Pin GPIO_PIN_9
#define VCOM_RX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Blue_LED_Pin GPIO_PIN_7
#define Blue_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

// Includes
#include "stm32f4xx_hal.h"

#ifndef LCD_ROUTINES_H
#define LCD_ROUTINES_H

///////////////////////////////////////////////////////////////////////////////
// Pinbelegung


#define PortData	GPIOF
#define RCC_PortData RCC_AHB1Periph_GPIOE

#define LCD_D7		GPIO_PIN_15		// PE15
#define LCD_D6		GPIO_PIN_14		// PE13
#define LCD_D5		GPIO_PIN_13		// PE11
#define LCD_D4		GPIO_PIN_12		// PE9
#define LCD_D3		GPIO_PIN_11		// PE7


#define PortCnt	 GPIOA
#define RCC_PortCnt RCC_AHB1Periph_GPIOB

#define LCD_RS		GPIO_PIN_5		// PB15
#define LCD_RW		GPIO_PIN_6		// PB13
#define LCD_E			GPIO_PIN_7 	// PB11


////////////////////////////////////////////////////////////////////////////////
// LCD Ausführungszeiten (MS=Millisekunden, US=Mikrosekunden)
 
#define LCD_BOOTUP_MS           15
#define LCD_ENABLE_US           5
#define LCD_WRITEDATA_US        5  //formally: 46
#define LCD_COMMAND_US          5   //formally: 42
 
#define LCD_SOFT_RESET_MS       5
#define LCD_SET_4BITMODE_MS     10
 
#define LCD_CLEAR_DISPLAY_MS    2
#define LCD_CURSOR_HOME_MS      2
 
////////////////////////////////////////////////////////////////////////////////
// Zeilendefinitionen des verwendeten LCD
// Die Einträge hier sollten für ein LCD mit einer Zeilenlänge von 16 Zeichen passen
// Bei anderen Zeilenlängen müssen diese Einträge angepasst werden
 
#define LCD_DDADR_LINE1         0x00
#define LCD_DDADR_LINE2         0x40   

////////////////////////////////////////////////////////////////////////////////
// GPIO Pin Setup: muss ganz am Anfang des Programms aufgerufen werden.
void lcd_gpio_setup (void);

////////////////////////////////////////////////////////////////////////////////
// Initialisierung: muss ganz am Anfang des Programms aufgerufen werden.
void lcd_init( void );

 
////////////////////////////////////////////////////////////////////////////////
// LCD löschen
void lcd_clear( void );
 
////////////////////////////////////////////////////////////////////////////////
// Cursor in die 1. Zeile, 0-te Spalte
void lcd_home( void );
 
////////////////////////////////////////////////////////////////////////////////
// Cursor an eine beliebige Position 
void lcd_setcursor( uint8_t x, uint8_t y );
 
////////////////////////////////////////////////////////////////////////////////
// Ausgabe eines einzelnen Zeichens an der aktuellen Cursorposition 
void lcd_data (int zahl);
 
////////////////////////////////////////////////////////////////////////////////
// Ausgabe eines Strings an der aktuellen Cursorposition 
void lcd_string( const char *data );
  
////////////////////////////////////////////////////////////////////////////////
// Ausgabe eines Kommandos an das LCD.
void lcd_command( int zahl );

////////////////////////////////////////////////////////////////////////////////
// Delay Funktion
//void delay (unsigned long ms);
 
 /* Private function prototypes -----------------------------------------------*/
static void delay(__IO uint32_t nTime);

////////////////////////////////////////////////////////////////////////////////
//LCD Befehle und Argumente.
//Zur Verwendung in lcd_command
 
// Clear Display -------------- 0b00000001
#define LCD_CLEAR_DISPLAY       0x01    
 
// Cursor Home ---------------- 0b0000001x
#define LCD_CURSOR_HOME         0x02    
 
// Set Entry Mode ------------- 0b000001xx
#define LCD_SET_ENTRY           0x04    
 
#define LCD_ENTRY_DECREASE      0x00    
#define LCD_ENTRY_INCREASE      0x02    
#define LCD_ENTRY_NOSHIFT       0x00    
#define LCD_ENTRY_SHIFT         0x01    
 
// Set Display ---------------- 0b00001xxx
#define LCD_SET_DISPLAY         0x08    
 
#define LCD_DISPLAY_OFF         0x00    
#define LCD_DISPLAY_ON          0x04    
#define LCD_CURSOR_OFF          0x00    
#define LCD_CURSOR_ON           0x02    
#define LCD_BLINKING_OFF        0x00    
#define LCD_BLINKING_ON         0x01    
 
// Set Shift ------------------ 0b0001xxxx
#define LCD_SET_SHIFT           0x10    
 
#define LCD_CURSOR_MOVE         0x00    
#define LCD_DISPLAY_SHIFT       0x08    
#define LCD_SHIFT_LEFT          0x00    
#define LCD_SHIFT_RIGHT         0x01    
 
// Set Function --------------- 0b001xxxxx
#define LCD_SET_FUNCTION        0x20    
 
#define LCD_FUNCTION_4BIT       0x00    
#define LCD_FUNCTION_8BIT       0x10    
#define LCD_FUNCTION_1LINE      0x00    
#define LCD_FUNCTION_2LINE      0x08    
#define LCD_FUNCTION_5X7        0x00    
#define LCD_FUNCTION_5X10       0x04    
 
#define LCD_8BIT_RESET          0x30    
#define LCD_4BIT_RESET          0x20    
 
// Set CG RAM Address --------- 0b01xxxxxx  (Character Generator RAM)
#define LCD_SET_CGADR           0x40    
 
#define LCD_GC_CHAR0            0
#define LCD_GC_CHAR1            1
#define LCD_GC_CHAR2            2
#define LCD_GC_CHAR3            3
#define LCD_GC_CHAR4            4
#define LCD_GC_CHAR5            5
#define LCD_GC_CHAR6            6
#define LCD_GC_CHAR7            7
 
// Set DD RAM Address --------- 0b1xxxxxxx  (Display Data RAM)
#define LCD_SET_DDADR           0x80    
 
#endif 


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
