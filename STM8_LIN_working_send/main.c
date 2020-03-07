/**
  ******************************************************************************
  * @file    UART1_HyperTerminal_Interrupt\main.c
  * @author  MCD Application Team
  * @version  V2.2.0
  * @date     30-September-2014
  * @brief   This file contains the main function for UART1 using interrupts in 
  *          communication example.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
//#include "stm8s_eval.h"
volatile unsigned char state = 0x00;
volatile unsigned char pid_idle = 0x12;
volatile unsigned char pid_1 = 0x12;
volatile unsigned char pid_2 = 0x12;
volatile unsigned char pid = 0x12;
volatile unsigned char pid_slave1 = 0x12;
volatile unsigned char fr1 = 0x00;
volatile unsigned char fr2 = 0x55;
volatile unsigned char fr1_rx = 0x00;
volatile unsigned char fr2_rx = 0x55;
volatile int b = 3;
volatile int x = 0;
uint8_t button_1 = 0;

__IO uint32_t current_millis = 0; //--IO: volatile read/write 
volatile uint32_t idle_millis = 0;
uint32_t last_millis = 0;


/**
  * @addtogroup UART1_HyperTerminal_Interrupt
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void GPIO_Config(void);
static void CLK_Config(void);
static void UART1_Config(void); 
static void TIM4_Config(void);
void Delay(uint32_t nCount);
unsigned char Parity( unsigned char pid );
/* Private functions ---------------------------------------------------------*/
void p_LIN_wait_us(uint32_t n)
{
  volatile uint32_t p,t;

  // kleine Pause
  for(p=0;p<n;p++) {
    for(t=0;t<1;t++); // ca 56us when p = 2
  }
}
uint32_t millis(void)
{
	return current_millis;
}

void LIN_send(void) {
  UART1_SendBreak();
  while (!(UART1_GetFlagStatus(UART1_FLAG_SBK) == RESET));
    //while (USART_GetFlagStatus(USART1, UART1_FLAG_SBK) == RESET);
    
    //while (!(UART1_CR2_SBK));
    
  p_LIN_wait_us(2); //insert interbyte space
  UART1_SendData8(0x55);
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  p_LIN_wait_us(21);
  UART1_SendData8(pid);
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  p_LIN_wait_us(21);
  UART1_SendData8(fr1);         //Data 0
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  p_LIN_wait_us(21);
  UART1_SendData8(fr2);        //Data 1
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  
  
  unsigned int chk=0; // объявляем беззнаковую переменную в 16 бит
  unsigned char chk1; // объявляем беззнаковую переменную в 8 бит
  //chk+=pid; // Protected ID, в подсчете не участвует в версии 1.3
  chk+=fr1; // 1 байт данных
  chk1+=fr2; // 2 байт данных
    //chk+=DataTX(0x80); // 3 байт данных
    //chk+=DataTX(0x6A); // 4 байт данных
    //chk+=DataTX(0x76); // 5 байт данных
    //chk+=DataTX(0xE0); // 6 байт данных
    //chk+=DataTX(0xFE); // 7 байт данных
    //chk+=DataTX(0x00); // 8 байт данных
    chk1=chk>>8; // сдвигаем и пишем во вторую переменную
    //DataTX(~(chk1+chk)); // передаем итоговую контрольную сумму
  
  
  
  p_LIN_wait_us(21);
  UART1_SendData8(~(chk1+chk));        //CRC
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  
  /*
  p_LIN_wait_us(21);
  UART1_SendData8(0x55);        //Data 2
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  p_LIN_wait_us(21);
  UART1_SendData8(0x55);        //Data 3
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  p_LIN_wait_us(21);
  UART1_SendData8(0x55);
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  p_LIN_wait_us(21);
  UART1_SendData8(0x55);
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  p_LIN_wait_us(21);
  UART1_SendData8(0x55);
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  p_LIN_wait_us(21);
  UART1_SendData8(0x55);
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  p_LIN_wait_us(21);
  UART1_SendData8(0x55);
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
  */
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  /* Clock configuration -----------------------------------------*/
  CLK_Config();  

  /* GPIO configuration -----------------------------------------*/
  GPIO_Config();  

  /* UART1 configuration -----------------------------------------*/
  UART1_Config();  
  
  /* TIM4 configuration -----------------------------------------*/  
  TIM4_Config();

  GPIO_WriteHigh(GPIOD,GPIO_PIN_4);
  
  while (1)
  {
    state = 0;
    pid = pid_idle;
    fr1 = 0x02;
    fr2 = 0;
    //button_1 = GPIO_ReadInputPin(GPIOD, GPIO_PIN_3);
    if (GPIO_ReadInputPin(GPIOD, GPIO_PIN_3) == 0) {
      state = 1;
      fr1 = fr1 + 128;
    }
    if (GPIO_ReadInputPin(GPIOD, GPIO_PIN_2) == 0) {
      state = 1;
      fr1 = fr1 + 64;
    }
    if (GPIO_ReadInputPin(GPIOD, GPIO_PIN_1) == 0) {
      state = 1;
      //fr1 = fr1 + 32;
    }
    if (GPIO_ReadInputPin(GPIOC, GPIO_PIN_7) == 0) {
      state = 1;
      fr1 = fr1 + 16;
    }
    if (GPIO_ReadInputPin(GPIOC, GPIO_PIN_6) == 0) {
      state = 1;
      fr1 = fr1 + 8;
    }
    if (GPIO_ReadInputPin(GPIOC, GPIO_PIN_5) == 0) {
      state = 1;
      fr1 = fr1 + 4;
    }
    if (GPIO_ReadInputPin(GPIOC, GPIO_PIN_4) == 0) {
      state = 1;
      fr1 = fr1 + 2;
    }
    if (GPIO_ReadInputPin(GPIOC, GPIO_PIN_3) == 0) {
      state = 1;
      fr1 = fr1 + 32;
    }
    
    
    //GPIO_WriteReverse(GPIOB,GPIO_PIN_5);
    //Delay((uint32_t)0xFFFF);
    //x = a + b;
    
    if (state == 0) {
      pid = pid_idle;
    }
    else {
      pid = pid_2;
    }
    
    pid = Parity (pid);
    
    /*
    if (idle_millis >= 1000) {
      
      //need to reset frame receiving status
    }*/
    //LIN_send();
    uint32_t currentTime = millis();
    if (currentTime - last_millis >= 15) {
      GPIO_WriteReverse(GPIOB,GPIO_PIN_5);
      LIN_send();
      last_millis = currentTime;
        // need to reset idle time in order to restore connection
      idle_millis = 0; 
    }
    
    /*
    UART1_SendBreak();
    while (!(UART1_GetFlagStatus(UART1_FLAG_SBK) == RESET));
    //while (USART_GetFlagStatus(USART1, UART1_FLAG_SBK) == RESET);
    
    //while (!(UART1_CR2_SBK));
    
    p_LIN_wait_us(2); //insert interbyte space
    UART1_SendData8(0x55);
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
    p_LIN_wait_us(22);
    UART1_SendData8(pid);
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
    p_LIN_wait_us(22);
    UART1_SendData8(fr1);
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
    p_LIN_wait_us(22);
    UART1_SendData8(0x55);
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
    p_LIN_wait_us(22);
    UART1_SendData8(0x55);
    
    //below toggle for testing only
    Delay(1000);
    */
  }
}

/**
  * @brief  Configure system clock to run at 16Mhz
  * @param  None
  * @retval None
  */
static void CLK_Config(void)
{
    /* Initialization of the clock */
    /* Clock divider to HSI/1 */
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

/**
  * @brief  Configure LEDs available on the evaluation board
  * @param  None
  * @retval None
  */
static void GPIO_Config(void)
{
    GPIO_DeInit(GPIOB);
    GPIO_DeInit(GPIOD);
    GPIO_DeInit(GPIOC);
/* Initialize LEDs mounted on the Eval board */
  
  GPIO_Init(GPIOB,GPIO_PIN_5,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(GPIOD,GPIO_PIN_4,GPIO_MODE_OUT_PP_HIGH_FAST);
  
  GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOD, GPIO_PIN_1, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_IN_PU_NO_IT);
}

/**
  * @brief  Configure UART1 for the communication with HyperTerminal
  * @param  None
  * @retval None
  */
static void UART1_Config(void)
{
  /* EVAL COM (UART) configuration -----------------------------------------*/
  /* USART configured as follow:
        - BaudRate = 19200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Receive and transmit enabled
        - UART Clock disabled
  */
  UART1_Init((uint32_t)19200, UART1_WORDLENGTH_8D,UART1_STOPBITS_1, UART1_PARITY_NO,
                   UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);

  /* Enable the UART Receive interrupt: this interrupt is generated when the UART
    receive data register is not empty */
  //UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
  
  /* Enable the UART Transmit complete interrupt: this interrupt is generated 
     when the UART transmit Shift Register is empty */
  //UART1_ITConfig(UART1_IT_TXE, ENABLE);

  /* Enable UART */
  //UART1_Cmd(ENABLE);
  UART1_LINCmd(ENABLE);
    /* Enable general interrupts */
  enableInterrupts();
}

static void TIM4_Config(void)
{
        /* TIM4 configuration:
	- TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
	clock used is 16 MHz / 128 = 125 000 Hz
	- With 125 000 Hz we can generate time base:
	  max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
	  min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
	- In this example we need to generate a time base equal to 1 ms
	so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

	/* Time base configuration */
	TIM4_TimeBaseInit(TIM4_PRESCALER_128, 124);
	/* Clear TIM4 update flag */
	TIM4_ClearFlag(TIM4_FLAG_UPDATE);
	/* Enable update interrupt */
	TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);

	/* enable interrupts */
	enableInterrupts();

	/* Enable TIM4 */
	TIM4_Cmd(ENABLE);
}  

/**
  * @brief  Delay.
  * @param  nCount
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

/**
  * @brief  Odd Parity.
  * @param  b
  * @retval ret parity bit
  */
unsigned char Parity( unsigned char b )
{
unsigned char odd_p;
unsigned char even_p;

odd_p = b & 0x17; //0b00010111

odd_p = odd_p ^ (odd_p>>4);
odd_p = odd_p ^ (odd_p>>2);
odd_p = odd_p ^ (odd_p>>1);
odd_p = odd_p & 1;
odd_p = odd_p << 6;

even_p = b & 0x3A; //0b00111010
even_p = even_p ^ (even_p>>4);
even_p = even_p ^ (even_p>>2);
even_p = even_p ^ (even_p>>1);
even_p = (~even_p) & 1;
even_p = even_p << 7;

return b|even_p|odd_p;
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/*
void LinCommand (void) // Объявляемся
  {
   unsigned int chk=0; // объявляем беззнаковую переменную в 16 бит
   unsigned char chk1; // объявляем беззнаковую переменную в 8 бит
    DataTX(0x3c); // служебные байты, например адрес, в подсчете не участвуют
    chk+=DataTX(0xA6); // 1 байт данных
    chk+=DataTX(0xB4); // 2 байт данных
    chk+=DataTX(0x80); // 3 байт данных
    chk+=DataTX(0x6A); // 4 байт данных
    chk+=DataTX(0x76); // 5 байт данных
    chk+=DataTX(0xE0); // 6 байт данных
    chk+=DataTX(0xFE); // 7 байт данных
    chk+=DataTX(0x00); // 8 байт данных
    chk1=chk>>8; // сдвигаем и пишем во вторую переменную
    DataTX(~(chk1+chk)); // передаем итоговую контрольную сумму
 }

void LIN_UART_ISR(void) {
  uint16_t wert;

  if (USART_GetITStatus(LIN_UART, USART_IT_LBD) == SET) {
    //---------------------------
    // BreakField
    //---------------------------
    // flag loeschen
    USART_ClearITPendingBit(LIN_UART, USART_IT_LBD);

    LIN_SLAVE.mode=RECEIVE_SYNC;
  }


  if (USART_GetITStatus(LIN_UART, USART_IT_RXNE) == SET) {
    // Daten auslesen
    wert=USART_ReceiveData(LIN_UART);

    // check welcher Mode gerade aktiv ist
    if(LIN_SLAVE.mode==RECEIVE_SYNC) {
      //---------------------------
      // SyncField
      //---------------------------
      if(wert==LIN_SYNC_DATA) {
        LIN_SLAVE.mode=RECEIVE_ID;
      }
      else {
        LIN_SLAVE.mode=WAIT_4_BREAK;              
      }
    }
    else if(LIN_SLAVE.mode==RECEIVE_ID) {
      //---------------------------
      // IDField
      //---------------------------
      LIN_SLAVE.mode=ID_OK;
      LIN_FRAME.frame_id=(uint8_t)(wert);      
    }
    else if(LIN_SLAVE.mode==RECEIVE_DATA) {
      //---------------------------
      // DataField
      //---------------------------
      // Daten speichern
      LIN_FRAME.data[LIN_SLAVE.data_ptr]=(uint8_t)(wert);
      // Pointer weiterschalten
      LIN_SLAVE.data_ptr++;
      // check ob alle Daten empfangen sind
      if(LIN_SLAVE.data_ptr>=LIN_FRAME.data_len) {          
        LIN_SLAVE.mode=RECEIVE_CRC;
      }
    }
    else if(LIN_SLAVE.mode==RECEIVE_CRC) { 
      //---------------------------
      // CRCField
      //---------------------------     
      LIN_SLAVE.crc=(uint8_t)(wert);
      LIN_SLAVE.mode=WAIT_4_BREAK; 
    }
  }
}

*/