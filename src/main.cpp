// #include <Arduino.h>
#include "stm32f1xx_hal.h"

// #include "stm32f10x.h"
// #include "stm32f10x_conf.h"
// #include "stm32f10x_gpio.h"
// #include "stm32f10x_rcc.h"

#define pin_cmd GPIO_PIN_1
#define pin_clk GPIO_PIN_2
#define pin_d1 GPIO_PIN_3
#define pin_d2 GPIO_PIN_5
#define pin_d3 GPIO_PIN_6
#define pin_rst GPIO_PIN_4

void delay(unsigned long p)
{
  volatile unsigned long i;
  for(i=0;i<p ;i++);
}

void LcdSendData(uint32_t b1)
{
  GPIOA->BSRR = pin_cmd;
  delay(10);
  for (int i = 0; i < 18; ++i)
  {
    GPIOA->BSRR = pin_clk;
    delay(1);
    GPIOA->BRR = pin_clk;
    delay(1);
  }
}

void SendCmd(uint8_t b1, uint8_t b2, uint8_t b3)
{
  GPIOA->BRR = pin_cmd;
  for (uint8_t i = 0; i < sizeof(uint8_t) * 8; ++i)
  {
    if ((b1 >> i) & 1)
      GPIOA->BSRR = pin_d1;
    else
      GPIOA->BRR = pin_d1;
    GPIOA->BSRR = pin_clk;
    // delay(1);
    GPIOA->BRR = pin_clk;
  }
  for (uint8_t i = 0; i < sizeof(uint8_t) * 8; ++i)
  {
    if ((b2 >> i) & 1)
      GPIOA->BSRR = pin_d1;
    else
      GPIOA->BRR = pin_d1;
    GPIOA->BSRR = pin_clk;
    // delay(1);
    GPIOA->BRR = pin_clk;
  }
  for (uint8_t i = 0; i < sizeof(uint8_t) * 8; ++i)
  {
    if ((b3 >> i) & 1)
      GPIOA->BSRR = pin_d1;
    else
      GPIOA->BRR = pin_d1;
    GPIOA->BSRR = pin_clk;
    // delay(1);
    GPIOA->BRR = pin_clk;
  }
  GPIOA->BRR = pin_d1;
  delay(20);
}

void LcdInit()
{
  GPIOA->BSRR = pin_d2 | pin_d3 | pin_rst;
  GPIOA->BRR = pin_rst;
  delay(13000000);
  GPIOA->BSRR = pin_rst;
  delay(1000);
  SendCmd(0xf, 0x0, 0x0);
  SendCmd(0xcf, 0x0, 0x0);
  SendCmd(0x2f, 0xc0, 0x0);
  GPIOA->BRR = pin_d2 | pin_d3;
  delay(2300000);
  SendCmd(0xf, 0x0, 0x0);
  SendCmd(0xc, 0x0, 0x0);
  SendCmd(0x8f, 0x0, 0x0);
  GPIOA->BSRR = pin_cmd;
  for (int i = 0; i < 10000; ++i){
    LcdSendData(0);
  }
}


int main() {
  HAL_Init();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init( GPIOC , &GPIO_InitStructure);

  GPIO_InitStructure.Pin = pin_cmd | pin_clk | pin_d1 | pin_rst | pin_d2 | pin_d3;
  HAL_GPIO_Init( GPIOA , &GPIO_InitStructure);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  GPIOA->ODR = pin_rst;
  delay(1000);
  LcdInit();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  while(1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_13);
    HAL_Delay(1000);
  }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1) {}
}


void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void SVC_Handler(void)
{
}


void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}
