
#include "time.h"

extern void on_dma1_ch2_irq();
extern void on_tim4_irq();
extern void on_usart1_irq();

void NMI_Handler() {
}

void HardFault_Handler() {
}

void MemManage_Handler() {
}

void BusFault_Handler() {
}

void UsageFault_Handler() {
}

void SVC_Handler() {
}

void DebugMon_Handler() {
}

void PendSV_Handler() {
}

void DMA1_Channel2_IRQHandler() {
  on_dma1_ch2_irq();
}

void TIM4_IRQHandler() {
  on_tim4_irq();
}

void SysTick_Handler() {
  time_SysTick_Handler();
}

void USART1_IRQHandler(void) {
  on_usart1_irq();
}

