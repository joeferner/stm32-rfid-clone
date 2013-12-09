
#include "time.h"

void on_dma1_ch2_irq();
void on_tim1_cc_irq();

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

void TIM1_CC_IRQHandler(void) {
  on_tim1_cc_irq();
}

void SysTick_Handler() {
  time_SysTick_Handler();
}
