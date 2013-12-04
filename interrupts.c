
#include "time.h"

void on_time1_cc_irq();

void NMI_Handler(void) {
}

void HardFault_Handler(void) {
}

void MemManage_Handler(void) {
}

void BusFault_Handler(void) {
}

void UsageFault_Handler(void) {
}

void SVC_Handler(void) {
}

void DebugMon_Handler(void) {
}

void PendSV_Handler(void) {
}

void TIM1_CC_IRQHandler(void) {
  on_time1_cc_irq();
}

void SysTick_Handler(void) {
  time_SysTick_Handler();
}
