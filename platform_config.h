
#ifndef PLATFORM_CONFIG_H
#define	PLATFORM_CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

#define DEBUG_LED_RCC          RCC_APB2Periph_GPIOA
#define DEBUG_LED_PORT         GPIOA
#define DEBUG_LED_PIN          GPIO_Pin_0

#define DEBUG_USART            USART1
#define DEBUG_USART_BAUD       9600
#define DEBUG_USART_IRQ        USART1_IRQn
#define DEBUG_USART_RCC        RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1
#define DEBUG_USART_TX         GPIOA
#define DEBUG_USART_TX_PIN     GPIO_Pin_9
#define DEBUG_USART_RX         GPIOA
#define DEBUG_USART_RX_PIN     GPIO_Pin_10

#define STATUS_LED_RCC         RCC_APB2Periph_GPIOA
#define STATUS_LED_PORT        GPIOA
#define STATUS_LED_PIN         GPIO_Pin_0

#define RF_TX_TIMER            TIM3
#define RF_TX_TIMER_CH_Init          TIM_OC3Init
#define RF_TX_TIMER_CH_PreloadConfig TIM_OC3PreloadConfig
#define RF_TX_TIMER_CH_SetCompare    TIM_SetCompare3
#define RF_TX_TIMER_RCC        RCC_APB1Periph_TIM3
#define RF_TX_RCC              RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO
#define RF_TX_PORT             GPIOB
#define RF_TX_PIN              GPIO_Pin_0

#define RF_RX_RCC              RCC_APB2Periph_GPIOB
#define RF_RX_PORT             GPIOB
#define RF_RX_PIN              GPIO_Pin_1

#ifdef	__cplusplus
}
#endif

#endif	/* PLATFORM_CONFIG_H */

