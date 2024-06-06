#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

void Error_Handler(void);

#define SOLENOID6_EN_Pin GPIO_PIN_13
#define SOLENOID6_EN_GPIO_Port GPIOC
#define SOLENOID7_EN_Pin GPIO_PIN_14
#define SOLENOID7_EN_GPIO_Port GPIOC
#define SOLENOID8_EN_Pin GPIO_PIN_15
#define SOLENOID8_EN_GPIO_Port GPIOC
#define SOLENOID0_FB_Pin GPIO_PIN_0
#define SOLENOID0_FB_GPIO_Port GPIOC
#define SOLENOID1_FB_Pin GPIO_PIN_1
#define SOLENOID1_FB_GPIO_Port GPIOC
#define SOLENOID2_FB_Pin GPIO_PIN_2
#define SOLENOID2_FB_GPIO_Port GPIOC
#define SOLENOID3_FB_Pin GPIO_PIN_3
#define SOLENOID3_FB_GPIO_Port GPIOC
#define SOLENOID4_FB_Pin GPIO_PIN_0
#define SOLENOID4_FB_GPIO_Port GPIOA
#define SOLENOID5_FB_Pin GPIO_PIN_1
#define SOLENOID5_FB_GPIO_Port GPIOA
#define SOLENOID6_FB_Pin GPIO_PIN_2
#define SOLENOID6_FB_GPIO_Port GPIOA
#define SOLENOID7_FB_Pin GPIO_PIN_3
#define SOLENOID7_FB_GPIO_Port GPIOA
#define SOLENOID8_FB_Pin GPIO_PIN_4
#define SOLENOID8_FB_GPIO_Port GPIOA
#define PT0_Pin GPIO_PIN_5
#define PT0_GPIO_Port GPIOA
#define PT1_Pin GPIO_PIN_6
#define PT1_GPIO_Port GPIOA
#define PT2_Pin GPIO_PIN_7
#define PT2_GPIO_Port GPIOA
#define PT3_Pin GPIO_PIN_4
#define PT3_GPIO_Port GPIOC
#define PT4_Pin GPIO_PIN_5
#define PT4_GPIO_Port GPIOC
#define PWR0_Pin GPIO_PIN_0
#define PWR0_GPIO_Port GPIOB
#define PWR1_Pin GPIO_PIN_1
#define PWR1_GPIO_Port GPIOB
#define ETH_nRST_Pin GPIO_PIN_2
#define ETH_nRST_GPIO_Port GPIOB
#define ETH_TX_Pin GPIO_PIN_10
#define ETH_TX_GPIO_Port GPIOB
#define ETH_RX_Pin GPIO_PIN_11
#define ETH_RX_GPIO_Port GPIOB
#define ALARM_Pin GPIO_PIN_12
#define ALARM_GPIO_Port GPIOB
#define ETH_CTS_Pin GPIO_PIN_13
#define ETH_CTS_GPIO_Port GPIOB
#define ETH_RTS_Pin GPIO_PIN_14
#define ETH_RTS_GPIO_Port GPIOB
#define ARMED_Pin GPIO_PIN_15
#define ARMED_GPIO_Port GPIOB
#define EMATCH0_FIRE_Pin GPIO_PIN_6
#define EMATCH0_FIRE_GPIO_Port GPIOC
#define EMATCH0_CONT_Pin GPIO_PIN_7
#define EMATCH0_CONT_GPIO_Port GPIOC
#define EMATCH1_FIRE_Pin GPIO_PIN_8
#define EMATCH1_FIRE_GPIO_Port GPIOC
#define EMATCH1_CONT_Pin GPIO_PIN_9
#define EMATCH1_CONT_GPIO_Port GPIOC
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TC2_nCS_Pin GPIO_PIN_15
#define TC2_nCS_GPIO_Port GPIOA
#define TC_SCK_Pin GPIO_PIN_10
#define TC_SCK_GPIO_Port GPIOC
#define TC_MISO_Pin GPIO_PIN_11
#define TC_MISO_GPIO_Port GPIOC
#define TC3_nCS_Pin GPIO_PIN_12
#define TC3_nCS_GPIO_Port GPIOC
#define TC4_nCS_Pin GPIO_PIN_2
#define TC4_nCS_GPIO_Port GPIOD
#define TC5_nCS_Pin GPIO_PIN_3
#define TC5_nCS_GPIO_Port GPIOB
#define SOLENOID0_EN_Pin GPIO_PIN_4
#define SOLENOID0_EN_GPIO_Port GPIOB
#define SOLENOID1_EN_Pin GPIO_PIN_5
#define SOLENOID1_EN_GPIO_Port GPIOB
#define SOLENOID2_EN_Pin GPIO_PIN_6
#define SOLENOID2_EN_GPIO_Port GPIOB
#define SOLENOID3_EN_Pin GPIO_PIN_7
#define SOLENOID3_EN_GPIO_Port GPIOB
#define SOLENOID4_EN_Pin GPIO_PIN_8
#define SOLENOID4_EN_GPIO_Port GPIOB
#define SOLENOID5_EN_Pin GPIO_PIN_9
#define SOLENOID5_EN_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif
