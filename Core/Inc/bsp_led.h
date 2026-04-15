#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LED_0 = 0,
    LED_1 = 1,
    LED_2 = 2,
    LED_3 = 3,
    LED_COUNT = 4
} LedNumber_t;

typedef enum {
    LED_OFF = 0,
    LED_ON = 1
} LedState_t;

void BSP_LED_Init(void);
bool BSP_LED_SetState(LedNumber_t led_number, LedState_t state);
LedState_t BSP_LED_GetState(LedNumber_t led_number);
bool BSP_LED_Toggle(LedNumber_t led_number);
bool BSP_LED_SetAllStates(const LedState_t states[LED_COUNT]);
bool BSP_LED_GetAllStates(LedState_t states[LED_COUNT]);

#ifdef __cplusplus
}
#endif

#endif