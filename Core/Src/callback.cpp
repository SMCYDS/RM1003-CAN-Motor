//
// Created by Thomas on 2026/10/3.
//

#include "../Inc/callback.h"
#include "main.h"
#include "tim.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim==&htim1)
    {
        HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin)
    {

    }
}