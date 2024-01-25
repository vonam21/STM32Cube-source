#include "main.h"
#include "stm32wbxx_hal.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();

    // Khởi tạo GPIO cho LED (ví dụ: GPIO_PIN_5 trên Nucleo)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    while (1) {
        // Bật LED
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

        // Đợi một chút
        HAL_Delay(1000);

        // Tắt LED
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

        // Đợi một chút
        HAL_Delay(1000);
    }
}
