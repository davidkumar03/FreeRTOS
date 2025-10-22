#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

void LEDTask(void *pvParameters);
TaskHandle_t Task1Handle;

void GPIO_toggle(void);
void delay_ms(uint32_t ms);
void EXTI_init(void);
void FreeRTOS_init(void);

int main(void)
{
    EXTI_init();
    FreeRTOS_init();
    while (1);
}

// -------------------- FreeRTOS setup --------------------
void FreeRTOS_init(void)
{
    xTaskCreate(LEDTask, "LED", 128, NULL, 1, &Task1Handle);
    vTaskStartScheduler();
}

void LEDTask(void *pvParameters)
{
    for (;;)
    {
        // Wait for notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        GPIO_toggle();
    }
}

// -------------------- GPIO + EXTI config --------------------
void EXTI_init(void)
{
    // Enable GPIOA and SYSCFG clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // PA15 input (00)
    GPIOA->MODER &= ~(3U << (15 * 2));

    // Connect EXTI15 to PA15 -> EXTICR[3]
    SYSCFG->EXTICR[3] &= ~(0xF << 12);  // Clear bits for EXTI15
    SYSCFG->EXTICR[3] |=  (0x0 << 12);  // 0 = Port A

    // Unmask interrupt
    EXTI->IMR  |= (1U << 15);
    EXTI->FTSR |= (1U << 15);  // Falling edge trigger
    EXTI->RTSR &= ~(1U << 15); // (optional) disable rising edge

    // Clear any pending flag
    EXTI->PR = (1U << 15);

    // Enable EXTI15_10 interrupt line in NVIC
    NVIC_SetPriority(EXTI15_10_IRQn, 6);   // 6 is lower than 5 (OK)
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// -------------------- ISR --------------------
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1U << 15))
    {
        EXTI->PR = (1U << 15); // clear pending bit

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(Task1Handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// -------------------- LED + Delay --------------------
void GPIO_toggle(void)
{
    // Enable GPIOC clock
    RCC->AHB1ENR |= (1U<<2);

    // PC6 output
    GPIOC->MODER &= ~(3U << (6 * 2));
    GPIOC->MODER |=  (1U << (6 * 2));

    // Toggle PC6
    GPIOC->ODR ^= (1U << 6);
    delay_ms(100);
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < (ms * 16000); i++)
        __NOP();
}
