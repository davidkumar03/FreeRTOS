#include "stm32f405xx.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

TaskHandle_t task1Handle=NULL;
TaskHandle_t task2Handle=NULL;

xSemaphoreHandle xBinarysem_1;
xSemaphoreHandle xBinarysem_2;

#define GPIOAEN   (1U<<0)
#define clock     16000000U
#define baud_rate 115200U

void Task1( void * Parameters);
void Task2( void * Parameters);
void uart_tx_init(void);
void uart_write(uint8_t *ch);
uint8_t sharedres[10];
uint8_t New[]="\r\n";
uint8_t freeMsg[] = "Semaphore Released\r\n";

void Task1(void *Parameters)
{
    uint8_t msg1[] = "SoftRTOS";
    while(1)
    {
        if(xSemaphoreTake(xBinarysem_1, ( TickType_t ) 0xffffffffUL) == pdTRUE)
        {
            int i;
            for(i = 0; i < sizeof(msg1); i++)
            {
                sharedres[i] = msg1[i];
                vTaskDelay(50);
            }
            sharedres[i] = '\0';
            uart_write(sharedres);
            uart_write(New);

            // Give xBinarysem_2 to wake Task2
            xSemaphoreGive(xBinarysem_2);
            uart_write(freeMsg);
        }
    }
}

void Task2(void *Parameters)
{
    uint8_t msg2[] = "HardRTOS";
    while(1)
    {
        if(xSemaphoreTake(xBinarysem_2, ( TickType_t ) 0xffffffffUL) == pdTRUE)
        {
            int i;
            for(i = 0; i < sizeof(msg2); i++)
            {
                sharedres[i] = msg2[i];
                vTaskDelay(50);
            }
            sharedres[i] = '\0';
            uart_write(sharedres);
            uart_write(New);

            // Give xBinarysem_1 to wake Task1
            xSemaphoreGive(xBinarysem_1);
            uart_write(freeMsg);
        }
    }
}

int main(void)
{
    uart_tx_init();

    // Create two binary semaphores
    xBinarysem_1 = xSemaphoreCreateBinary();
    xBinarysem_2 = xSemaphoreCreateBinary();

    // INITIAL TOKENS
    xSemaphoreGive(xBinarysem_1);   // Start with Task1
    // xBinarysem_2 is NOT given so Task2 waits

    xTaskCreate(Task1, "task1", 128, NULL, 1, &task1Handle);
    xTaskCreate(Task2, "task2", 128, NULL, 1, &task2Handle);

    vTaskStartScheduler();
    return 0;
}

void uart_tx_init(void)
{
    RCC->AHB1ENR |= GPIOAEN;

    // PA2 alternate function AF7
    GPIOA->MODER &= ~(3U << (2*2));
    GPIOA->MODER |=  (2U << (2*2));
    GPIOA->AFR[0] &= ~(0xF << (4*2));
    GPIOA->AFR[0] |=  (7   << (4*2));

    RCC->APB1ENR |= (1U<<17);

    USART2->BRR = (clock + (baud_rate/2U))/baud_rate;
    USART2->CR1 |= (1U<<3);   // TE
    USART2->CR1 |= (1U<<13);  // UE
}

void uart_write(uint8_t *ch)
{
    int i=0;
    while(ch[i] != '\0')
    {
        while(!(USART2->SR & (1U<<7))){};
        USART2->DR = (ch[i] & 0xFF);
        i++;
    }
}
