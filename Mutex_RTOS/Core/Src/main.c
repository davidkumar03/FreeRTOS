#include "stm32f405xx.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

TaskHandle_t task1Handle=NULL;
TaskHandle_t task2Handle=NULL;

xSemaphoreHandle xMutex;

#define GPIOAEN   (1U<<0)
#define clock     16000000U
#define baud_rate 115200U

void Task1( void * Parameters);
void Task2( void * Parameters);
void uart_tx_init(void);
void uart_write(uint8_t *ch);
uint8_t smsg[]="Mutex Created successfully";
uint8_t sharedres[10];

void Task1(void *Parameters)
{

	uint8_t msg1[]="SoftRTOS";
    while(1)
    {
    	if(xSemaphoreTake(xMutex,(TickType_t)0xFFFFFFFF)==pdTRUE)
    	{
    		int i;
	        for(i=0;i<sizeof(msg1);i++)
	        {
	        	sharedres[i]=msg1[i];
		        vTaskDelay(50);
	        }
	        sharedres[i]='\0';
	        uart_write(sharedres);
	        uart_write("\n\r");
	        xSemaphoreGive(xMutex);
    	}
    	vTaskDelay(100);
    }
}
void Task2(void *Parameters)
{
	uint8_t msg2[]="HardRTOS";
    while(1)
    {
    	if(xSemaphoreTake(xMutex,(TickType_t)0xFFFFFFFF)==pdTRUE)
    	{
    		int i;
	        for(i=0;i<sizeof(msg2);i++)
	        {
	        	sharedres[i]=msg2[i];
		        vTaskDelay(50);
	        }
	        sharedres[i]='\0';
	        uart_write(sharedres);
	        uart_write("\n\r");
	        xSemaphoreGive(xMutex);
    	}
    	vTaskDelay(100);
    }
}

int main(void)
{
	uart_tx_init();
	xMutex = xSemaphoreCreateMutex();
	if(xMutex != NULL )
	{
        uart_write(smsg);
        uart_write("\n\r");
	}
    xTaskCreate(Task1,"task1", 128,NULL,0,&task1Handle);
    xTaskCreate(Task2,"task2", 128,NULL,0,&task2Handle);
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
