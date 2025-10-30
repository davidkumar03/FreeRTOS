#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stdio.h"
#include "stdint.h"

void FreeRTOS_init();
void uart_tx_init(void);
void uart_write(uint8_t *ch);
void Task1( void * pvParameters);
void Task2( void * pvParameters);
QueueHandle_t myQueue;
uint8_t empty_msg[]="Empty Queue \n\r";
uint8_t Recv_msg[]="Message Recived: ";
uint8_t newline[]="\n\r";
uint32_t Queuesize=5;

#define GPIOAEN   (1U<<0)
#define clock     16000000U
#define baud_rate 115200U

int main(void)
{
	uart_tx_init();
	FreeRTOS_init();
	return 0;
}

void Task1( void * pvParameters)
{
	uint8_t Txbuff[30];
	myQueue = xQueueCreate(Queuesize,sizeof(Txbuff));
	sprintf(Txbuff,"Queue Number 1");
	xQueueSend(myQueue,( void * )Txbuff,( TickType_t ) 0 );
	sprintf(Txbuff,"Queue Number 2");
	xQueueSend(myQueue,( void * )Txbuff,( TickType_t ) 0 );
	sprintf(Txbuff,"Queue Number 3");
	xQueueSend(myQueue,( void * )Txbuff,( TickType_t ) 0 );
	for(;;)
	{

	}
}

void Task2( void * pvParameters)
{
	uint8_t Rxbuff[30];
	for(;;)
	{
		if(myQueue!=0)
		{
			if(uxQueueMessagesWaiting(myQueue)!=0)
			{
			if(xQueueReceive(myQueue,( void *)Rxbuff,(TickType_t)5))
			{
				uart_write(Recv_msg);
				uart_write(Rxbuff);
				uart_write(newline);
			}
			}
			else
			{
				uart_write(empty_msg);
				break;
			}
		}
	}
}

void FreeRTOS_init()
{
	xTaskCreate(Task1,"Task1",128,NULL,1,NULL);
	xTaskCreate(Task2,"Task2",128,NULL,1,NULL);
	vTaskStartScheduler();
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
