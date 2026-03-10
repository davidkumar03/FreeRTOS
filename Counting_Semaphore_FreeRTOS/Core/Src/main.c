#include "stm32f405xx.h"
#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

enum {BUF_SIZE = 5};

static const uint32_t num_prod_task = 5;
static const uint32_t num_cons_task = 2;
static const uint32_t num_writes    = 3;

#define clock     16000000U
#define baud_rate 115200U

void uart_tx_init(void);
void uart_write_single(uint8_t);
void uart_write_multi(uint8_t*);

void print_system_state(void);

/* Shared buffer */
static uint32_t buf[BUF_SIZE];
static uint32_t head = 0;
static uint32_t tail = 0;

/* FreeRTOS objects */
static SemaphoreHandle_t mutex;
static SemaphoreHandle_t sem_empty;
static SemaphoreHandle_t sem_filled;

/* Producer IDs */
static int prod_id[5];


/* ================= DEBUG FUNCTION ================= */

void print_system_state(void)
{
    char msg[120];

    uint32_t empty  = uxSemaphoreGetCount(sem_empty);
    uint32_t filled = uxSemaphoreGetCount(sem_filled);

    sprintf(msg,"Task: %s\r\n", pcTaskGetName(NULL));
    uart_write_multi((uint8_t*)msg);

    sprintf(msg,"Buffer: [%lu %lu %lu %lu %lu]\r\n",
            buf[0],buf[1],buf[2],buf[3],buf[4]);
    uart_write_multi((uint8_t*)msg);

    sprintf(msg,"Head=%lu Tail=%lu\r\n", head, tail);
    uart_write_multi((uint8_t*)msg);

    sprintf(msg,"sem_empty=%lu sem_filled=%lu\r\n",empty, filled);
    uart_write_multi((uint8_t*)msg);
}


/* ================= PRODUCER ================= */

void producer(void *parameters)
{
    int num = *(int *)parameters;

    for(int i = 0; i < num_writes; i++)
    {
    	xSemaphoreTake(sem_empty, portMAX_DELAY);

    	xSemaphoreTake(mutex, portMAX_DELAY);

    	buf[head] = num;
    	head = (head + 1) % BUF_SIZE;
    	uart_write_multi("\r\nProducer wrote: ");
		uart_write_single(num + '0');
		uart_write_multi("\r\n");
		print_system_state();
    	xSemaphoreGive(mutex);

    	xSemaphoreGive(sem_filled);


    }

    vTaskDelete(NULL);
}


/* ================= CONSUMER ================= */

void consumer(void *parameters)
{
    uint8_t val;

    while(1)
    {
    	xSemaphoreTake(sem_filled, portMAX_DELAY);

    	xSemaphoreTake(mutex, portMAX_DELAY);

    	val = buf[tail];
    	buf[tail] = 0;
    	tail = (tail + 1) % BUF_SIZE;

    	uart_write_multi("\r\nConsumer read: ");
    	uart_write_single(val + '0');
    	uart_write_multi("\r\n");

    	print_system_state();
    	xSemaphoreGive(mutex);

    	xSemaphoreGive(sem_empty);

    }
}


/* ================= MAIN ================= */

int main(void)
{
    uart_tx_init();

    uart_write_multi("\r\nFreeRTOS Producer Consumer Working\r\n");

    mutex      = xSemaphoreCreateMutex();
    sem_empty  = xSemaphoreCreateCounting(BUF_SIZE, BUF_SIZE);
    sem_filled = xSemaphoreCreateCounting(BUF_SIZE, 0);

    for(int i=0;i<num_prod_task;i++)
    {
        prod_id[i] = i;

        xTaskCreate(producer,"Producer",256,(void*)&prod_id[i],7,NULL);
    }

    for(int i=0;i<num_cons_task;i++)
    {
        xTaskCreate(consumer,"Consumer",256,NULL,7,NULL);
    }

    uart_write_multi("Tasks created\r\n");

    vTaskStartScheduler();

    while(1)
    {
    }
}


/* ================= UART DRIVER ================= */

void uart_tx_init(void)
{
    RCC->AHB1ENR |= (1U<<0);

    GPIOA->MODER &= ~(3U << (2*2));
    GPIOA->MODER |=  (2U << (2*2));

    GPIOA->AFR[0] &= ~(0xF << (4*2));
    GPIOA->AFR[0] |=  (7 << (4*2));

    RCC->APB1ENR |= (1U<<17);

    USART2->BRR = (clock + (baud_rate/2U))/baud_rate;

    USART2->CR1 |= (1U<<3);
    USART2->CR1 |= (1U<<13);
}


void uart_write_single(uint8_t ch)
{
    while(!(USART2->SR & (1U<<7)));
    USART2->DR = ch;
}


void uart_write_multi(uint8_t *ch)
{
    int i = 0;

    while(ch[i] != '\0')
    {
        while(!(USART2->SR & (1U<<7)));
        USART2->DR = ch[i];
        i++;
    }
}
