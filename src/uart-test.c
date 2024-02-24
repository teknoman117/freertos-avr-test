// FreeRTOS Test
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

static QueueHandle_t TransmitQueue, ReceiveQueue;
static SemaphoreHandle_t TransmitSemaphore, ReceiveSemaphore;

// read a byte
ISR(USART_RX_vect) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t cIn = UDR0;

    xQueueSendFromISR(ReceiveQueue, &cIn, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// send a byte
ISR(USART_UDRE_vect) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t cOut;

    if (xQueueReceiveFromISR(TransmitQueue, &cOut, &xHigherPriorityTaskWoken)) {
        UDR0 = cOut;
    } else {
        // Disable interrupt if the transmit queue is empty
        UCSR0B &= ~_BV(UDRIE0);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void vApplicationMallocFailedHook(void) {
    while (1);
}

static int _putchar(char c, FILE *stream) {
    while (xQueueSendToBack(TransmitQueue, &c, 1) != pdPASS);
    UCSR0B |= _BV(UDRIE0);
    return 0;
}

static int _getchar(FILE *stream) {
    uint8_t cIn;
    while (xQueueReceive(ReceiveQueue, &cIn, 1) != pdPASS);
    return cIn;
}

static FILE uart0_stream =
        FDEV_SETUP_STREAM(_putchar, _getchar, _FDEV_SETUP_RW);

void setupUART(void) {
    TransmitQueue = xQueueCreate(128, 1);
    ReceiveQueue = xQueueCreate(128, 1);
    TransmitSemaphore = xSemaphoreCreateMutex();
    ReceiveSemaphore = xSemaphoreCreateMutex();
    stdout = stdin = stderr = &uart0_stream;
    
    // Configure 115.2k 8n1
    UBRR0L = 16;
    UBRR0H = 0;
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
    UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
    UCSR0A = _BV(U2X0);
}

void TaskBlink1(void *pvParameters) {
    // Blink every second
    TickType_t wakeUpTime = xTaskGetTickCount();
    while (1) {
        PORTB |= _BV(PORTB5);
        vTaskDelayUntil(&wakeUpTime, 500 / portTICK_PERIOD_MS);
        PORTB &= ~_BV(PORTB5);
        vTaskDelayUntil(&wakeUpTime, 500 / portTICK_PERIOD_MS);
    }
}

void TaskSend1(void *pvParameters) {
    TickType_t wakeUpTime = xTaskGetTickCount();

    while (1) {
        if (xSemaphoreTake(TransmitSemaphore, 10) == pdTRUE) {
            printf_P(PSTR("[%u] I am task 1, I send every 333 ms\r\n"), xTaskGetTickCount());
            xSemaphoreGive(TransmitSemaphore);
            vTaskDelayUntil(&wakeUpTime, 333 / portTICK_PERIOD_MS);
        }
    }
}

void TaskSend2(void *pvParameters) {
    TickType_t wakeUpTime = xTaskGetTickCount();

    while (1) {
        if (xSemaphoreTake(TransmitSemaphore, 10) == pdTRUE) {
            printf_P(PSTR("[%u] I am task 2, I send every 500 ms\r\n"), xTaskGetTickCount());
            xSemaphoreGive(TransmitSemaphore);
            vTaskDelayUntil(&wakeUpTime, 500 / portTICK_PERIOD_MS);
        }
    }
}

void TaskSend3(void *pvParameters) {
    TickType_t wakeUpTime = xTaskGetTickCount();

    while (1) {
        if (xSemaphoreTake(TransmitSemaphore, 10) == pdTRUE) {
            printf_P(PSTR("[%u] I am task 3, I send every 1000 ms\r\n"), xTaskGetTickCount());
            xSemaphoreGive(TransmitSemaphore);
            vTaskDelayUntil(&wakeUpTime, 1000 / portTICK_PERIOD_MS);
        }
    }
}

void setupLED(void) {
    // Set LED pin to an output and default to off
    DDRB |= _BV(PORTB5);
    PORTB &= ~_BV(PORTB5);
}

void main(void) {
    setupLED();
    setupUART();
    sei();

    xTaskCreate(TaskBlink1, "Blink1", 128, NULL, 2, NULL);
    xTaskCreate(TaskSend1, "Send1", 128, NULL, 2, NULL);
    xTaskCreate(TaskSend2, "Send2", 128, NULL, 2, NULL);
    xTaskCreate(TaskSend3, "Send3", 128, NULL, 2, NULL);
    vTaskStartScheduler();
}