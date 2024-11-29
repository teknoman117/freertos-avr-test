// FreeRTOS Test
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"

static SemaphoreHandle_t TransmitSemaphore, ReceiveSemaphore;
static StreamBufferHandle_t TransmitBuffer, ReceiveBuffer;
static StreamBufferHandle_t Buffer;

// read a byte
#ifdef USART0_RX_vect
ISR(USART0_RX_vect) {
#else
ISR(USART_RX_vect) {
#endif
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t cIn = UDR0;

    xStreamBufferSendFromISR(ReceiveBuffer, &cIn, 1, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// send a byte
#ifdef USART0_UDRE_vect
ISR(USART0_UDRE_vect) {
#else
ISR(USART_UDRE_vect) {
#endif
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t cOut;

    if (xStreamBufferReceiveFromISR(TransmitBuffer, &cOut, 1, &xHigherPriorityTaskWoken)) {
        UDR0 = cOut;
    } else {
        // Disable interrupt if the transmit queue is empty
        UCSR0B &= ~_BV(UDRIE0);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static int _putchar(char c, FILE *stream) {
    while (!xStreamBufferSend(TransmitBuffer, &c, 1, 1));
    UCSR0B |= _BV(UDRIE0);
    return 0;
}

static int _getchar(FILE *stream) {
    uint8_t cIn;
    while (!xStreamBufferReceive(ReceiveBuffer, &cIn, 1, 1));
    return cIn;
}

static FILE uart0_stream =
        FDEV_SETUP_STREAM(_putchar, _getchar, _FDEV_SETUP_RW);

void setupUART(void) {
    TransmitBuffer = xStreamBufferCreate(128, 1);
    ReceiveBuffer = xStreamBufferCreate(128, 1);
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

void TaskWrite(void *pvParameters) {
    while (1) {
        int num = 0;
        if (xStreamBufferReceive(Buffer, &num, sizeof(num), 1000 / portTICK_PERIOD_MS)
                == sizeof(num)) {
            if (xSemaphoreTake(TransmitSemaphore, 10) == pdTRUE) {
                printf_P(PSTR("[%u] Received: %d\r\n"), xTaskGetTickCount(), num);
                xSemaphoreGive(TransmitSemaphore);
            }
        }
    }
}

void TaskGenerate(void *pvParameters) {
    TickType_t wakeUpTime = xTaskGetTickCount();

    while (1) {
        int num = rand();
        xStreamBufferSend(Buffer, &num, sizeof(num), 1);
        vTaskDelayUntil(&wakeUpTime, 100 / portTICK_PERIOD_MS);
    }
}

void vApplicationMallocFailedHook(void) {
    while (1);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char*) {
    while (1);
}

void main(void) {
    setupUART();
    sei();

    Buffer = xStreamBufferCreate(128, sizeof(int));

    xTaskCreate(TaskWrite, "Write", 192, NULL, 2, NULL);
    xTaskCreate(TaskGenerate, "Generate", 96, NULL, 2, NULL);
    vTaskStartScheduler();
}