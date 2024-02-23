// FreeRTOS Test
#include <avr/io.h>

#include "FreeRTOS.h"
#include "task.h"

void vApplicationMallocFailedHook(void) {
    while (1);
}

void TaskBlink1(void *pvParameters) {
    // Set LED pin to an output
    DDRB |= _BV(PORTB5);

    // Blink every second
    TickType_t wakeUpTime = xTaskGetTickCount();
    while (1) {
        PORTB |= _BV(PORTB5);
        vTaskDelayUntil(&wakeUpTime, 500 / portTICK_PERIOD_MS);
        PORTB &= ~_BV(PORTB5);
        vTaskDelayUntil(&wakeUpTime, 500 / portTICK_PERIOD_MS);
    }
}

void TaskServo1(void *pvParameters) {
    // Configure Fast-PWM to overflow at 333 Hz.
    // At 16 MHz, gives 32000 steps across a 180 degree range
    DDRB |= _BV(PORTB2) | _BV(PORTB1);
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    ICR1 = (F_CPU / 333) - 1;

    // 1.5 ms
    OCR1A = 24000;
    OCR1B = 24000;
    TickType_t wakeUpTime = xTaskGetTickCount();
    vTaskDelayUntil(&wakeUpTime, 2000 / portTICK_PERIOD_MS);

    while (1) {
        // 0.5 ms
        OCR1A = 8000;
        // 1.0 ms
        OCR1B = 16000;
        vTaskDelayUntil(&wakeUpTime, 2500 / portTICK_PERIOD_MS);

        // 2.5 ms
        OCR1A = 40000;
        // 2.0 ms
        OCR1B = 32000;
        vTaskDelayUntil(&wakeUpTime, 2500 / portTICK_PERIOD_MS);
    }
}

void main(void) {
    xTaskCreate(TaskBlink1, "Blink", 128, NULL, 2, NULL);
    xTaskCreate(TaskServo1, "Servo", 128, NULL, 2, NULL);
    vTaskStartScheduler();
}