/*
 * main.c
 *
 * Author: Sena Cetin
 * Created on: 18 August 2025
 *
 * Main application for the FreeRTOS-based Train Control System.
 * This file defines two tasks that are synchronized using Task Notifications
 * to read sensor data and print it to a UART console. A Mutex is used
 * to protect the shared UART resource.
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "train_system.h"

/* Global variable for system clock frequency */
uint32_t g_ui32SysClock;

/* Handles for task synchronization and resource management */
SemaphoreHandle_t g_uartMutex;
TaskHandle_t g_ldrTaskHandle = NULL;
TaskHandle_t g_distanceTaskHandle = NULL;

/* Task function prototypes */
void TaskLDR(void *pvParameters);
void TaskDistance(void *pvParameters);

/* Hardware setup function prototype */
static void prvSetupHardware(void);

int main(void)
{
    prvSetupHardware();
    g_uartMutex = xSemaphoreCreateMutex();

    if (g_uartMutex != NULL)
    {
        UARTprintf("Train Control System Initialized\n");
        UARTprintf("--------------------------------\n");

        xTaskCreate(TaskLDR, "LDR_Task", 256, NULL, 1, &g_ldrTaskHandle);
        xTaskCreate(TaskDistance, "Distance_Task", 256, NULL, 1, &g_distanceTaskHandle);

        // Give the initial notification to start the LDR task and the chain
        xTaskNotifyGive(g_ldrTaskHandle);

        // Start the FreeRTOS scheduler. Control is now passed to the RTOS.
        vTaskStartScheduler();
    }
    // This line should never be reached in a successful application.
    for (;;);
}

/**
 * Task to handle the LDR sensor. It waits for a notification, reads the
 * light level, controls the tunnel LED, and prints the value to the UART console.
 */
void TaskLDR(void *pvParameters)
{
    uint32_t ldrValue;
    while (1)
    {
        // Wait here until notified by the Distance task. This saves CPU cycles.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ldrValue = LDR_ReadValue();

        if (ldrValue < 90) { // Threshold for darkness
            GPIOPinWrite(LED_TUNNEL_PORT, LED_TUNNEL_PIN, LED_TUNNEL_PIN);
        } else {
            GPIOPinWrite(LED_TUNNEL_PORT, LED_TUNNEL_PIN, 0);
        }

        // Take the UART mutex to ensure exclusive access for printing.
        if (xSemaphoreTake(g_uartMutex, portMAX_DELAY) == pdTRUE) {
            if(ldrValue == 0xFFFFFFFF) {
                 UARTprintf("LDR: Sensor Error!\n");
            } else {
                 UARTprintf("LDR Value: %u\n", ldrValue);
            }
            // Release the mutex so other tasks can use the UART.
            xSemaphoreGive(g_uartMutex);
        }
        xTaskNotifyGive(g_distanceTaskHandle); // Notify the next task

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * Task to handle the HC-SR04 distance sensor. It waits for a notification,
 * reads the distance, controls the warning LED, and prints the value.
 */
void TaskDistance(void *pvParameters)
{
    float distance;
    while (1)
    {
        // Wait here until notified by the LDR task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        distance = Distance_Read();

        if (distance > 0 && distance < 20.0) { // Threshold for warning
            GPIOPinWrite(LED_WARN_PORT, LED_WARN_PIN, LED_WARN_PIN);
        } else {
            GPIOPinWrite(LED_WARN_PORT, LED_WARN_PIN, 0);
        }

        // Take the UART mutex to ensure exclusive access for printing.
        if (xSemaphoreTake(g_uartMutex, portMAX_DELAY) == pdTRUE) {
            if(distance < 0) {
                UARTprintf("Distance: Sensor Error!\n");
            } else {
                int d = (distance * 100);
                UARTprintf("Distance: %d.%02d cm\n", d / 100, d % 100);
            }
            // Release the mutex.
            xSemaphoreGive(g_uartMutex);
        }

        // Notify the LDR task that it is its turn to run.
        xTaskNotifyGive(g_ldrTaskHandle);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * Initializes the system clock and all hardware peripherals.
 */
static void prvSetupHardware(void)
{
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);
    GPIO_Init();
    LDR_Init();
    Distance_Init();
    UART_Init();
}

// --- FreeRTOS hook functions for error handling and system health monitoring ---

void vApplicationMallocFailedHook(void) {
    // This function is called if a call to pvPortMalloc() fails.
    // Indicates insufficient FreeRTOS heap memory.
    taskDISABLE_INTERRUPTS();
    for(;;);
}
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    // This function is called if a task overflows its stack.
    (void) pxTask;
    (void) pcTaskName;
    taskDISABLE_INTERRUPTS();
    for(;;);
}
void vApplicationIdleHook(void) {
    // This hook is called repeatedly by the idle task when there are no other tasks to run.
    // Toggling a "heartbeat" LED here is a good way to see that the system is alive.
    GPIOPinWrite(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN, LED_HEARTBEAT_PIN);
    SysCtlDelay(g_ui32SysClock / 50); // Short delay
    GPIOPinWrite(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN, 0);
}
