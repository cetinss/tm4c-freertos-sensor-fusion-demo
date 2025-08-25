/*
 * train_system.c
 *
 * Author: Sena Cetin
 * Created on: 18 August 2025
 *
 * Implementation of hardware drivers for the train control system.
 * Includes robust, timeout-enabled functions for sensor readings.
 */
#include "train_system.h"

// Access the global system clock variable defined in main.c
extern uint32_t g_ui32SysClock;

void GPIO_Init(void) {
    // Enable GPIO peripherals for all connected hardware
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM); // For Tunnel LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // For Warning & Heartbeat LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL); // For HC-SR04 Sensor
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // For LDR Sensor

    // Wait for the peripherals to be ready before configuration
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    // Configure pin types for each component
    GPIOPinTypeGPIOOutput(LED_TUNNEL_PORT, LED_TUNNEL_PIN);
    GPIOPinTypeGPIOOutput(LED_WARN_PORT, LED_WARN_PIN);
    GPIOPinTypeGPIOOutput(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN);
    GPIOPinTypeGPIOOutput(TRIGGER_PORT, TRIGGER_PIN);
    GPIOPinTypeGPIOInput(ECHO_PORT, ECHO_PIN);
}

void UART_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure UART for standard I/O at 115200 baud
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

void LDR_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    GPIOPinTypeADC(LDR_PORT_BASE, LDR_PIN);
    ADCSequenceConfigure(ADC0_BASE, LDR_ADC_SEQUENCER, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, LDR_ADC_SEQUENCER, 0, LDR_ADC_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, LDR_ADC_SEQUENCER);
    ADCIntClear(ADC0_BASE, LDR_ADC_SEQUENCER);
}

// Robust LDR reading function with a timeout to prevent system freeze
uint32_t LDR_ReadValue(void) {
    uint32_t value;
    volatile uint32_t timeout = 100000;
    ADCProcessorTrigger(ADC0_BASE, LDR_ADC_SEQUENCER);
    while (!ADCIntStatus(ADC0_BASE, LDR_ADC_SEQUENCER, false) && (timeout > 0)) {
        timeout--;
    }
    if (timeout == 0) {
        return 0xFFFFFFFF; // Return error code on timeout
    } else {
        ADCIntClear(ADC0_BASE, LDR_ADC_SEQUENCER);
        ADCSequenceDataGet(ADC0_BASE, LDR_ADC_SEQUENCER, &value);
        return value;
    }
}

void Distance_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFFFFFF);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

// Robust distance reading function with timeouts to prevent system freeze
float Distance_Read(void) {
    uint32_t start, end, diff;
    volatile uint32_t timeout_start = 100000;
    volatile uint32_t timeout_end = 4000000;

    // 1. Send a 10us trigger pulse to the sensor
    GPIOPinWrite(TRIGGER_PORT, TRIGGER_PIN, 0);
    SysCtlDelay(g_ui32SysClock / (3 * 1000000)); // ~1us delay
    GPIOPinWrite(TRIGGER_PORT, TRIGGER_PIN, TRIGGER_PIN);
    SysCtlDelay(g_ui32SysClock / (3 * 100000));  // 10us delay
    GPIOPinWrite(TRIGGER_PORT, TRIGGER_PIN, 0);

    // Give the sensor a brief moment to respond to prevent timing issues
    SysCtlDelay(g_ui32SysClock / 10000); // ~100us delay

    // 2. Wait for the ECHO pin to go HIGH (with a timeout)
    while ((GPIOPinRead(ECHO_PORT, ECHO_PIN) == 0) && (timeout_start > 0)) {
        timeout_start--;
    }
    if (timeout_start == 0) return -1.0f; // Error: Echo never started

    start = TimerValueGet(TIMER0_BASE, TIMER_A);

    // 3. Wait for the ECHO pin to go LOW again (with a timeout)
    while ((GPIOPinRead(ECHO_PORT, ECHO_PIN)) && (timeout_end > 0)) {
        timeout_end--;
    }
    if (timeout_end == 0) return -2.0f; // Error: Echo never finished

    end = TimerValueGet(TIMER0_BASE, TIMER_A);

    // Calculate pulse duration, accounting for timer overflow
    if (end < start)
        diff = (0xFFFFFFFF - start) + end;
    else
        diff = end - start;

    // Convert time to distance in cm
    float distance = ((float)diff / (float)g_ui32SysClock) * (34300.0 / 2.0);
    return distance;
}
