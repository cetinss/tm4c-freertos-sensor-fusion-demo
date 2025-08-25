/*
 * train_system.h
 *
 * Author: Sena Cetin
 * Created on: 18 August 2025
 *
 * Hardware driver interface for the FreeRTOS-based train control system.
 * This file defines the hardware connections and function prototypes for the sensors and LEDs.
 */

#ifndef TRAIN_SYSTEM_H_
#define TRAIN_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

// -- LDR Sensor Definitions --
#define LDR_ADC_CHANNEL      ADC_CTL_CH0      // AIN0 is on pin PE3
#define LDR_ADC_SEQUENCER    3
#define LDR_PORT_BASE        GPIO_PORTE_BASE
#define LDR_PIN              GPIO_PIN_3

// -- HC-SR04 Distance Sensor Definitions --
#define TRIGGER_PORT  GPIO_PORTL_BASE
#define TRIGGER_PIN   GPIO_PIN_5  // PL5 is used for Trigger
#define ECHO_PORT     GPIO_PORTL_BASE
#define ECHO_PIN      GPIO_PIN_4  // PL4 is used for Echo

// -- LED Feedback Definitions --
#define LED_TUNNEL_PORT      GPIO_PORTM_BASE
#define LED_TUNNEL_PIN       GPIO_PIN_3

#define LED_WARN_PORT        GPIO_PORTN_BASE
#define LED_WARN_PIN         GPIO_PIN_1

#define LED_HEARTBEAT_PORT   GPIO_PORTN_BASE
#define LED_HEARTBEAT_PIN    GPIO_PIN_0 // On-board LED for system health feedback

// -- Function Prototypes --

void GPIO_Init(void);
void LDR_Init(void);
uint32_t LDR_ReadValue(void);
void Distance_Init(void);
float Distance_Read(void);
void UART_Init(void);

#endif /* TRAIN_SYSTEM_H_ */
