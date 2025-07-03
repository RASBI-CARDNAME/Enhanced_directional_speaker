/*
=========================================================================
  Ultrasonic Parametric Speaker Source Code for STM32
  Copyright (C) 2019,2020 Gene Ruebsamen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

==========================================================================
  Modified by: RASBI
  Date: 2025-06-16
  Description: NEED TEST!!!!!
  
  Final version with dual driver support and improved DSB-SC modulation.
==========================================================================
*/

#include <Arduino.h>
#include <STM32ADC.h>

// --- Configuration ---
#define PWM_OVERFLOW 1800        // 72MHz / 1800 = 40kHz PWM carrier frequency

// Driver 1 Output Pins (TIM1 Channel 1)
#define PWM_OUT_1 PA8
#define PWM_OUT_COMP_1 PB13

// Driver 2 Output Pins (TIM1 Channel 2)
#define PWM_OUT_2 PA9
#define PWM_OUT_COMP_2 PB14

#define ANALOG_PIN PA7           // ADC input pin
#define LED_PIN PB2              // Status/Crash indicator LED
#define maxSamples 1             // DMA buffer size for ADC

// --- Global Objects and Variables ---
HardwareTimer hTimer1 = HardwareTimer(1);
STM32ADC myADC(ADC1);

uint16_t buffer[maxSamples];     // DMA buffer for ADC results
uint8 pins = 7;                  // ADC pin index for PA7
const uint16_t adcMid = 2048;    // Midpoint for 12-bit ADC (assuming 0-3.3V audio input centered at 1.65V)

void isr(void);                  // Timer interrupt service routine declaration

void setup() {
  // --- Pin Initialization ---
  pinMode(ANALOG_PIN, INPUT_ANALOG);
  pinMode(LED_PIN, OUTPUT);

  // Initialize all PWM pins for both drivers
  pinMode(PWM_OUT_1, PWM);
  pinMode(PWM_OUT_COMP_1, PWM);
  pinMode(PWM_OUT_2, PWM);
  pinMode(PWM_OUT_COMP_2, PWM);

  // --- Timer 1 Setup for 40kHz Complementary PWM ---
  hTimer1.pause();
  hTimer1.setPrescaleFactor(1);
  hTimer1.setOverflow(PWM_OVERFLOW);
  
  // Set up a dummy channel to attach the ISR. 
  // The actual PWM modes for CH1 and CH2 will be set via registers.
  hTimer1.setMode(4, TIMER_OUTPUT_COMPARE);
  hTimer1.setCompare(4, PWM_OVERFLOW);
  hTimer1.attachInterrupt(4, isr);

  // --- Direct Register Access for Advanced Timer Features ---
  timer_dev *t = TIMER1;
  timer_reg_map r = t->regs;

  // Enable complementary outputs for both channels in TIM1_CCER register
  bitSet(r.adv->CCER, 0); // CC1E: Enable CH1 output (PA8)
  bitSet(r.adv->CCER, 2); // CC1NE: Enable CH1N complementary output (PB13)
  bitSet(r.adv->CCER, 4); // CC2E: Enable CH2 output (PA9)
  bitSet(r.adv->CCER, 6); // CC2NE: Enable CH2N complementary output (PB14)

  // Start the timer
  hTimer1.refresh();
  hTimer1.resume();

  // Set initial duty cycle to 50% for both channels
  pwmWrite(PWM_OUT_1, PWM_OVERFLOW / 2);
  pwmWrite(PWM_OUT_2, PWM_OVERFLOW / 2);

  // --- ADC and DMA Setup ---
  // Use Timer 3 to trigger ADC conversions at 100kHz
  Timer3.setPeriod(10); // 10 microseconds -> 100kHz sampling rate
  Timer3.setMasterModeTrGo(TIMER_CR2_MMS_UPDATE); // Trigger on update event

  myADC.calibrate();
  myADC.setSampleRate(ADC_SMPR_1_5);
  myADC.setPins(&pins, 1);
  
  // Configure DMA to continuously transfer ADC results to the 'buffer'
  myADC.setDMA(buffer, maxSamples,
               (DMA_MINC_MODE | DMA_CIRC_MODE | DMA_HALF_TRNS | DMA_TRNS_CMPLT), NULL);
               
  myADC.setTrigger(ADC_EXT_EV_TIM3_TRGO); // Set ADC to trigger from Timer 3
  myADC.startConversion();
}

void loop() {
  // Main loop can be used for other tasks.
  // Blinking LED provides a simple visual check that the MCU is not frozen.
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

/**
 * @brief Interrupt Service Routine (ISR) called by Timer 1 at 40kHz.
 * This function performs the Double-Sideband Suppressed-Carrier (DSB-SC)
 * modulation by adjusting the PWM duty cycle based on the audio input.
 */
void isr(void) {
  // 1. Get the latest ADC sample from the DMA buffer and calculate its signed deviation from the center point.
  int16_t deviation = (int16_t)buffer[0] - adcMid;

  // 2. Map the audio deviation to a PWM duty cycle shift.
  // The full audio range (-2048 to +2047) is mapped to the full duty cycle range (0% to 100%).
  int16_t dutyShift = map(deviation, -adcMid, adcMid, -PWM_OVERFLOW / 2 + 1, PWM_OVERFLOW / 2 - 1);

  // 3. Calculate the final duty cycle by adding the shift to the 50% center point.
  // 'constrain' ensures the value stays within the valid range [0, PWM_OVERFLOW-1].
  uint16_t pwmDuty = constrain(PWM_OVERFLOW / 2 + dutyShift, 0, PWM_OVERFLOW - 1);

  // 4. Apply the same duty cycle to both PWM channels.
  // The hardware will automatically generate the complementary signals.
  pwmWrite(PWM_OUT_1, pwmDuty); // Controls Driver 1
  pwmWrite(PWM_OUT_2, pwmDuty); // Controls Driver 2
}
