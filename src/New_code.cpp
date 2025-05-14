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
  Date: 2025-05-14
  Description: Optimization, sound improvement.
  ==========================================================================
*/

#include <Arduino.h>
#include <STM32ADC.h>

#define PWM_OVERFLOW 1800        // 72MHz / 1800 = 40kHz PWM
#define PWM_OUT PA8              // PWM main output (TIM1 CH1)
#define PWM_OUT_COMP PB13        // Complementary output (TIM1 CH1N)
#define ANALOG_PIN PA7           // ADC input pin
#define LED_PIN PB2              // Crash indicator LED
#define maxSamples 1             // DMA buffer size

HardwareTimer hTimer1 = HardwareTimer(1);
STM32ADC myADC(ADC1);

uint16_t buffer[maxSamples];     // DMA buffer
uint8 pins = 7;                  // ADC pin index
const uint16_t adcMid = 2048;    // Midpoint for 12-bit ADC

void isr(void);                  // Timer interrupt (for PWM modulation)

void setup() {
  pinMode(ANALOG_PIN, INPUT_ANALOG);
  pinMode(PWM_OUT, PWM);
  pinMode(PWM_OUT_COMP, PWM);
  pinMode(LED_PIN, OUTPUT);

  // Setup Timer1 for 40kHz PWM
  hTimer1.pause();
  hTimer1.setPrescaleFactor(1);
  hTimer1.setOverflow(PWM_OVERFLOW);
  hTimer1.setMode(4, TIMER_OUTPUT_COMPARE);
  hTimer1.setCompare(4, PWM_OVERFLOW);
  hTimer1.attachInterrupt(4, isr);

  // Enable complementary outputs via direct register
  timer_dev *t = TIMER1;
  timer_reg_map r = t->regs;
  bitSet(r.adv->CCER, 0); // enable PA8
  bitSet(r.adv->CCER, 2); // enable PB13

  hTimer1.refresh();
  hTimer1.resume();
  pwmWrite(PWM_OUT, PWM_OVERFLOW / 2); // 50% duty at start

  // Setup ADC with Timer3 trigger
  Timer3.setPeriod(10); // 100kHz
  Timer3.setMasterModeTrGo(TIMER_CR2_MMS_UPDATE);

  myADC.calibrate();
  myADC.setSampleRate(ADC_SMPR_1_5);
  myADC.setPins(&pins, 1);
  myADC.setDMA(buffer, maxSamples,
               (DMA_MINC_MODE | DMA_CIRC_MODE | DMA_HALF_TRNS | DMA_TRNS_CMPLT), NULL);
  myADC.setTrigger(ADC_EXT_EV_TIM3_TRGO);
  myADC.startConversion();
}

void loop() {
  // Crash check: blink LED every second
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

// ISR: Called at 40kHz, modulate PWM duty based on ADC input
void isr(void) {
  // Compute signed deviation from mid-point
  int16_t deviation = (int16_t)buffer[0] - adcMid;

  // Map deviation to duty shift (± PWM_OVERFLOW/2)
  int16_t dutyShift = map(deviation, -adcMid, adcMid, -PWM_OVERFLOW / 2 + 1, PWM_OVERFLOW / 2 - 1);

  // Final duty with center offset
  uint16_t pwmDuty = constrain(PWM_OVERFLOW / 2 + dutyShift, 0, PWM_OVERFLOW - 1);

  pwmWrite(PWM_OUT, pwmDuty);
}
