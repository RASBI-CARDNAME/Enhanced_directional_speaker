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
  Description: Optimization and sampling rate improvements.
               Added second complementary PWM channel for dual driver setup.
==========================================================================
*/

#include <Arduino.h>
#include <STM32ADC.h>

#define PWM_OVERFLOW 1800 // 1800 = 40Khz

// Channel 1 Outputs for Driver 1
#define PWM_OUT_1 PA8       // PWM output 1 (TIM1 CH1)
#define PWM_OUT_COMP_1 PB13 // PA8 pin's complementary output (TIM1 CH1N)

// Channel 2 Outputs for Driver 2
#define PWM_OUT_2 PA9       // PWM output 2 (TIM1 CH2)                // <--- ADDED
#define PWM_OUT_COMP_2 PB14 // PA9 pin's complementary output (TIM1 CH2N) // <--- ADDED

#define ANALOG_PIN PA7    // ADC input
#define maxSamples 1      // sample buffer

HardwareTimer hTimer1 = HardwareTimer(1); // PA8, PB13, PA9, PB14 all USE TIMER 1
STM32ADC myADC(ADC1);                     // ADC object (use ADC 1)

uint16_t buffer[maxSamples];              // ADC resolution is 12bit. So use 16bit.
uint8 pins = 7;                           // USE PA7 pin

void isr(void); //Interrupt

void setup() {
  pinMode(PA7, INPUT_ANALOG);  // setup PIN PA7 for analog in
  
  // Setup PWM pins for Driver 1 (Channel 1)
  pinMode(PWM_OUT_1, PWM);  
  pinMode(PWM_OUT_COMP_1, PWM); 
  
  // Setup PWM pins for Driver 2 (Channel 2)
  pinMode(PWM_OUT_2, PWM);      // <--- ADDED
  pinMode(PWM_OUT_COMP_2, PWM); // <--- ADDED

  pinMode(PB2, OUTPUT);  // LED out

  // Make 40KHz CLK for 40KHz transducer
  hTimer1.pause();
  hTimer1.setPrescaleFactor(1);  // Timer PreScale Factor 1
  hTimer1.setOverflow(PWM_OVERFLOW);  // 72000000 / (const: PWM OVERFLOW=1800) = 40Khz
  hTimer1.setMode(4,TIMER_OUTPUT_COMPARE);
  hTimer1.setCompare(4,PWM_OVERFLOW);
  hTimer1.attachInterrupt(4,isr); // Attach ISR

  // Prepare to use Direct Regiser Access
  timer_dev *t = TIMER1; //refers t to Timer 1 memory location
  timer_reg_map r = t->regs;
  
  // Prepare to use PWM output AND complementary output
  // READ RM0008 reference manual.
  // TIM1 and TIM8 capture/compare enable register (TIMx_CCER)
  // bitSet function make bit 1.
  
  // Enable PWM output and complementary output for Channel 1
  bitSet(r.adv->CCER, 0); // CC1E: Enable CH1 output (PA8)
  bitSet(r.adv->CCER, 2); // CC1NE: Enable CH1N output (PB13)

  // Enable PWM output and complementary output for Channel 2
  bitSet(r.adv->CCER, 4); // CC2E: Enable CH2 output (PA9)    // <--- ADDED
  bitSet(r.adv->CCER, 6); // CC2NE: Enable CH2N output (PB14) // <--- ADDED


  // Update Timer
  hTimer1.refresh();
  hTimer1.resume();
  
  // Set initial duty cycle to 50% for both channels
  pwmWrite(PWM_OUT_1, PWM_OVERFLOW / 2); 
  pwmWrite(PWM_OUT_2, PWM_OVERFLOW / 2); // <--- ADDED
 
  // Set ADC to free running mode and transfer data via DMA
  // ADC use Timer3
  Timer3.setPeriod(10); // sampling rate = 100KHz, 10 micro seconds
  Timer3.setMasterModeTrGo(TIMER_CR2_MMS_UPDATE); // Timer3 send trigger signal(TRGO) on every 100KHz

  myADC.calibrate(); // Calibration ADC
  myADC.setSampleRate(ADC_SMPR_1_5); // ADC clock cycle = 1.5
  myADC.setPins(&pins,1); // Use PA7 as input

  // Set DMA(Direct Memory Access) and Save ADC result. 
  myADC.setDMA(buffer, maxSamples, (DMA_MINC_MODE | DMA_CIRC_MODE | DMA_HALF_TRNS | DMA_TRNS_CMPLT), NULL);

  // Set ADC to start conversion When external trigger signal from Timer3`s TRGO
  myADC.setTrigger(ADC_EXT_EV_TIM3_TRGO);

  // Start Analog digital convert
  myADC.startConversion();
}

void loop() {
    // Crash check
    digitalWrite(PB2, HIGH);
    delay(1000);
    digitalWrite(PB2, LOW);
    delay(1000);
}


 // Interrupt on Timer Overflow 
 // modulation part (PWM)
void isr(void) {
  // convert frequency to Pulse Width Modulation Duty Cycle Using DMA
  uint16_t pDuty = (uint16_t)map(buffer[0], 0, 4095, 0, PWM_OVERFLOW / 2 - 1);
  
  // Calculate the new compare value once
  uint16_t new_compare_value = PWM_OVERFLOW / 2 + pDuty; // <--- CHANGED for clarity

  // Write the same duty cycle to both channels
  pwmWrite(PWM_OUT_1, new_compare_value); // Sets TIM1->CCR1
  pwmWrite(PWM_OUT_2, new_compare_value); // Sets TIM1->CCR2 // <--- ADDED
}
