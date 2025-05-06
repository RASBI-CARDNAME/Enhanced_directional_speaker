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
  Date: 2025-05-06
  Description: Optimization and sampling rate improvements

  ==========================================================================
*/

#include <Arduino.h>
#include <STM32ADC.h>

#define PWM_OVERFLOW 1800 // 1800 = 40Khz
#define PWM_OUT PA8       // PWM output (TIM1 CH1)
#define PWM_OUT_COMP PB13 // PA8 pin`s complementary output (TIM1 CH1N)
#define ANALOG_PIN PA7    // ADC input
#define maxSamples 1      // sample buffer

HardwareTimer hTimer1 = HardwareTimer(1); // PA8, PB13 USE TIMER 1
STM32ADC myADC(ADC1);                     // ADC object (use ADC 1)

uint16_t buffer[maxSamples];              // ADC resolution is 12bit. So use 16bit.
uint8 pins = 7;                           // USE PA7 pin

void isr(void); //Interrupt

void setup() {
  pinMode(PA7, INPUT_ANALOG);  // setup PIN PA7 for analog in
  pinMode(PWM_OUT, PWM);  // Uses Timer1 / Channel 1
  pinMode(PWM_OUT_COMP, PWM); // Timer1 / Channel 1 `s complementary output
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
  bitSet(r.adv->CCER,0); //use PA8 as PWM output
  bitSet(r.adv->CCER,2); //use PB13 as complementary output

// Update Timer
  hTimer1.refresh();
  hTimer1.resume();
  pwmWrite(PWM_OUT,PWM_OVERFLOW/2);  // 50% duty cycle for initial start

 
// Set ADC to free running mode and transfer data via DMA
// ADC use Timer3
  Timer3.setPeriod(10); // sampling rate = 100KHz, 10 micro seconds, 10/1000000 = 100*1000 = 100KHz
  Timer3.setMasterModeTrGo(TIMER_CR2_MMS_UPDATE); // Timer3 send trigger signal(TRGO) on every 100KHz

  myADC.calibrate(); // Calibration ADC
  myADC.setSampleRate(ADC_SMPR_1_5); // ADC clock cycle = 1.5
  myADC.setPins(&pins,1); // Use PA7 as input

// Set DMA(Direct Memory Access) and Save ADC result. 
  myADC.setDMA(buffer, maxSamples, (DMA_MINC_MODE | DMA_CIRC_MODE | DMA_HALF_TRNS | DMA_TRNS_CMPLT), NULL);
// maxSamples: number of samples to collect
// DMA_MINC_MODE: increment memory pointer after each transfer
// DMA_CIRC_MODE: circular buffer mode (restart from beginning when buffer is full)
// DMA_HALF_TRNS, DMA_TRNS_CMPLT: enable half-transfer and transfer-complete interrupts
// NULL: no callback function

// Set ADC to start conversion When external trigger signal from Timer3`s TRGO
  myADC.setTrigger(ADC_EXT_EV_TIM3_TRGO);

// Start Analog digital convert
  myADC.startConversion();
}

void loop() {
    // Crash check
    digitalWrite(PB2 HIGH);
    delay(1000);
    digitalWrite(PB2, LOW);
    delay(1000);
}


 // Interrupt on Timer Overflow 
 // modulation part (PWM)
void isr(void) {
  // convert frequency to Pulse Width Modulation Duty Cycle Using DMA
  uint16_t pDuty = (uint16_t)map(buffer[0],0,4095,0,PWM_OVERFLOW/2-1);
  pwmWrite(PWM_OUT,PWM_OVERFLOW/2+pDuty);
}
