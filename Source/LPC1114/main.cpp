#include "mbed.h"
#include "PIDController.h"
#pragma O3
#pragma Otime

DigitalOut led1(LED1);
PIDController pid(1.0, 0.1, 0.0);
static int ad_result;

void initPWM()
{
    // PWM configuration
    LPC_SYSCON->SYSAHBCLKCTRL |=  (1<<8);             // Enable Clock for TMR1
    LPC_IOCON->PIO1_9 |=  (1<<0);                     // PIN1_9 = CT16B1_MAT0
    LPC_TMR16B1->MR0 = 127;                             // 0% Duty Cycle
    LPC_TMR16B1->MR3 = 256;                           // Cycle Length (PWM frequency is SYSCLK/MR3 Hz)
    LPC_TMR16B1->MCR |= (1<<10);                      // TC Reset on MR3 Match
    LPC_TMR16B1->PWMC |= (1<<0);                      // PWM Mode
    LPC_GPIO1->DIR |= (1<<9);                         // Output
    LPC_TMR16B1->TCR |= (1<<0);                       // GO
}

void setPWM(int pulth_width)
{
    LPC_TMR16B1->MR0 = pulth_width;                     // change pulth width
}

extern "C" void ADC0_IRQHandler()
{
    int adc_stat = LPC_ADC->STAT; // Get ADC status, clear interrupt flag
    //led1 = !led1;  //Debug
    ad_result   = ((LPC_ADC->DR[5] & 0x0000FFC0) >> 6);   //get conversion result (10 bits)
    //printf("result 5:%x\n",ad_result);    //Debug
    pid.setProcessValueFixedPoint(ad_result<<SFT);
    int pwm_width = pid.calculate();
    setPWM(255 - pwm_width);
    LPC_ADC->CR |= (1 << 24);                  // go conversion
}

void initADC()
{
    // ADC configuration
    LPC_SYSCON->SYSAHBCLKCTRL |=  (1<<13);             // Enable Clock for ADC
    LPC_SYSCON->PDRUNCFG  &= ~(1<<4);                  // Power the ADC
    LPC_IOCON->PIO1_4 &= ~(0x9F);                      // Clear FUNC field for pin 13, set to analog input mode, disabled Pull-up/Pull-down
    LPC_IOCON->PIO1_4 |= (1<<0);                       // set to ADC for pin 13
    LPC_ADC->CR &= ~0xFFFF;                              // clear ADCR reg(default no channels enabled)
    LPC_ADC->CR |=  (1<<5);                            // select ADC chanel AD5 (pin 13)
    LPC_ADC->CR |= 0x0b00 ;                         // set up ADC clock

    // ADC interrupt configuration
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_SetVector(ADC_IRQn, (uint32_t)&ADC0_IRQHandler);
    LPC_ADC->INTEN = 0x1FF; /* Enable all interrupts */

    // go ADC
    //LPC_ADC->CR |= (1 << 24);
}

int main()
{
    //unsigned char pulth_width = 0;
    pid.setTargetValue(388);
    pid.setBias(388);
    initPWM();
    initADC();
    printf("configuration finished!\n");

    // go ADC
    LPC_ADC->CR |= (1 << 24);
    while(1) {
        led1 = 1;
        //setPWM(++pulth_width); //Debug
        wait(0.2);
        led1 = 0;
        //setPWM(++pulth_width); //Debug
        wait(0.2);
    }
}
