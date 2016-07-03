#include "hal.h"
#include "drv_pwm.h"
#include "config.h"

#define PULSE_1MS       (1000) // 1ms pulse width

#define MWII_PWM_MAX  1000
#define MWII_PWM_PRE    1

// returns whether driver is asking to calibrate throttle or not
bool pwmInit(drv_pwm_config_t *init)
{
    CLK_EnableModuleClock(PWM01_MODULE);
    CLK_EnableModuleClock(PWM45_MODULE);

    // PWM clock source
    CLK->CLKSEL1 &= ~(CLK_CLKSEL2_PWM45_S_Msk | CLK_CLKSEL1_PWM01_S_Msk);
    CLK->CLKSEL1 |= CLK_CLKSEL2_PWM45_S_HCLK | CLK_CLKSEL1_PWM01_S_HCLK;
    
    // Multifuncional pin set up PWM0-4
		
		//P2.2/3/6
		SYS->P2_MFP &= ~(SYS_MFP_P22_Msk | SYS_MFP_P23_Msk | SYS_MFP_P26_Msk);
		SYS->P2_MFP |= SYS_MFP_P22_PWM0 | SYS_MFP_P23_PWM1 | SYS_MFP_P26_PWM4;
		//P0.4 Pin
		SYS->P0_MFP &= ~SYS_MFP_P04_Msk;
		SYS->P0_MFP |= SYS_MFP_P04_PWM5;
		

#define MWII_PWM_MASK ((1 << 0) | (1 << 1) | (1 << 4) | (1 << 5))

    // Even channel N and N+1 share prescaler
    PWM_SET_PRESCALER(PWM, 0, MWII_PWM_PRE);
    PWM_SET_PRESCALER(PWM, 4, MWII_PWM_PRE);
    PWM_SET_DIVIDER(PWM, 0, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWM, 1, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWM, 4, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWM, 5, PWM_CLK_DIV_1);

    PWM_Start(PWM, MWII_PWM_MASK);
    // No analog of PWM_Start for enabling auto-reload mode
    PWM->PCR |= PWM_PCR_CH0MOD_Msk << (4 * 0);
    PWM->PCR |= PWM_PCR_CH0MOD_Msk << (4 * 1);
    PWM->PCR |= PWM_PCR_CH0MOD_Msk << (4 * 4);
    PWM->PCR |= PWM_PCR_CH0MOD_Msk << (4 * 5);
    
    // Duty
    PWM_SET_CMR(PWM, 0, 0);
    PWM_SET_CMR(PWM, 1, 0);
    PWM_SET_CMR(PWM, 4, 0);
    PWM_SET_CMR(PWM, 5, 0);
    // Period, actually sets it to safe value 1000+1 
    PWM_SET_CNR(PWM, 0, MWII_PWM_MAX);
    PWM_SET_CNR(PWM, 1, MWII_PWM_MAX);
    PWM_SET_CNR(PWM, 4, MWII_PWM_MAX);
    PWM_SET_CNR(PWM, 5, MWII_PWM_MAX);
	
    PWM_EnableOutput(PWM, MWII_PWM_MASK);

    return false;
}
void pwmWriteMotor(uint8_t index, uint16_t value)
{
    // Motor 0 BACK_R  - PWM0
    // Motor 1 FRONT_R - PWM1
    // Motor 2 BACK_L  - PWM4
    // Motor 3 FRONT_L - PWM5
    static uint8_t motor_to_pwm[] = { 0, 1, 4, 5 };

    if (index > 3) return;
    PWM_SET_CMR(PWM, motor_to_pwm[index], value-1000);
}

// Not implmented
//void pwmWriteServo(uint8_t index, uint16_t value)
//{
//}
//uint16_t pwmRead(uint8_t channel)
//{
//    return 0;
//}
