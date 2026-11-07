#include "tim_pwm.h"
#include "cmsis_os.h"
#include <stdio.h>

/* Software PWM on PD12 (channel 0), PC6 (channel 1), PC7 (channel 2)
   Uses DWT cycle counter for microsecond delays for better resolution. */

static volatile uint8_t duty[3] = {25, 50, 75}; /* default duties: ch0=25%, ch1=50%, ch2=75% */
static const uint32_t pwm_freq_hz = 1000; /* 1kHz default */
static const uint32_t period_us = 1000000U / 1000U; /* 1000 us */

/* DWT based microsecond delay */
static void DWT_Delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000UL) * us;
    /* Ensure DWT enabled */
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

void PWM_Init(void)
{
    /* Nothing else: GPIOs are configured in MX_GPIO_Init (gpio.c) */
}

/* PWM task */
static void PWM_Task(void *argument)
{
    (void) argument;
    const TickType_t xDelay = pdMS_TO_TICKS(1);

    while (1) {
        uint32_t on_us0 = (period_us * duty[0]) / 100U;
        uint32_t on_us1 = (period_us * duty[1]) / 100U;
        uint32_t on_us2 = (period_us * duty[2]) / 100U;

        uint32_t t0 = 0;

        /* Start of period: set pins according to duty>0 */
        if (duty[0]) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); else HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        if (duty[1]) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        if (duty[2]) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

        /* handle channel 0 */
        if (on_us0 && on_us0 < period_us) {
            DWT_Delay_us(on_us0);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        }

        /* compute remaining time after ch0 */
        /* For simplicity: chain delays for ch1/ch2 - worst-case jitter small at 1kHz */
        if (on_us1 && on_us1 < period_us) {
            uint32_t delay = (on_us1 > on_us0) ? (on_us1 - on_us0) : 0;
            if (delay) DWT_Delay_us(delay);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        }

        if (on_us2 && on_us2 < period_us) {
            uint32_t max_on = (on_us0>on_us1)?on_us0:on_us1;
            uint32_t delay = (on_us2 > max_on) ? (on_us2 - max_on) : 0;
            if (delay) DWT_Delay_us(delay);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        }

        /* finish remaining of period */
        uint32_t elapsed_us = 0; /* we don't precisely track, so sleep small */
        /* Sleep to yield CPU; DWT busy-wait covered most of period for accuracy */
        osDelay(1);
    }
}

void PWM_StartTask(void)
{
    osThreadAttr_t attr = {0};
    attr.name = "PWMTask";
    attr.priority = osPriorityHigh;
    attr.stack_size = 256;

    osThreadNew(PWM_Task, NULL, &attr);
}

void PWM_SetDuty(uint8_t channel, uint8_t duty_percent)
{
    if (channel > 2) return;
    if (duty_percent > 100) duty_percent = 100;
    duty[channel] = duty_percent;
}
