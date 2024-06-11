#include "rtthread.h"
#include "zf_common_headfile.h"

#define PWM_CH1             (TCPWM_CH30_P10_2)                                 // PWMÊä³ö¶Ë¿Ú

extern int16_t duty_out;
extern bool flag_stop;

void brshlessmotor_thread_entry(void* paramter)
{
    while (1)
    {
        pwm_set_duty(PWM_CH1, duty_out);
        if (flag_stop)
        {
            if (duty_out > 200)
            {
                duty_out -= duty_out * 0.15;
            }
            else
            {
                duty_out = 0;
            }
        }
        rt_thread_mdelay(20);
    }
}