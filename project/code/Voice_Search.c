#include "math.h"
#include "stdio.h"
#include "filter.h"
#include "voice_deal.h"
#include "rtthread.h"
#include "zf_common_headfile.h"

extern unsigned short ADC_data1[2050];
extern unsigned short ADC_data2[2050];
extern unsigned short ADC_data3[2050];
extern unsigned short ADC_data4[2050];
extern bool flag_gps_voice;
extern bool flag_ADC;

float voice_x_out;
float voice_y_out;
double voice_angle;
int32_t timer_voice;

void voice_thread_entry(void* paramter)
{
    while (1)
    {
        if (flag_ADC && flag_gps_voice)
        {
            flag_ADC = 0;
            voice_x_out = Micdata_cov(ADC_data2, ADC_data4, 2048, -7, 7, 1);
            voice_y_out = Micdata_cov(ADC_data1, ADC_data3, 2048, -7, 7, 1);
            voice_angle = atan2(voice_y_out, voice_x_out) * (180 / 3.14159);
//            voice_angle = sliding_average_filter(voice_angle);
        }
        rt_thread_mdelay(10);
    }
}