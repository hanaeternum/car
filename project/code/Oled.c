#include "GPS.h"
#include "math.h"
#include "stdio.h"
#include "rtthread.h"
#include "zf_common_headfile.h"
#include "inv_mpu.h" 

extern int16_t duty_out;
extern bool flag_stop;
extern gnss_info_struct gps_data; 
extern bool gps_flag;
extern double distance;
extern double gps_angle;
extern float z1;
extern double voice_angle;
extern uint8 gps_number_uart;
extern float voice_x_out;
extern float voice_y_out;
extern uint8 gps_number_uart;
extern struct gps GPS_Point[9];

char text[50];

void oled_thread_entry(void* paramter)
{
    while (1)
    {
        sprintf(text, "Stop:%d", flag_stop);
        oled_show_string(8, 0, text);
        sprintf(text, "PWM:%4d", duty_out);
        oled_show_string(60, 0, text);
        sprintf(text, "Agl:%.4f", z1);
        oled_show_string(8, 1, text);
//        sprintf(text, "X:%.4f", voice_x_out);
//        oled_show_string(8, 2, text);
//        sprintf(text, "Y:%.4f", voice_y_out);
//        oled_show_string(8, 3, text);
        sprintf(text, "N:%d", gps_number_uart);
        oled_show_string(8, 4, text);
        sprintf(text, "TIMER:%6d", timer_get(TC_TIME2_CH0));
        oled_show_string(8, 5, text);
        sprintf(text, "GPS:%d*%.4f", gps_flag, distance*100);
        oled_show_string(8, 2, text);
        sprintf(text, "SER:%.4f", voice_angle);
        oled_show_string(8, 3, text);
//        sprintf(text, "SER:%d", gps_number_uart);
//        oled_show_string(8, 4, text);
        
        rt_thread_mdelay(20);
    }
}