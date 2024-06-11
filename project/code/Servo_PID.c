#include "math.h"
#include "Servo_PID.h"
#include "rtthread.h"
#include "Brushlessmotor.h"
#include "zf_common_headfile.h"

#define PWM_SERVO_CH            (TCPWM_CH37_P21_5)

float GPS_Servomotor_PID_Voice_kp = 0.95;//0.95
float GPS_Servomotor_PID_Voice_kd = 0.1;//0.1
double Servomotor_Voice_ek,Servomotor_Voice_ek1;  
float GPS_Servomotor_PID_GPS_kp = 0.65;
float GPS_Servomotor_PID_GPS_kd = 0.15;
double Servomotor_GPS_ek,Servomotor_GPS_ek1;  
double set_angle;  
double PID_Out_Angele;
double servo_duty = 0;
bool flag_gps_voice ;
uint16 voice_speed;

extern double gps_angle;
extern float x1;
extern float y1;
extern float z1;
extern double voice_angle;
extern gnss_info_struct gps_data;  

/*****½Ç¶È×ª»»º¯Êý*****
*
*´Ó×óÂú¶æËã0¡ã£¬µ½ÓÒÂú¶æ56¡ã
*
*Õ¼¿Õ±È    ½Ç¶È
*  4%    ×óÆ«30¡ã
*  8%    ÓÒÆ«26¡ã
*********************/
double Angle_Con(double d_angle)
{
    d_angle += 30;

    if (d_angle > 58)        d_angle = 58;
    else if (d_angle < 6)  d_angle = 6;

    return (400 + 7.14f * d_angle);
}

void servo_pid_thread_entry(void* paramter)
{
    while (1)
    {
        if (!flag_gps_voice)
        {
            Servomotor_GPS_ek1 = Servomotor_GPS_ek;
            Servomotor_GPS_ek =  - gps_angle + z1;
            if (Servomotor_GPS_ek < -180)
                {
                   Servomotor_GPS_ek+= 360;
                }
                if (Servomotor_GPS_ek > 180)
                {
                  Servomotor_GPS_ek-= 360;
                }
            PID_Out_Angele = GPS_Servomotor_PID_GPS_kp * Servomotor_GPS_ek + GPS_Servomotor_PID_GPS_kd * (Servomotor_GPS_ek - Servomotor_GPS_ek1);
        }
        else
        {
            Servomotor_Voice_ek1 = Servomotor_Voice_ek;
            Servomotor_Voice_ek =  voice_angle;
            if (Servomotor_Voice_ek < -180)
            {
                Servomotor_Voice_ek+= 360;
            }
            if (Servomotor_Voice_ek > 180)
            {
                Servomotor_Voice_ek-= 360;
            }
            if (timer_get(TC_TIME2_CH0) > 2000)
            {
                voice_speed -= 100; 
                if (voice_speed < 800)
                {
                    voice_speed = 800;
                }
            }
            PID_Out_Angele = GPS_Servomotor_PID_Voice_kp * Servomotor_Voice_ek + GPS_Servomotor_PID_Voice_kd * (Servomotor_Voice_ek - Servomotor_Voice_ek1);
        }
        servo_duty = Angle_Con(PID_Out_Angele);
        pwm_set_duty(PWM_SERVO_CH, (uint32)servo_duty);
        rt_thread_mdelay(20);
    }
}
