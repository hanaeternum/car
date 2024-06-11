#include "GPS.h"
#include "math.h"
#include "stdio.h"
#include "rtthread.h"
#include "Servo_PID.h"
#include "Brushlessmotor.h"
#include "zf_common_headfile.h"

extern gnss_info_struct gps_data;  
extern bool gps_flag;
extern bool flag_gps_voice;
extern bool flag_stop;
extern int16_t duty_out;
extern struct gps GPS_Point[9];
extern uint16 voice_speed;

double gps_angle;
double distance;
uint8 gps_number_uart;

//void gps_thread_entry(void* paramter)
//{
//    while (1)
//    {   
//        if (gps_flag)
//        {
//          z1 = get_two_points_azimuth (gps_data.latitude, gps_data.longitude, GPS_Endpoint_Latituds, GPS_Endpoint_Longitude);
//          gps_angle = gps_data.direction - z1;
////          if (gps_angle < -180)
////            {
////               gps_angle+= 360;
////            }
////            if (gps_angle > 180)
////            {
////              gps_angle-= 360;
////            }
//            distance = get_two_points_distance (gps_data.latitude, gps_data.longitude, GPS_Endpoint_Latituds, GPS_Endpoint_Longitude);
//        }
//        rt_thread_mdelay(20);
//    }
//}

void gps_thread_entry(void* paramter)
{
    while (1)
    {   
        if (gps_flag)
        { 
          distance = get_two_points_distance (gps_data.latitude, gps_data.longitude, GPS_Point[gps_number_uart].x, GPS_Point[gps_number_uart].y);
          if (distance < 2.25)
          {
              flag_gps_voice = 1;
              while(duty_out > voice_speed + 200)
              {
                  duty_out -= duty_out * 0.15;
                  rt_thread_mdelay(15);
              }
              timer_start(TC_TIME2_CH0); 
              duty_out = voice_speed;
          }
          else
          {
              flag_gps_voice = 0;
              while(duty_out < GPS_DUTY - 200)
              {
                  duty_out += duty_out * 0.15;
                  rt_thread_mdelay(15);
              }
              timer_stop(TC_TIME2_CH0);
              timer_clear(TC_TIME2_CH0);
              duty_out = GPS_DUTY;
              voice_speed = VOICE_DUTY;
          }
          gps_angle = get_two_points_azimuth (gps_data.latitude, gps_data.longitude, GPS_Point[gps_number_uart].x, GPS_Point[gps_number_uart].y);
          gps_angle = 360.0000 - gps_angle;
        }
        rt_thread_mdelay(20);
    }
}