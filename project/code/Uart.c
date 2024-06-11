#include "rtthread.h"
#include "zf_common_headfile.h"

uint8_t uart_text[80];

extern double gps_angle;
extern double PID_Out_Angele;
extern double distance;
extern gnss_info_struct gps_data; 
extern double Servomotor_ek;

void uart_thread_entry(void* paramter)
{
    while (1)
    {
        sprintf((char *)uart_text, "%.4fAAA%.4fAAA%.4ffPID%.4fDIS%.4fEK%.4f\r\n", gps_data.longitude, gps_data.latitude, gps_angle, PID_Out_Angele, distance*100.0000, Servomotor_ek);
        uart_write_buffer(UART_1, uart_text, sizeof(uart_text));
        rt_thread_mdelay(500);
    }
}