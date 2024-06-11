#include "zf_common_headfile.h"
#include "zf_device_gnss.h"
#include "stdio.h"
#include "string.h"
#include "GPS.h"
#include "key.h"
#include "Oled.h"
#include "Uart.h"
#include "Mpu6050.h"
#include "rtthread.h"
#include "Brushlessmotor.h"
#include "Servo_PID.h"
#include "Voice_Search.h"
#include "inv_mpu.h"

#define DIR_CH1             (P10_3)                                            // 电机方向输出端口
#define PWM_CH1             (TCPWM_CH30_P10_2)                                 // PWM输出端口
#define ENCODER1_TIM        (TC_CH58_ENCODER)                                  // 编码器定时器
#define ENCODER1_PLUS       (TC_CH58_ENCODER_CH1_P17_3)                        // 编码器计数端口
#define ENCODER1_DIR        (TC_CH58_ENCODER_CH2_P17_4)                        // 编码器方向采值端口

#define UART_DOUBLE             (UART_1)
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)
#define UART_DOUBLE_TX_PIN      (UART1_TX_P04_1)
#define UART_DOUBLE_RX_PIN      (UART1_RX_P04_0)

#define UART_GPS             (UART_2)
#define UART_GPS_TX_PIN      (UART2_TX_P10_1)
#define UART_GPS_RX_PIN      (UART2_RX_P10_0)

/*       舵机      */
#define PWM_SERVO_CH            (TCPWM_CH37_P21_5)

int16_t duty_out = 50;                                                       //PWM输出(duty_out/100即占空比)
int16 encoder[2] = {0};                                                        // 编码器值
//串口GPS数据处理
uint8 uart_get_data[150];
uint8_t number;
uint8  get_gps_data = 0; 
bool gps_flag;

struct gps GPS_Point[9];
uint8_t gps_number;

int flag;

extern struct key KEY[2];

//串口双机数据处理
uint8  get_double_data = 0;
uint8_t double_number;
uint8 double_uart_data[10];

extern uint8 gps_number_uart;

////GPS数据处理
gnss_info_struct gps_data;  

bool flag_stop = 0;

bool oled_show = 0;

uint8 flag_timer;
   
int main(void)
{ 
    clock_init(SYSTEM_CLOCK_250M);
    debug_info_init(); 
    
    KEY_Init();
    
    pwm_init(PWM_CH1, 1000, 0);                                                 // PWM 通道1 初始化频率1KHz 占空比初始为0
    gpio_init(DIR_CH1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                          // 初始化电机方向输出引脚
    encoder_dir_init(ENCODER1_TIM, ENCODER1_PLUS, ENCODER1_DIR);                // 初始化编码器采值引脚及定时器
    
    /*          舵机PWM初始化        */
    pwm_init(PWM_SERVO_CH, 50, 614);
    gpio_set_level(DIR_CH1, 0);
   
    /*       硅麦初始化       */
    adc_init(ADC1_CH13_P13_1, ADC_12BIT);
    adc_init(ADC1_CH14_P13_2, ADC_12BIT);
    adc_init(ADC1_CH15_P13_3, ADC_12BIT);
    adc_init(ADC1_CH16_P13_4, ADC_12BIT);
   
    uart_init(UART_DOUBLE, UART_BAUDRATE, UART_DOUBLE_TX_PIN, UART_DOUBLE_RX_PIN);
    uart_rx_interrupt(UART_DOUBLE, 1); 
    uart_init(UART_GPS, UART_BAUDRATE, UART_GPS_TX_PIN, UART_GPS_RX_PIN);
    uart_rx_interrupt(UART_GPS, 1); 
    
    flag = mpu_dmp_init();
    while(flag)
    {
        flag = mpu_dmp_init();
    }
    pit_us_init(PIT_CH0, 100);
    pit_us_init(PIT_CH1, 100);
    pit_us_init(PIT_CH2, 5);
    pit_ms_init(PIT_CH10, 10);
    
    oled_init();
    oled_clear();
    
    flash_init();
    while(gps_number < 8)
    {
      char text[50];
      sprintf(text, "%.4f:::%.4f", gps_data.latitude, gps_data.longitude);
      oled_show_string(0, gps_number, text);
      if (KEY[1].flag)
      {
        KEY[1].flag = 0;
        GPS_Point[gps_number].x = gps_data.latitude;
        GPS_Point[gps_number].y = gps_data.longitude;
        
        flash_buffer_clear();
        flash_union_buffer[0].uint32_type = (uint32)GPS_Point[gps_number].x; 
        flash_union_buffer[1].uint32_type  = (uint32)((GPS_Point[gps_number].x - (uint16)GPS_Point[gps_number].x) * 100000);
        flash_union_buffer[2].uint32_type = (uint32)GPS_Point[gps_number].y; 
        flash_union_buffer[3].uint32_type  = (uint32)((GPS_Point[gps_number].y - (uint16)GPS_Point[gps_number].y) * 100000);
        flash_write_page_from_buffer(gps_number, gps_number, 4);
        
        sprintf(text, "%9.4f::%9.4f", GPS_Point[gps_number].x, GPS_Point[gps_number].y);
        oled_show_string(0, gps_number, text);
        gps_number++;
      }
      if (KEY[0].flag)
      {
        KEY[0].flag = 0;
        oled_clear();
        for (uint8_t i = 0; i < 8; i++)
        {
          flash_buffer_clear();
          flash_read_page_to_buffer(i, i, 4); 
          GPS_Point[i].x = flash_union_buffer[0].uint32_type + (double)(flash_union_buffer[1].uint32_type/100000.0000);
          GPS_Point[i].y = flash_union_buffer[2].uint32_type + (double)(flash_union_buffer[3].uint32_type/100000.0000);
          sprintf(text, "%9.4f::%9.4f", GPS_Point[i].x, GPS_Point[i].y);
          oled_show_string(0, i, text);
        }
        break;
      }
    }
    
    rt_thread_t brshlessmotor_thread;
    brshlessmotor_thread = rt_thread_create("brshlessmotorthread", brshlessmotor_thread_entry, RT_NULL, 256, 29, 20);
    if (brshlessmotor_thread != RT_NULL)
    {
      rt_thread_startup(brshlessmotor_thread);
    }
    
    rt_thread_t oled_thread;
    oled_thread = rt_thread_create("oledthread", oled_thread_entry, RT_NULL, 1280, 29, 20);
    if (oled_thread != RT_NULL)
    {
      rt_thread_startup(oled_thread);
    }
    
    rt_thread_t mpu6050_thread;
    mpu6050_thread = rt_thread_create("mpu6050thread", mpu6050_thread_entry, RT_NULL, 518, 29, 20);
    if (mpu6050_thread != RT_NULL)
    {
      rt_thread_startup(mpu6050_thread);
    }
    
    rt_thread_t gps_thread;
    gps_thread = rt_thread_create("gpsthreadentry", gps_thread_entry, RT_NULL, 512, 29, 20);
    if (gps_thread != RT_NULL)
    {
      rt_thread_startup(gps_thread);
    }
    
    rt_thread_t servo_pid_thread;
    servo_pid_thread = rt_thread_create("servopidthreadentry", servo_pid_thread_entry, RT_NULL, 256, 29, 20);
    if (servo_pid_thread != RT_NULL)
    {
      rt_thread_startup(servo_pid_thread);
    }
    
//    rt_thread_t uart_thread;
//    uart_thread = rt_thread_create("uartthreadentry", uart_thread_entry, RT_NULL, 512, 29, 20);
//    if (uart_thread != RT_NULL)
//    {
//      rt_thread_startup(uart_thread);
//    }
    
    rt_thread_t voice_thread;
    voice_thread = rt_thread_create("voicethreadentry", voice_thread_entry, RT_NULL, 1024, 29, 20);
    if (voice_thread != RT_NULL)
    {
      rt_thread_startup(voice_thread);
    }
    
    while(true)
    {  
      if (flag_stop)
      {
          rt_thread_delete(mpu6050_thread);
          rt_thread_delete(gps_thread);
          rt_thread_delete(servo_pid_thread);
          rt_thread_delete(voice_thread);
      }
      if (KEY[0].flag)
      { 
          if (!oled_show)
          {
              oled_show = 1;
              oled_clear();
              flash_buffer_clear();
              flash_read_page_to_buffer(0, 0, 4); 
              GPS_Point[0].x = flash_union_buffer[0].uint32_type + (double)(flash_union_buffer[1].uint32_type/100000.0000);
              GPS_Point[0].y = flash_union_buffer[2].uint32_type + (double)(flash_union_buffer[3].uint32_type/100000.0000);
              timer_init(TC_TIME2_CH0, TIMER_MS);
              while(duty_out < GPS_DUTY - 200)
              {
                  duty_out += duty_out * 0.15;
                  rt_thread_delay(50);
              }
              duty_out = GPS_DUTY;
          }
          rt_thread_mdelay(100);
      }
    }
}

void uart_double_interrupt_handler (void)
{
    if(uart_query_byte(UART_DOUBLE, &get_double_data))
    {       
        flag_timer = 0;
        if (double_number == 1)
        {
            gps_number_uart= get_double_data;
            double_number = 2;
        }
      
        if (get_double_data == 0x66)
        {
            double_number = 1;
        }
        
        if (get_double_data == 0x79)
        {
            double_number = 0;
        }
    }
}

void uart_gps_interrupt_handler (void)
{   
    if(uart_query_byte(UART_GPS, &get_gps_data))
    {   
        uart_get_data[number] = get_gps_data;
        number++;
        if(uart_get_data[number-1] == '\n' && number < 50)
        {
            number = 0;
            gps_flag = 0;
        }
        else if (uart_get_data[number-1] == '\n' && number >= 50)
        {
            number = 0;
            gps_flag = 1;
            gps_gnrmc_parse((char*)uart_get_data, &gps_data);
        }
    }
}
