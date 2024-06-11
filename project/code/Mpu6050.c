#include "rtthread.h"
#include "inv_mpu.h"
#include "zf_common_headfile.h"

float x1;
float y1;
float z1;

void mpu6050_thread_entry(void* paramter)
{
    while (1)
    {
        mpu_dmp_get_data(&x1,&y1,&z1);
        if (z1 < 0)
        {
          z1+= 360;
        }
        if (z1 > 360)
        {
          z1-= 360;
        }
        rt_thread_mdelay(10);
    }
}