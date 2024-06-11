#include "zf_common_headfile.h"

#define Filter_N 5

#define window_size 5            //滑动窗口长度
float buffer[window_size] = {0}; //滑动窗口数据buf

float sliding_average_filter(float value)
{
    static int data_num = 0;
    float output = 0;

    if (data_num < window_size) //不满窗口，先填充
    {
      buffer[data_num++] = value;
      output = value; //返回相同的值
    }
    else
    {
      int i = 0;
      float sum = 0;

      memcpy(&buffer[0], &buffer[1], (window_size - 1) * 4); //将1之后的数据移到0之后，即移除头部
      buffer[window_size - 1] = value;                       //即添加尾部

      for (i = 0; i < window_size; i++) //每一次都计算，可以避免累计浮点计算误差
        sum += buffer[i];

      output = sum / window_size;
    }

  return output;
}
