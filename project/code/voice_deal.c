#include "zf_common_headfile.h"
#include "voice_deal.h"

uint16 sample_point = Point_Size;
uint16 data1[Point_Size];
uint16 data2[Point_Size];

int16 Micdata_cov(unsigned short* data1, unsigned short* data2, unsigned short point, short point1, short point2, unsigned short step) {
    int16 cnt = 0;
    uint32 data1val = 0;
    uint32 data2val = 0;
    uint32 data3val = 0;
    int t = 0;
    int16 pos = 0;
    int16 maxpos = 0;
    uint32 maxval = 0;
    for (cnt = point1; cnt < point2; cnt++) {
        for (t = 0; t < point; t += step) {
            pos = t - cnt;
            if (pos < 0) {
                pos += sample_point;
            }
            else if (pos >= sample_point) {
                pos -= sample_point;
            }
            data3val = data1[t] * data2[pos]; //互相关
            data1val = data1val + data3val;
          }

        data2val = data1val >> 12; 
        if (maxval <= data2val) {
            maxpos = cnt;
            maxval = data2val;
        }
        data1val = 0;
    }
    return maxpos;//返回值即为时延
}
