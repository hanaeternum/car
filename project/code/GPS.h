#ifndef _GPS_H
#define _GPS_H

#include "zf_common_headfile.h"

struct gps
{
  double x;
  double y;
};

void gps_thread_entry(void* paramter);

#endif