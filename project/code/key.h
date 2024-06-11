#ifndef _HAN_KEY_H
#define _HAN_KEY_H

#include "zf_common_headfile.h"

struct key
{
	bool sta;
	uint8_t judge;
	bool flag;
};

void KEY_Init();
void KEY_Scanf();

#endif