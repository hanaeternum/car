#include "key.h"
#include "zf_common_headfile.h"

struct key KEY[2];

void KEY_Init()
{
    gpio_init(P06_0, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(P06_1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(P19_0, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void KEY_Scanf()
{
    KEY[0].sta = gpio_get_level(P06_0);
    KEY[1].sta = gpio_get_level(P06_1);
    for (uint8_t i = 0; i < 2; i++)
    {
        switch (KEY[i].judge)
        {
            case 0:
                if (!KEY[i].sta)
                {
                    KEY[i].judge = 1;
                }
                break;
            case 1:
                if (!KEY[i].sta)
                {
                    KEY[i].judge = 2;
                }
                break;
            case 2:
                if (KEY[i].sta)
                {
                    KEY[i].flag = 1;
                    KEY[i].judge = 0;
                }
                break;
        }
    }
}