#include "zf_common_headfile.h"
#include "mpu6050_dmp.h"


#define SDA             gpio_get_level (SEEKFREE_SDA)
#define SDA0()          gpio_set_level (SEEKFREE_SDA, 0)		//IO口输出低电平
#define SDA1()          gpio_set_level (SEEKFREE_SDA, 1)		//IO口输出高电平  
#define SCL0()          gpio_set_level (SEEKFREE_SCL, 0)		//IO口输出低电平
#define SCL1()          gpio_set_level (SEEKFREE_SCL, 1)		//IO口输出高电平
#define DIR_OUT()       gpio_set_dir (SEEKFREE_SDA, GPO, GPO_PUSH_PULL)   //输出方向
#define DIR_IN()        gpio_set_dir (SEEKFREE_SDA, GPI, GPO_OPEN_DTAIN)    //输入方向


#define MPU_ADDR 0x68

 //内部数据定义
uint8 IIC_ad_main; //器件从地址	    
uint8 IIC_ad_sub;  //器件子地址	   
uint8* IIC_buf;    //发送|接收数据缓冲区	    
uint8 IIC_num;     //发送|接收数据个数	     

#define ack 1      //主应答
#define no_ack 0   //从应答	 



uint16 simiic_delay_time = SIMIIC_DELAY_TIME;   //ICM等传感器应设置为20


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时 时间设置
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通讯失败可以尝试增加simiic_delay_time的值
//-------------------------------------------------------------------------------------------------------------------
void simiic_delay_set(uint16 time)
{
    simiic_delay_time = time;           //ICM等传感器应设置为100
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通讯失败可以尝试增加simiic_delay_time的值
//-------------------------------------------------------------------------------------------------------------------
void simiic_delay(void)
{
    uint16 delay_time;
    delay_time = simiic_delay_time;
    while (delay_time--);
}


//内部使用，用户无需调用
void simiic_start(void)
{
    SDA1();
    SCL1();
    simiic_delay();
    SDA0();
    simiic_delay();
    SCL0();
    simiic_delay();
}

//内部使用，用户无需调用
void simiic_stop(void)
{
    SDA0();
    SCL0();
    simiic_delay();
    SCL1();
    simiic_delay();
    SDA1();
    simiic_delay();
}

//主应答(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
void simiic_sendack(unsigned char ack_dat)
{
    SCL0();
    simiic_delay();
    if (ack_dat) SDA0();
    else    	SDA1();

    SCL1();
    simiic_delay();
    SCL0();
    simiic_delay();
}


static int sccb_waitack(void)
{
    SCL0();
    DIR_IN();
    simiic_delay();

    SCL1();
    simiic_delay();

    if (SDA)           //应答为高电平，异常，通信失败
    {
        DIR_OUT();
        SCL0();
        return 1;
    }
    DIR_OUT();
    SCL0();
    simiic_delay();
    return 0;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
void send_ch(uint8 c)
{
    uint8 i = 8;
    while (i--)
    {
        if (c & 0x80)	SDA1();//SDA 输出数据
        else			SDA0();
        c <<= 1;
        simiic_delay();
        SCL1();                //SCL 拉高，采集信号
        simiic_delay();
        SCL0();                //SCL 时钟线拉低
    }
    //sccb_waitack();
}

//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|使用
//内部使用，用户无需调用
uint8 read_ch(uint8 ack_x)
{
    uint8 i;
    uint8 c;
    c = 0;
    SCL0();
    simiic_delay();
    SDA1();
    DIR_IN();           //置数据线为输入方式
    for (i = 0;i < 8;i++)
    {
        simiic_delay();
        SCL0();         //置时钟线为低，准备接收数据位
        simiic_delay();
        SCL1();         //置时钟线为高，使数据线上数据有效
        simiic_delay();
        c <<= 1;
        if (SDA)
        {
            c += 1;   //读数据位，将接收的数据存c
        }
    }
    DIR_OUT();
    SCL0();
    simiic_delay();
    simiic_sendack(ack_x);

    return c;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC写数据到设备寄存器函数
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat				写入的数据
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
    simiic_start();
    send_ch((dev_add << 1) | 0x00);   //发送器件地址加写位
    send_ch(reg);   				 //发送从机寄存器地址
    send_ch(dat);   				 //发送需要写入的数据
    simiic_stop();
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC从设备寄存器读取数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8 simiic_read_reg(uint8 dev_add, uint8 reg, IIC_type type)
{
    uint8 dat = 0;
    simiic_start();
    send_ch((dev_add << 1) | 0x00);  //发送器件地址加写位
    send_ch(reg);   				//发送从机寄存器地址
    if (type == SCCB)simiic_stop();

    simiic_start();
    send_ch((dev_add << 1) | 0x01);  //发送器件地址加读位
    dat = read_ch(ack);   				//读取数据
    simiic_stop();

    return dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC读取多字节数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat_add			数据保存的地址指针
//  @param      num				读取字节数量
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_read_regs(uint8 dev_add, uint8 reg, uint8* dat_add, uint8 num, IIC_type type)
{
    simiic_start();
    send_ch((dev_add << 1) | 0x00);  //发送器件地址加写位
    send_ch(reg);   				//发送从机寄存器地址
    if (type == SCCB)simiic_stop();

    simiic_start();
    send_ch((dev_add << 1) | 0x01);  //发送器件地址加读位
    while (num--)
    {
        *dat_add = read_ch(no_ack); //读取数据
        dat_add++;
    }
    //*dat_add = read_ch(no_ack); //读取数据
    simiic_stop();
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC端口初始化
//  @param      NULL
//  @return     void	
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void simiic_init(void)
{
    gpio_init(SEEKFREE_SCL, GPO, 1, GPO_PUSH_PULL);
    gpio_init(SEEKFREE_SDA, GPO, 1, GPO_OPEN_DTAIN);
}
//IIC连续写
//addr:器件地址
//reg: 寄存器地址
//len: 写入长度
//buf: 数据区
//返回值: 0,正常
//              其他，错误代码
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    uint8_t i;
    simiic_start();
    send_ch((addr << 1) | 0);//发送器件地址+写命令
    if (sccb_waitack())//等待应答
    {
        simiic_stop();
        return 1;
    }
    send_ch(reg);//写寄存器地址
    sccb_waitack();//等待应答
    for (i = 0;i < len;i++)
    {
        send_ch(buf[i]);//发送数据
        if (sccb_waitack())//等待ACK
        {
            simiic_stop();
            return 1;
        }
    }
    simiic_stop();
    return 0;
}
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取得长度
//buf:读取到的数据存储区
//返回值: 0,正常
//              其他，错误代码
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    simiic_start();
    send_ch((addr << 1) | 0);//发送器件地址+写命令
    if (sccb_waitack())//等待应答
    {
        simiic_stop();
        return 1;
    }
    send_ch(reg);//写寄存器地址
    sccb_waitack();//等待应答
    simiic_start();
    send_ch((addr << 1) | 1);//发送器件地址+读命令
    sccb_waitack();//等待应答
    while (len)
    {
        if (len == 1) *buf = read_ch(0);//读数据，发送nACK
        else *buf = read_ch(1);//读数据，发送ACK
        len--;
        buf++;
    }
    simiic_stop();//产生一个停止条件
    return 0;
}

//IIC写一个字节
//reg:      寄存器地址
//data:     数据
//返回值:  0,正常
//          其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    simiic_start();
    send_ch((MPU_ADDR << 1) | 0);//发送器件地址+写命令
    if (sccb_waitack())  //等待应答
    {
        simiic_stop();
        return 1;
    }
    send_ch(reg); //写寄存器地址
    sccb_waitack();     //等待应答
    send_ch(data);//发送数据
    if (sccb_waitack())  //等待ACK
    {
        simiic_stop();
        return 1;
    }
    simiic_stop();
    return 0;
}

//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    simiic_start();
    send_ch((MPU_ADDR << 1) | 0);//发送器件地址+写命令
    sccb_waitack();//等待应答
    send_ch(reg);//写寄存器地址
    sccb_waitack();//等待应答
    simiic_start();
    send_ch((MPU_ADDR << 1) | 1);//发送期间地址+读命令
    sccb_waitack();//等待应答
    res = read_ch(0);//读取数据，发送nACK
    simiic_stop();//产生一个停止条件
    return res;
}
