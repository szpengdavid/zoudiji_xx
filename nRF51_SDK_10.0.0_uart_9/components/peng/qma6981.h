/* gsensor sc7a20 driver head file
 * author: peter Lee
 * date: 2017-5-10
 */

#ifndef QMA6981_H
#define QMA6981_H

#include <stdbool.h>
#include <stdint.h>



//------peng------


#define QMA6981_SCL   				3
#define QMA6981_SDA   				2
#define QMA6981_IIC_ADDR 			0x12
#define IOI2C_1_BYTE_ADDRESS	0x01
#define IOI2C_2_BYTE_ADDRESS	0x00

#define STEP_ERROR	0
#define STEP_NOW		1		 
#define STEP_QUIT		2	

#define Must_step_length 	        16	    //原先12
#define Step_SENSITIVITY          0x0a    //0x13寄存器的值, 原先0x19    上一个次 0x10          要求低:0x13   要求高:0x0a
#define HIGH_G_SENSITIVITY        0x17    //0x26寄存器的值,白天 加判断0x0c|0x03    0x14:100%   要求低:0x1f   要求高:0x17->0x14
#define HIGH_G_SENSITIVITY_NIGHT  0x17    //0x26寄存器的值,睡眠开启后，去除 0x0c|0x03   0x14:100%
#define READ_STEP_CYCLE           16384   //16384:0.5s   32768:1s 
#define Sleep_Awake_Filter_Step   36      //改为26试一试,配合0x13寄存器改为0x10，原先18

typedef enum {
	iic_write,
	iic_read,
}IIC_DIRECTION;

void GPIO_Pin_Clear(uint32_t pin);
void GPIO_Pin_Set(uint32_t pin);
void GPIO_Set_Input(uint32_t pin);
void GPIO_Set_Output(uint32_t pin);
/*
设置iic管脚函数
gpio15->SDA  gpio16->SCL 
*/
void ioi2c_ioconfig(uint32_t iic_sclk,uint32_t iic_sda);
/*
IIC停止函数 产生停止位
*/
void ioi2c_stop(uint32_t iic_sclk,uint32_t iic_sda);

/*
IIC接收一个字节的数据函数
输入变量：uint8_t mack   0：主机不给从机回应 1：主机给从机回应 （只是在多字节读的时候用到）
返回值：uint8_t接收到的数据（一个字节）
*/
uint8_t iic_receivedata(uint32_t iic_sclk,uint32_t iic_sda,uint8_t mack);

/*
IIC发送设备ID(设备地址)字节函数
输入参数：uint8_t id 设备的地址 这里是七位地址 最终输出的地址将会左移一位并且加1，如0x50->0xb0
					IIC_DIRECTION direction 传输方向 参考IIC_DIRECTION枚举，该变量决定了第一个直接的最低位
*/
void iic_sendid(uint32_t iic_sclk,uint32_t iic_sda,uint8_t id,IIC_DIRECTION direction);
/*
IIC发送一个字节的数据函数
输入参数：uint8_t da 要发送的数据（一个字节）
*/
void iic_senddata(uint32_t iic_sclk,uint32_t iic_sda,uint8_t da);

/*
IIC开始函数 时序开始 产生停止位
*/
void ioi2c_start(uint32_t iic_sclk,uint32_t iic_sda);

/*
iic0的写函数
传入参数：uint8_t id 设备的地址 这里是七位地址 最终输出的地址将会左移一位，如0x50->0xa0
					uint8_t addr_len 设备内部地址（寄存器地址）的长度，如果传入I2C_2_BYTE_ADDRESS则为2个字节个地址，否则为1个字节
					uint8_t * buf 读取数据的储存区域
					uint16_t sz 要读取的数据长度
*/
uint8_t i2c_reg_read(uint32_t iic_sclk,uint32_t iic_sda,uint8_t id, uint8_t addr_len, uint16_t addr, uint8_t * buf, uint16_t sz);

/******************************************
	qma6981_read_bytes
*******************************************/ 
uint32_t qma6981_read_bytes(uint8_t reg, uint8_t *data, uint16_t count);



bool QQMA6981_ConReadBytes(uint8_t* Data, uint8_t RegAddr, uint8_t Length);



uint16_t QQMA6981_read_setpcounter(void);


/*
iic0的写函数
传入参数：uint8_t id 设备的地址 这里是七位地址 最终输出的地址将会左移一位并且加1，如0x50->0xb0
					uint8_t addr_len 设备内部地址（寄存器地址）的长度，如果传入I2C_2_BYTE_ADDRESS则为2个字节个地址，否则为1个字节
					uint8_t * buf 要发的数据内容
					uint16_t sz 要发送的数据长度
*/
uint8_t i2c_reg_write(uint32_t iic_sclk,uint32_t iic_sda,uint8_t id, uint8_t addr_len, uint16_t addr, uint8_t * buf, uint16_t sz);

/******************************************
	QQMA6981 write bytes
*******************************************/ 
bool QQMA6981_WriteBytes(uint8_t RegAddr, uint8_t Data);


void QQMA6981_setp_conuter_enable(void);

void  QQMA6981_stepCounter_On(void);




















//======peng======

#endif  /* COMBO_QMA6981_H__ */
	

