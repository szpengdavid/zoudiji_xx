/* gsensor qma6981 driver source file
 * author: peter Lee
 * date: 2017-5-10
 */
 
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "qma6981.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "gpio.h"



uint8_t QMA6981_Sleep_Read_Fifo_Flag = 0;
uint32_t qma6981_last_step_num = 0;

uint16_t STEP_COUNTER_OLD=0;
uint16_t STEP_COUNTER_FIX=0;  
uint16_t STEP_COUNTER_FIX_OLD=0;

uint16_t  STEP_COUNTER_R=0; 
uint16_t  STEP_STATUS=0;
uint16_t  STEP_STATUS_LAST= STEP_QUIT;

uint16_t STEP_TMP=0;
uint16_t STEP_DIFF=0;


//------peng------

/*
设置iic管脚函数
gpio15->SDA  gpio16->SCL 
*/
void ioi2c_ioconfig(uint32_t iic_sclk,uint32_t iic_sda){
	GPIO_Set_Output(iic_sclk);
	GPIO_Set_Output(iic_sda);
	GPIO_Pin_Set(iic_sclk);   //LCD_CS
	GPIO_Pin_Set(iic_sda);    //LCD_POWER_EN
}
/*
IIC停止函数 产生停止位
*/
void ioi2c_stop(uint32_t iic_sclk,uint32_t iic_sda)
{
	GPIO_Pin_Clear(iic_sclk);
	GPIO_Pin_Clear(iic_sda);
	GPIO_Pin_Set(iic_sclk);
	GPIO_Pin_Set(iic_sda);
}

/*
IIC接收一个字节的数据函数
输入变量：uint8_t mack   0：主机不给从机回应 1：主机给从机回应 （只是在多字节读的时候用到）
返回值：uint8_t接收到的数据（一个字节）
*/
uint8_t iic_receivedata(uint32_t iic_sclk,uint32_t iic_sda,uint8_t mack){
	uint8_t j,da=0;
	GPIO_Set_Input(iic_sda);  //设为输入
	for(j=0;j<8;j++)
	{
		GPIO_Pin_Clear(iic_sclk);
		GPIO_Pin_Set(iic_sclk);
		if(nrf_gpio_pin_read(iic_sda)) da=(da<<1)+1;
		else da=da<<1;
	}
	 //应答
	GPIO_Pin_Clear(iic_sclk);  
	GPIO_Set_Output(iic_sda);   //恢复输出
	if(mack)GPIO_Pin_Clear(iic_sda);   //主机给回应
	else GPIO_Pin_Set(iic_sda); 
	GPIO_Pin_Set(iic_sclk);
	GPIO_Pin_Clear(iic_sclk); 
	return da;
}

/*
IIC发送设备ID(设备地址)字节函数
输入参数：uint8_t id 设备的地址 这里是七位地址 最终输出的地址将会左移一位并且加1，如0x50->0xb0
					IIC_DIRECTION direction 传输方向 参考IIC_DIRECTION枚举，该变量决定了第一个直接的最低位
*/
void iic_sendid(uint32_t iic_sclk,uint32_t iic_sda,uint8_t id,IIC_DIRECTION direction){
	uint8_t j;
	id<<=1;
	for(j=0;j<7;j++)
	{
		GPIO_Pin_Clear(iic_sclk);
		if((id&0x80)==0x80)
		{		
			GPIO_Pin_Set(iic_sda);
		}
		else
		{
			GPIO_Pin_Clear(iic_sda);
		}
		id=id<<1;
		GPIO_Pin_Set(iic_sclk);
	}
	GPIO_Pin_Clear(iic_sclk);
	if(direction==iic_write) GPIO_Pin_Clear(iic_sda);
	else GPIO_Pin_Set(iic_sda);
	GPIO_Pin_Set(iic_sclk);
	//应答
	GPIO_Pin_Clear(iic_sclk);   
	GPIO_Pin_Set(iic_sda);   //可以查看有没有应答
	GPIO_Pin_Set(iic_sclk);
	GPIO_Pin_Clear(iic_sclk); //注意这里必须要再次清除掉SCLK  否则的话设备就会认为还没有到响应结束，那么就造成了当再次开始时第一个时钟就会被认为是应答结束时钟
}
/*
IIC发送一个字节的数据函数
输入参数：uint8_t da 要发送的数据（一个字节）
*/
void iic_senddata(uint32_t iic_sclk,uint32_t iic_sda,uint8_t da){
	uint8_t m;
	uint8_t j;
	for(j=0;j<8;j++)
	{
		m=da;
		GPIO_Pin_Clear(iic_sclk);
		m=m&0x80;
		if(m==0x80)
		{		
			GPIO_Pin_Set(iic_sda);
		}
		else
		{
			GPIO_Pin_Clear(iic_sda);
		}
		da=da<<1;
		GPIO_Pin_Set(iic_sclk);
	}
	//应答
	GPIO_Pin_Clear(iic_sclk);  
	GPIO_Pin_Set(iic_sda);   //可以查看有没有应答
	GPIO_Pin_Set(iic_sclk);
	GPIO_Pin_Clear(iic_sclk);    
}

/*
IIC开始函数 时序开始 产生停止位
*/
void ioi2c_start(uint32_t iic_sclk,uint32_t iic_sda)
{
	GPIO_Pin_Set(iic_sda);
	GPIO_Pin_Set(iic_sclk);
	GPIO_Pin_Clear(iic_sda);
	GPIO_Pin_Clear(iic_sclk);
}

/*
iic0的写函数
传入参数：uint8_t id 设备的地址 这里是七位地址 最终输出的地址将会左移一位，如0x50->0xa0
					uint8_t addr_len 设备内部地址（寄存器地址）的长度，如果传入I2C_2_BYTE_ADDRESS则为2个字节个地址，否则为1个字节
					uint8_t * buf 读取数据的储存区域
					uint16_t sz 要读取的数据长度
*/
uint8_t i2c_reg_read(uint32_t iic_sclk,uint32_t iic_sda,uint8_t id, uint8_t addr_len, uint16_t addr, uint8_t * buf, uint16_t sz)
{
  uint8_t i=0;
	ioi2c_start(iic_sclk,iic_sda);
	iic_sendid(iic_sclk,iic_sda,id,iic_write);
	if(addr_len==IOI2C_2_BYTE_ADDRESS){
		iic_senddata( iic_sclk, iic_sda,addr>>8);
	}
	iic_senddata(iic_sclk,iic_sda,addr);
	ioi2c_start(iic_sclk,iic_sda);   //需要再次开始 注意
	iic_sendid(iic_sclk, iic_sda,id,iic_read);   //读ID
	for(i=0;i<sz;i++){
		if((sz>1) && (i<(sz-1))) * buf=iic_receivedata(iic_sclk,iic_sda,1);
		else * buf=iic_receivedata(iic_sclk,iic_sda,0);
		buf++;
	}
	ioi2c_stop(iic_sclk,iic_sda);
	return 1;
}

/******************************************
	qma6981_read_bytes
*******************************************/ 
uint32_t qma6981_read_bytes(uint8_t reg, uint8_t *data, uint16_t count)
{	
	ioi2c_ioconfig(QMA6981_SCL, QMA6981_SDA);
	if(i2c_reg_read(QMA6981_SCL, QMA6981_SDA, QMA6981_IIC_ADDR, IOI2C_1_BYTE_ADDRESS, reg, data  ,count))
	{
		return 0x00;
	}
	return 0x01;
}


bool QQMA6981_ConReadBytes(uint8_t* Data, uint8_t RegAddr, uint8_t Length)
{
	return (bool)i2c_reg_read(QMA6981_SCL, QMA6981_SDA, QMA6981_IIC_ADDR, IOI2C_1_BYTE_ADDRESS, RegAddr, Data, Length);
}



uint16_t QQMA6981_read_setpcounter(void)
{
	uint8_t counter[2];
	uint16_t step_data;

//read status
	QQMA6981_ConReadBytes(counter, 0x0a, 1);
	
	
	STEP_STATUS = counter[0];

	QQMA6981_ConReadBytes(counter, 0x07, 2);
	step_data = counter[0] + counter[1] * 16 * 16;

	return step_data;
}






/*
iic0的写函数
传入参数：uint8_t id 设备的地址 这里是七位地址 最终输出的地址将会左移一位并且加1，如0x50->0xb0
					uint8_t addr_len 设备内部地址（寄存器地址）的长度，如果传入I2C_2_BYTE_ADDRESS则为2个字节个地址，否则为1个字节
					uint8_t * buf 要发的数据内容
					uint16_t sz 要发送的数据长度
*/
uint8_t i2c_reg_write(uint32_t iic_sclk,uint32_t iic_sda,uint8_t id, uint8_t addr_len, uint16_t addr, uint8_t * buf, uint16_t sz)
{
	uint8_t i=0;
	ioi2c_start(iic_sclk,iic_sda);
	iic_sendid(iic_sclk,iic_sda,id,iic_write);
	if(addr_len==IOI2C_2_BYTE_ADDRESS){
		iic_senddata(iic_sclk,iic_sda,addr>>8);
	}
	iic_senddata(iic_sclk,iic_sda,addr);
	for(i=0;i<sz;i++){
		iic_senddata(iic_sclk,iic_sda,* buf);
		buf++;
	}
	ioi2c_stop(iic_sclk,iic_sda);
	return 1;  //为了兼容硬件IIC函数
}

/******************************************
	QQMA6981 write bytes
*******************************************/ 
bool QQMA6981_WriteBytes(uint8_t RegAddr, uint8_t Data)
{
	if(i2c_reg_write(QMA6981_SCL, QMA6981_SDA, QMA6981_IIC_ADDR, IOI2C_1_BYTE_ADDRESS, RegAddr, &Data, 1))
	{
		return false;
	}
	return true;
}


void QQMA6981_setp_conuter_enable(void)
{
		bool temp_result;	  
		temp_result = QQMA6981_WriteBytes(0x10 , 0x0e);	//0x0cs
	#if  1 //def SCALE_8G
		temp_result = QQMA6981_WriteBytes(0x0f , 0x04);//range 8G
		temp_result = QQMA6981_WriteBytes(0x13 , 0x10);	//0f//0x65此值改小灵敏度加强
	#else
		temp_result = QQMA6981_WriteBytes(0x0f , 0x01);//range 2G
		temp_result = QQMA6981_WriteBytes(0x13 , 0x40);	//0x65此值改小灵敏度加强  40
	#endif
		temp_result = QQMA6981_WriteBytes(0x12 , 0x90); //0x8c
		temp_result = QQMA6981_WriteBytes(0x14 , 0x0b); //0x12
		temp_result = QQMA6981_WriteBytes(0x15 , 0x09); //0x0f
		temp_result = QQMA6981_WriteBytes(0x32 , 0x01); //0x02->xz  0x01->zy
		
  #if 1//def SCALE_8G
		temp_result = QQMA6981_WriteBytes(0x27 , 0x60);				  
		temp_result = QQMA6981_WriteBytes(0x28 , 0x60);
		temp_result = QQMA6981_WriteBytes(0x29 , 0x60);				 
	#else
	//	temp_result = QQMA6981_WriteBytes(0x27 , 0x64);
	//	temp_result = QQMA6981_WriteBytes(0x28 , 0x64);   
	//	temp_result = QQMA6981_WriteBytes(0x29 , 0x64);
	#endif
		
		temp_result = QQMA6981_WriteBytes(0x16 , 0x0c);
		temp_result = QQMA6981_WriteBytes(0x11 , 0xcb);//0x80
	  
}

void  QQMA6981_stepCounter_On(void)//开启计步
{	
	
	QQMA6981_setp_conuter_enable();
	
}




//======peng======

