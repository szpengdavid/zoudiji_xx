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
����iic�ܽź���
gpio15->SDA  gpio16->SCL 
*/
void ioi2c_ioconfig(uint32_t iic_sclk,uint32_t iic_sda){
	GPIO_Set_Output(iic_sclk);
	GPIO_Set_Output(iic_sda);
	GPIO_Pin_Set(iic_sclk);   //LCD_CS
	GPIO_Pin_Set(iic_sda);    //LCD_POWER_EN
}
/*
IICֹͣ���� ����ֹͣλ
*/
void ioi2c_stop(uint32_t iic_sclk,uint32_t iic_sda)
{
	GPIO_Pin_Clear(iic_sclk);
	GPIO_Pin_Clear(iic_sda);
	GPIO_Pin_Set(iic_sclk);
	GPIO_Pin_Set(iic_sda);
}

/*
IIC����һ���ֽڵ����ݺ���
���������uint8_t mack   0�����������ӻ���Ӧ 1���������ӻ���Ӧ ��ֻ���ڶ��ֽڶ���ʱ���õ���
����ֵ��uint8_t���յ������ݣ�һ���ֽڣ�
*/
uint8_t iic_receivedata(uint32_t iic_sclk,uint32_t iic_sda,uint8_t mack){
	uint8_t j,da=0;
	GPIO_Set_Input(iic_sda);  //��Ϊ����
	for(j=0;j<8;j++)
	{
		GPIO_Pin_Clear(iic_sclk);
		GPIO_Pin_Set(iic_sclk);
		if(nrf_gpio_pin_read(iic_sda)) da=(da<<1)+1;
		else da=da<<1;
	}
	 //Ӧ��
	GPIO_Pin_Clear(iic_sclk);  
	GPIO_Set_Output(iic_sda);   //�ָ����
	if(mack)GPIO_Pin_Clear(iic_sda);   //��������Ӧ
	else GPIO_Pin_Set(iic_sda); 
	GPIO_Pin_Set(iic_sclk);
	GPIO_Pin_Clear(iic_sclk); 
	return da;
}

/*
IIC�����豸ID(�豸��ַ)�ֽں���
���������uint8_t id �豸�ĵ�ַ ��������λ��ַ ��������ĵ�ַ��������һλ���Ҽ�1����0x50->0xb0
					IIC_DIRECTION direction ���䷽�� �ο�IIC_DIRECTIONö�٣��ñ��������˵�һ��ֱ�ӵ����λ
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
	//Ӧ��
	GPIO_Pin_Clear(iic_sclk);   
	GPIO_Pin_Set(iic_sda);   //���Բ鿴��û��Ӧ��
	GPIO_Pin_Set(iic_sclk);
	GPIO_Pin_Clear(iic_sclk); //ע���������Ҫ�ٴ������SCLK  ����Ļ��豸�ͻ���Ϊ��û�е���Ӧ��������ô������˵��ٴο�ʼʱ��һ��ʱ�Ӿͻᱻ��Ϊ��Ӧ�����ʱ��
}
/*
IIC����һ���ֽڵ����ݺ���
���������uint8_t da Ҫ���͵����ݣ�һ���ֽڣ�
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
	//Ӧ��
	GPIO_Pin_Clear(iic_sclk);  
	GPIO_Pin_Set(iic_sda);   //���Բ鿴��û��Ӧ��
	GPIO_Pin_Set(iic_sclk);
	GPIO_Pin_Clear(iic_sclk);    
}

/*
IIC��ʼ���� ʱ��ʼ ����ֹͣλ
*/
void ioi2c_start(uint32_t iic_sclk,uint32_t iic_sda)
{
	GPIO_Pin_Set(iic_sda);
	GPIO_Pin_Set(iic_sclk);
	GPIO_Pin_Clear(iic_sda);
	GPIO_Pin_Clear(iic_sclk);
}

/*
iic0��д����
���������uint8_t id �豸�ĵ�ַ ��������λ��ַ ��������ĵ�ַ��������һλ����0x50->0xa0
					uint8_t addr_len �豸�ڲ���ַ���Ĵ�����ַ���ĳ��ȣ��������I2C_2_BYTE_ADDRESS��Ϊ2���ֽڸ���ַ������Ϊ1���ֽ�
					uint8_t * buf ��ȡ���ݵĴ�������
					uint16_t sz Ҫ��ȡ�����ݳ���
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
	ioi2c_start(iic_sclk,iic_sda);   //��Ҫ�ٴο�ʼ ע��
	iic_sendid(iic_sclk, iic_sda,id,iic_read);   //��ID
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
iic0��д����
���������uint8_t id �豸�ĵ�ַ ��������λ��ַ ��������ĵ�ַ��������һλ���Ҽ�1����0x50->0xb0
					uint8_t addr_len �豸�ڲ���ַ���Ĵ�����ַ���ĳ��ȣ��������I2C_2_BYTE_ADDRESS��Ϊ2���ֽڸ���ַ������Ϊ1���ֽ�
					uint8_t * buf Ҫ������������
					uint16_t sz Ҫ���͵����ݳ���
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
	return 1;  //Ϊ�˼���Ӳ��IIC����
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
		temp_result = QQMA6981_WriteBytes(0x13 , 0x10);	//0f//0x65��ֵ��С�����ȼ�ǿ
	#else
		temp_result = QQMA6981_WriteBytes(0x0f , 0x01);//range 2G
		temp_result = QQMA6981_WriteBytes(0x13 , 0x40);	//0x65��ֵ��С�����ȼ�ǿ  40
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

void  QQMA6981_stepCounter_On(void)//�����Ʋ�
{	
	
	QQMA6981_setp_conuter_enable();
	
}




//======peng======

