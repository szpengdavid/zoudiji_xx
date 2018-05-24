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

#define Must_step_length 	        16	    //ԭ��12
#define Step_SENSITIVITY          0x0a    //0x13�Ĵ�����ֵ, ԭ��0x19    ��һ���� 0x10          Ҫ���:0x13   Ҫ���:0x0a
#define HIGH_G_SENSITIVITY        0x17    //0x26�Ĵ�����ֵ,���� ���ж�0x0c|0x03    0x14:100%   Ҫ���:0x1f   Ҫ���:0x17->0x14
#define HIGH_G_SENSITIVITY_NIGHT  0x17    //0x26�Ĵ�����ֵ,˯�߿�����ȥ�� 0x0c|0x03   0x14:100%
#define READ_STEP_CYCLE           16384   //16384:0.5s   32768:1s 
#define Sleep_Awake_Filter_Step   36      //��Ϊ26��һ��,���0x13�Ĵ�����Ϊ0x10��ԭ��18

typedef enum {
	iic_write,
	iic_read,
}IIC_DIRECTION;

void GPIO_Pin_Clear(uint32_t pin);
void GPIO_Pin_Set(uint32_t pin);
void GPIO_Set_Input(uint32_t pin);
void GPIO_Set_Output(uint32_t pin);
/*
����iic�ܽź���
gpio15->SDA  gpio16->SCL 
*/
void ioi2c_ioconfig(uint32_t iic_sclk,uint32_t iic_sda);
/*
IICֹͣ���� ����ֹͣλ
*/
void ioi2c_stop(uint32_t iic_sclk,uint32_t iic_sda);

/*
IIC����һ���ֽڵ����ݺ���
���������uint8_t mack   0�����������ӻ���Ӧ 1���������ӻ���Ӧ ��ֻ���ڶ��ֽڶ���ʱ���õ���
����ֵ��uint8_t���յ������ݣ�һ���ֽڣ�
*/
uint8_t iic_receivedata(uint32_t iic_sclk,uint32_t iic_sda,uint8_t mack);

/*
IIC�����豸ID(�豸��ַ)�ֽں���
���������uint8_t id �豸�ĵ�ַ ��������λ��ַ ��������ĵ�ַ��������һλ���Ҽ�1����0x50->0xb0
					IIC_DIRECTION direction ���䷽�� �ο�IIC_DIRECTIONö�٣��ñ��������˵�һ��ֱ�ӵ����λ
*/
void iic_sendid(uint32_t iic_sclk,uint32_t iic_sda,uint8_t id,IIC_DIRECTION direction);
/*
IIC����һ���ֽڵ����ݺ���
���������uint8_t da Ҫ���͵����ݣ�һ���ֽڣ�
*/
void iic_senddata(uint32_t iic_sclk,uint32_t iic_sda,uint8_t da);

/*
IIC��ʼ���� ʱ��ʼ ����ֹͣλ
*/
void ioi2c_start(uint32_t iic_sclk,uint32_t iic_sda);

/*
iic0��д����
���������uint8_t id �豸�ĵ�ַ ��������λ��ַ ��������ĵ�ַ��������һλ����0x50->0xa0
					uint8_t addr_len �豸�ڲ���ַ���Ĵ�����ַ���ĳ��ȣ��������I2C_2_BYTE_ADDRESS��Ϊ2���ֽڸ���ַ������Ϊ1���ֽ�
					uint8_t * buf ��ȡ���ݵĴ�������
					uint16_t sz Ҫ��ȡ�����ݳ���
*/
uint8_t i2c_reg_read(uint32_t iic_sclk,uint32_t iic_sda,uint8_t id, uint8_t addr_len, uint16_t addr, uint8_t * buf, uint16_t sz);

/******************************************
	qma6981_read_bytes
*******************************************/ 
uint32_t qma6981_read_bytes(uint8_t reg, uint8_t *data, uint16_t count);



bool QQMA6981_ConReadBytes(uint8_t* Data, uint8_t RegAddr, uint8_t Length);



uint16_t QQMA6981_read_setpcounter(void);


/*
iic0��д����
���������uint8_t id �豸�ĵ�ַ ��������λ��ַ ��������ĵ�ַ��������һλ���Ҽ�1����0x50->0xb0
					uint8_t addr_len �豸�ڲ���ַ���Ĵ�����ַ���ĳ��ȣ��������I2C_2_BYTE_ADDRESS��Ϊ2���ֽڸ���ַ������Ϊ1���ֽ�
					uint8_t * buf Ҫ������������
					uint16_t sz Ҫ���͵����ݳ���
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
	

