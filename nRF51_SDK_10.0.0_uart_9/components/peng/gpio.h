#ifndef _GPIO_H
#define _GPIO_H

#define RST 11
#define SDA 3
#define SCL 4
#define MOT 7

#include <stdint.h>
#include <stdbool.h>

void SetOutput(uint8_t pin,uint8_t value);
void blink(uint8_t gpio);

void GPIO_Pin_Clear(uint32_t pin);
void GPIO_Pin_Set(uint32_t pin);
void GPIO_Set_Input(uint32_t pin);
void GPIO_Set_Output(uint32_t pin);

#endif



