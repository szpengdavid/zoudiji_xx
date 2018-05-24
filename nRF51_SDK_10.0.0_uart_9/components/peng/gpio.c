#include "boards.h"
#include "nrf_delay.h"

//gpioÉèÖÃ³ÉÊä³ö
void SetOutput(uint8_t pin,uint8_t value){	
	
	nrf_gpio_cfg_output(pin);	
	uint32_t gpio_state = NRF_GPIO->OUT;
	
	if(value==0){
		NRF_GPIO->OUTCLR = (~gpio_state) | (1<<pin);   		
	}else{                     
		NRF_GPIO->OUTSET = gpio_state | (1<<pin) ; 	
	}	
}




void GPIO_Pin_Clear(uint32_t pin){
						SetOutput(pin,0);	
}
void GPIO_Pin_Set(uint32_t pin){
						SetOutput(pin,1);	
}
void GPIO_Set_Input(uint32_t pin){
	nrf_gpio_cfg_input(pin,NRF_GPIO_PIN_PULLUP);
}
void GPIO_Set_Output(uint32_t pin){
	nrf_gpio_cfg_output(pin);
}



void blink(int gpio){
	SetOutput(gpio,1);
nrf_delay_ms(1000);
	SetOutput(gpio,0);
nrf_delay_ms(1000);
	SetOutput(gpio,1);
nrf_delay_ms(1000);
	SetOutput(gpio,0);
nrf_delay_ms(1000);

}


