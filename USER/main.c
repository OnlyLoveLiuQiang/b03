#include"stm32f10x.h"
#include"key.h"
#include"exti.h"
#include"led.h"
#include"delay.h"
#include"usart.h"
#include"mpu9250.h"

int main(){
	unsigned char i = 0;
	unsigned char whoami = 0;
	unsigned char akmid = 0;
	unsigned char accel[6] = "";
	unsigned char gyro[6] = "";
	unsigned char mag[6] = "";
	key_init();
	led_init();
//	led_on(1);
	key_on();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	exti0_init();
	delay_init();
	uart3_init(115200);
	spi2_init();
	MPU9250_init();
	whoami = MPU9250_Read_Reg(WHO_AM_I);
	delay_ms(1000);
	delay_ms(1000);
	USART_SendData(USART3,whoami);
	/*
	while(1){
		akmid = i2c_Mag_read(AK8963_WHOAMI_REG);
		delay_ms(1000);
		delay_ms(1000);
		USART_SendData(USART3,akmid);
	}
	*/

	while(1){
//		READ_MPU9250_ACCEL(accel);
//		READ_MPU9250_GYRO(gyro);
		READ_MPU9250_MAG(mag);
		delay_ms(1000);
		for(i=0;i<6;i++){
//			USART_SendData(USART3,accel[i]);
//			USART_SendData(USART3,gyro[i]);
			USART_SendData(USART3,mag[i]);
			delay_ms(50);
		}
		delay_ms(1000);
		delay_ms(1000);
	}

}
