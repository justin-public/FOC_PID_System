/*
 * I2C_Gpio.h
 *
 *  Created on: Aug 11, 2025
 *      Author: compro
 */

#ifndef INC_I2C_GPIO_H_
#define INC_I2C_GPIO_H_

#define I2C_WR	0
#define I2C_RD	1

#define GPIO_PORT_I2C	GPIOF
#define I2C_SCL_PIN		GPIO_PIN_1
#define I2C_SDA_PIN		GPIO_PIN_3

#define I2C_SCL_1()  HAL_GPIO_WritePin(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_0()  HAL_GPIO_WritePin(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_PIN_RESET)

#define I2C_SDA_1()  HAL_GPIO_WritePin(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_0()  HAL_GPIO_WritePin(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_PIN_RESET)

#define I2C_SDA_READ()  (HAL_GPIO_ReadPin(GPIO_PORT_I2C, I2C_SDA_PIN) == GPIO_PIN_SET)     // SET
#define I2C_SCL_READ()  (HAL_GPIO_ReadPin(GPIO_PORT_I2C, I2C_SCL_PIN) == GPIO_PIN_SET)     // SET

void bsp_InitI2C(void);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(void);
uint8_t i2c_WaitAck(void);
void i2c_Delay(void);
void i2c_Ack(void);
void i2c_NAck(void);
uint8_t i2c_CheckDevice(uint8_t _Address);
uint8_t AS5600_ReadRegister(uint8_t reg_addr);
uint16_t AS5600_ReadRawAngle(void);
void AS5600_CheckStatus(void);
void AS5600_CheckMagnitude(void);
void AS5600_FullTest(void);
void AS5600_ContinuousMonitor(void);
float AS5600_GetAngle(void);

#endif /* INC_I2C_GPIO_H_ */
