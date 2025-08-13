/*
 * I2C_gpio.c
 *
 *  Created on: Aug 11, 2025
 *      Author: compro
 */


/*
 * i2c_gpio.c
 *
 *  Created on: Aug 2, 2025
 *      Author: compro
 */
#include "main.h"
#include "i2c_gpio.h"
#include <stdio.h>

#if 0
#define GPIO_PORT_I2C	GPIOF
#define I2C_SCL_PIN		GPIO_PIN_1
#define I2C_SDA_PIN		GPIO_PIN_3

#define I2C_SCL_1()  HAL_GPIO_WritePin(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_0()  HAL_GPIO_WritePin(GPIO_PORT_I2C, I2C_SCL_PIN, GPIO_PIN_RESET)

#define I2C_SDA_1()  HAL_GPIO_WritePin(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_0()  HAL_GPIO_WritePin(GPIO_PORT_I2C, I2C_SDA_PIN, GPIO_PIN_RESET)

#define I2C_SDA_READ()  (HAL_GPIO_ReadPin(GPIO_PORT_I2C, I2C_SDA_PIN) == GPIO_PIN_SET)
#define I2C_SCL_READ()  (HAL_GPIO_ReadPin(GPIO_PORT_I2C, I2C_SCL_PIN) == GPIO_PIN_SET)
#endif

#define AS5600_ADDR         0x36
#define AS5600_WRITE        (AS5600_ADDR << 1 | 0)  // 0x6C
#define AS5600_READ         (AS5600_ADDR << 1 | 1)  // 0x6D

#define AS5600_RAW_ANGLE_H  0x0C
#define AS5600_RAW_ANGLE_L  0x0D
#define AS5600_STATUS       0x0B
#define AS5600_MAGNITUDE_H  0x1B
#define AS5600_MAGNITUDE_L  0x1C

void i2c_Delay(void)
{
	uint8_t i;
	for (i = 0; i < 30; i++);
}

void i2c_Start(void)
{
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1();
		}
		_ucByte <<= 1;
		i2c_Delay();
	}
}

uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	value = 0;

	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		i2c_Delay();
	}
	return value;
}

uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();
	i2c_Delay();
	I2C_SCL_1();
	i2c_Delay();
	if (I2C_SDA_READ())
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	i2c_Delay();
	return re;
}

void i2c_Ack(void)
{
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();
}

void i2c_NAck(void)
{
	I2C_SDA_1();
	i2c_Delay();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

void i2c_Stop(void)
{
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

void bsp_InitI2C(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	__HAL_RCC_GPIOF_CLK_ENABLE();

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pin = I2C_SCL_PIN | I2C_SDA_PIN;

	HAL_GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);

	i2c_Stop();
}

uint8_t i2c_CheckDevice(uint8_t _Address)
{
    uint8_t ucAck;

    //printf("AS5600 디바이스 체크 중...\n");
    //printf("7비트 주소: 0x%02X\n", _Address);

    if (I2C_SDA_READ() && I2C_SCL_READ())
    {
        //printf("I2C 버스 상태 정상\n");

        i2c_Start();
        //printf("START 신호 전송\n");

        uint8_t write_addr = (_Address << 1) | I2C_WR;  // I2C 주소 체계는 7비트 , 8비트 주소 생성 필요
        i2c_SendByte(write_addr);
        //printf("8비트 주소 0x%02X 전송\n", write_addr);

        ucAck = i2c_WaitAck();
        //printf("ACK 응답: %d\n", ucAck);

        if(ucAck == 0){
            printf("AS5600 확인!\n");
        }
        else
        {
            printf("AS5600 확인 실패!\n");
        }

        i2c_Stop();

        return ucAck;
    }
    return 1;
}

uint8_t AS5600_ReadRegister(uint8_t reg_addr)
{
	uint8_t data = 0xFF;
	uint8_t ack;

	i2c_Start();

	i2c_SendByte(AS5600_WRITE);
	ack = i2c_WaitAck();
	if(ack != 0){
		i2c_Stop();
		return 0xFF;  // 오류
	}

	i2c_SendByte(reg_addr);
	ack = i2c_WaitAck();
	if (ack != 0) {
		i2c_Stop();
	    return 0xFF;  // 오류
	}

	i2c_Start();
	i2c_SendByte(AS5600_READ);
	ack = i2c_WaitAck();
	if (ack != 0) {
		i2c_Stop();
	    return 0xFF;  // 오류
	}

	data = i2c_ReadByte();
	i2c_NAck();  // 마지막 바이트이므로 NACK
	i2c_Stop();

	return data;
}

uint16_t AS5600_ReadRawAngle(void)
{
    uint8_t high_byte, low_byte;
    uint16_t raw_angle;

    // 상위 바이트 읽기
    high_byte = AS5600_ReadRegister(AS5600_RAW_ANGLE_H);
    if (high_byte == 0xFF) {
        return 0xFFFF;  // 오류
    }

    // 하위 바이트 읽기
    low_byte = AS5600_ReadRegister(AS5600_RAW_ANGLE_L);
    if (low_byte == 0xFF) {
        return 0xFFFF;  // 오류
    }

    // 12비트 각도 조합 (상위 4비트 + 하위 8비트)
    raw_angle = ((uint16_t)(high_byte & 0x0F) << 8) | low_byte;

    return raw_angle;
}

float AS5600_GetAngle(void) {
    uint16_t raw_angle = AS5600_ReadRawAngle();

    if(raw_angle == 0xFFFF) {  // 오류 확인
        return 0.0f;  // 오류시 0 리턴
    }

    // 12비트 값(0~4095)을 라디안(0~2π)으로 변환
    return (float)raw_angle * 2.0f * 3.14159f / 4096.0f;
}

/**
 * AS5600 상태 확인
 */
void AS5600_CheckStatus(void)
{
    uint8_t status = AS5600_ReadRegister(AS5600_STATUS);

    printf("AS5600 상태 레지스터: 0x%02X\n", status);

    if (status != 0xFF) {
        if (status & 0x20) {
            printf("✅ 자석 감지됨\n");
        } else {
            printf("⚠️  자석 감지 안됨\n");
        }

        if (status & 0x10) {
            printf("⚠️  자석이 너무 강함\n");
        }

        if (status & 0x08) {
            printf("⚠️  자석이 너무 약함\n");
        }

        if ((status & 0x18) == 0) {
            printf("✅ 자석 강도 적절함\n");
        }
    } else {
        printf("❌ 상태 레지스터 읽기 실패\n");
    }
}

/**
 * AS5600 자기장 강도 확인
 */
void AS5600_CheckMagnitude(void)
{
    uint8_t mag_h = AS5600_ReadRegister(AS5600_MAGNITUDE_H);
    uint8_t mag_l = AS5600_ReadRegister(AS5600_MAGNITUDE_L);

    if (mag_h != 0xFF && mag_l != 0xFF) {
        uint16_t magnitude = ((uint16_t)(mag_h & 0x0F) << 8) | mag_l;
        printf("자기장 강도: %d\n", magnitude);

        if (magnitude > 2000) {
            printf("✅ 자기장 강도 양호\n");
        } else {
            printf("⚠️  자기장 강도 약함 - 자석을 가까이 가져다 대세요\n");
        }
    } else {
        printf("❌ 자기장 강도 읽기 실패\n");
    }
}

/**
 * AS5600 전체 테스트
 */
void AS5600_FullTest(void)
{
    printf("\n=== AS5600 전체 테스트 ===\n");

    // 1. 디바이스 체크
    //if (AS5600_CheckDevice() != 0) {
        //printf("AS5600 연결 실패 - 테스트 중단\n");
        //return;
    //}

    // 2. 상태 확인
    AS5600_CheckStatus();

    // 3. 자기장 강도 확인
    AS5600_CheckMagnitude();

    // 4. 각도 읽기 테스트 (10회)
    printf("\n--- 각도 읽기 테스트 ---\n");
    for (int i = 0; i < 10; i++) {
        uint16_t raw_angle = AS5600_ReadRawAngle();

        if (raw_angle != 0xFFFF) {
            float angle_deg = (raw_angle * 360.0f) / 4096.0f;
            float angle_rad = (raw_angle * 2.0f * 3.14159f) / 4096.0f;

            printf("읽기 %d: Raw=%d, 각도=%.1f°, %.3f rad\n", i+1, raw_angle, angle_deg, angle_rad);
        } else {
            printf("읽기 %d: 실패\n", i+1);
        }

        HAL_Delay(500);  // 0.5초 간격
    }

    printf("\n자석을 천천히 돌려보세요...\n");
}

/**
 * AS5600 연속 모니터링
 */
void AS5600_ContinuousMonitor(void)
{
    static uint32_t last_time = 0;

    if (HAL_GetTick() - last_time > 200) {  // 5Hz 업데이트
        uint16_t raw_angle = AS5600_ReadRawAngle();

        if (raw_angle != 0xFFFF) {
            float angle_deg = (raw_angle * 360.0f) / 4096.0f;

            printf("AS5600: %.1f° (Raw: %d)\n", angle_deg, raw_angle);
        } else {
            printf("AS5600: 읽기 실패\n");
        }

        last_time = HAL_GetTick();
    }
}
