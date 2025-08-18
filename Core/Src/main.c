#include "main.h"
#include "I2C_gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

// 모터 파라미터
#define POLE_PAIRS 7

// 전류 센서 파라미터
#define VREF 3.3f
#define INA240_VREF 2.5f
#define ACTUAL_ZERO_A  1.653f  // 실제 측정된 A상 제로점
#define ACTUAL_ZERO_B  1.654f  // 실제 측정된 B상 제로점
#define ADC_RESOLUTION 4096.0f
#define INA240_GAIN 20.0f
#define R_SENSE 0.01f

// FOC 관련 파라미터
#define PI 3.14159265359f
#define TWO_PI (2.0f * PI)
#define SQRT3 1.732050808f
#define SQRT3_2 0.866025404f

// 전류 센서 변수
volatile uint16_t adc_values[2];
volatile uint8_t adc_conversion_complete = 0;
float current_a = 0.0f;
float current_b = 0.0f;
float current_c = 0.0f;

// 3상 모터 제어 변수
uint32_t pwm_period;
uint8_t motor_enabled = 0;
float angle = 0.0f;

// 엔코더 관련 변수
float electrical_offset = 0.0f;  // 엔코더 오프셋 저장

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
void Current_Sensor_Init(void);
void Read_Current_Sensors(void);
static void MX_USART2_UART_Init(void);

// 3상 인버터
void set_pwm_duty(float a, float b, float c);
void motor_enable(void);
void motor_disable(void);
void motor_control(void);
float electrical_alignment_test(void);  // d축 정렬 함수

void FOC_control_step1_openloop(void);

// 엔코더
float normalize_angle(float angle);

// FOC + PID
typedef struct{
	float alpha;
	float beta;
} AlphaBeta_t;

typedef struct {
    float d;
    float q;
} DQ_t;

AlphaBeta_t i_ab = {0, 0};
DQ_t i_dq = {0, 0};
DQ_t v_dq = {0, 0};

void FOC_control(void);
AlphaBeta_t clarke_transform(float a, float b, float c);
DQ_t park_transform(AlphaBeta_t ab, float theta);

// FOC 관련 변수
float electrical_angle = 0.0f;			// 전기적 각도 변수
float mechanical_angle = 0.0f;			// 기계적 각도 변수
float current_velocity = 0.0f;
float target_velocity = 0.0f;			// 초기값을 0으로 설정

int _write(int fd, char *ptr, int len) {
    HAL_StatusTypeDef hstatus;

    if (fd == 1 || fd == 2) {
        hstatus = HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
        if (hstatus == HAL_OK)
            return len;
        else
            return -1;
    }
    return -1;
}


int main(void)
{
	HAL_Init();

	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC3_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	bsp_InitI2C();

	motor_disable();
	Current_Sensor_Init();

	uint8_t result = i2c_CheckDevice(0x36);  // AS5600 주소
	if(result == 0){
		printf("AS5600 detect! \r\n");
	}
	else{
		printf("AS5600 not detect! \r\n");
	}

	AS5600_FullTest();

	pwm_period = htim1.Init.Period;

	//PWM 시작
	set_pwm_duty(0.5f, 0.5f, 0.5f);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	motor_enable();

	HAL_Delay(2000);
	electrical_offset = electrical_alignment_test();
	printf("Initial electrical offset saved: %.3f rad\r\n", electrical_offset);


	float desired_velocity = 5.0f;  // 최종 원하는 속도
	target_velocity = 0.0f;

	uint32_t foccontrol_time = HAL_GetTick();

	//uint32_t velocity_change_time = HAL_GetTick();

	while(1)
	{
		// 5ms = 200Hz
		if(HAL_GetTick() - foccontrol_time >= 5) {
			FOC_control();
			foccontrol_time = HAL_GetTick();
		}
#if 0
		float max_acceleration = 8.0f;  // 10 rad/s² 최대 가속도
		float velocity_error = desired_velocity - target_velocity;
		float max_change = max_acceleration * 0.005f;  // 5ms 간격

		if(fabsf(velocity_error) > max_change){
			if(velocity_error > 0){
				target_velocity += max_change;
			}
			else{
				target_velocity -= max_change;
			}
		}
		else{
			target_velocity = desired_velocity;
		}
#endif
	}

}

float electrical_alignment_test(void) {
    printf("Start electrical alignment test\r\n");

    // 1. d축에 고정 전압 인가 (로터를 d축 방향으로 강제 정렬)
    float vd_align = 3.0f;  // d축 전압 (적당한 크기)
    float vq_align = 0.0f;  // q축 전압 없음

    // 2. 임의의 전기각도로 설정 (예: 0도)
    float test_electrical_angle = 0.0f;
    float cos_theta = cosf(test_electrical_angle);
    float sin_theta = sinf(test_electrical_angle);

    // 3. 역 Park 변환
    float v_alpha = vd_align * cos_theta - vq_align * sin_theta;
    float v_beta = vd_align * sin_theta + vq_align * cos_theta;

    // 4. 역 Clarke 변환
    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    // 5. PWM 듀티 계산
    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    // 6. PWM 출력 (2초간)
    printf("d축 전압 인가 중... 로터가 정렬됩니다.\r\n");

    for(int i = 0; i < 400; i++) {  // 2초간 (5ms × 400회)
        set_pwm_duty(duty_a, duty_b, duty_c);
        HAL_Delay(5);

        // 매 100ms마다 엔코더 값 출력
        if(i % 20 == 0) {
            uint16_t raw = AS5600_ReadRawAngle();
            //float angle = (4096.0f - (float)raw) * TWO_PI / 4096.0f;
            float angle = ((float)raw) * TWO_PI / 4096.0f;
            printf("Time: %dms, Raw: %d, Angle: %.3f rad (%.1f deg)\r\n",
                   i*5, raw, angle, angle * 180.0f / PI);
        }
    }
#if 1
    // 7. 최종 측정
    uint16_t final_raw = AS5600_ReadRawAngle();
    //float final_angle = (4096.0f - (float)final_raw) * TWO_PI / 4096.0f;
    float final_angle = ((float)final_raw) * TWO_PI / 4096.0f;
#endif
    // 8. PWM 정지
    set_pwm_duty(0.5f, 0.5f, 0.5f);
    return final_angle;
}

void set_pwm_duty(float a, float b, float c) {
    if (a < 0.0f) a = 0.0f; if (a > 1.0f) a = 1.0f;
    if (b < 0.0f) b = 0.0f; if (b > 1.0f) b = 1.0f;
    if (c < 0.0f) c = 0.0f; if (c > 1.0f) c = 1.0f;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(a * pwm_period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(b * pwm_period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(c * pwm_period));
}

void motor_enable(void){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
	motor_enabled = 1;
}

void motor_disable(void){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
	motor_enabled = 0;
}


void motor_control(void) {
    if (!motor_enabled) {
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 각도 증가
    angle += 2 * 0.01f * POLE_PAIRS;
    if (angle > 6.28318f) angle -= 6.28318f;

    // 3상 전압 계산
    float amplitude = 1 / 12.0f;

    float va = amplitude * cosf(angle);
    float vb = amplitude * cosf(angle - 2.094395f);
    float vc = amplitude * cosf(angle + 2.094395f);

    float duty_a = 0.5f + va * 0.5f;
    float duty_b = 0.5f + vb * 0.5f;
    float duty_c = 0.5f + vc * 0.5f;

    set_pwm_duty(duty_a, duty_b, duty_c);
}

void Read_Current_Sensors(void){
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_values, 2);
	uint32_t timeout = HAL_GetTick() + 100;
	while(!adc_conversion_complete && HAL_GetTick() < timeout) {
		// ADC 변환 완료 대기
	}
	if(adc_conversion_complete){
		adc_conversion_complete = 0;

		float adc_voltage_a = (float)adc_values[0] * VREF / ADC_RESOLUTION;
		float adc_voltage_b = (float)adc_values[1] * VREF / ADC_RESOLUTION;
		float R_sense = 0.01f;

		current_a = (adc_voltage_a - ACTUAL_ZERO_A) / (INA240_GAIN * R_sense);
		current_b = (adc_voltage_b - ACTUAL_ZERO_B) / (INA240_GAIN * R_sense);
		current_c = -(current_a + current_b);

		if(fabs(current_a) < 0.02f) current_a = 0.0f;
		if(fabs(current_b) < 0.02f) current_b = 0.0f;
		if(fabs(current_c) < 0.02f) current_c = 0.0f;

		 //printf("Raw ADC: [%d, %d] | V: [%.3f, %.3f] | I_abc: [%.3f, %.3f, %.3f]A\r\n",
		               //adc_values[0], adc_values[1], adc_voltage_a, adc_voltage_b,
		               //current_a, current_b, current_c);
	}
}

// 전류 센서 초기화
void Current_Sensor_Init(void)
{
	if(HAL_ADCEx_Calibration_Start(&hadc3) != HAL_OK) {
		printf("ERROR: ADC Calibration Failed!\r\n");
		Error_Handler();
	}
	if (HAL_TIM_Base_Start(&htim1) != HAL_OK) {
		printf("ERROR: Timer1 Start Failed!\r\n");
		Error_Handler();
	}
	// TIM1 CH3 Compare 이벤트 활성화
	if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
		printf("ERROR: Timer1 CH3 OC Start Failed!\r\n");
		Error_Handler();
	}
	if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_values, 2) != HAL_OK) {
		printf("ERROR: ADC DMA Start Failed!\r\n");
		Error_Handler();
	}
	printf("Current sensor hardware initialization complete\r\n");
	HAL_Delay(1000);
}

float normalize_angle(float angle){
	while (angle > PI) angle -= TWO_PI;
	while (angle < -PI) angle += TWO_PI;
	return angle;
}


#if 0
// 오픈 루프 FOC 가감속 확인
void FOC_control(void){
    //if(!motor_enabled){
        //set_pwm_duty(0.5f, 0.5f, 0.5f);
        //return;
    //}

    // 엔코더 대신 고정된 각도로 테스트
    static float fixed_electrical_angle = 0;
    fixed_electrical_angle += 0.3f;  // 일정한 각속도
    if(fixed_electrical_angle > TWO_PI) fixed_electrical_angle = 0;

    // 간단한 오픈루프 FOC
    float vd = 0.0f;
    float vq = 4.0f;        // 고정된 토크

    // dq → αβ 역변환
    float cos_theta = cosf(fixed_electrical_angle);
    float sin_theta = sinf(fixed_electrical_angle);

    float v_alpha = vd * cos_theta - vq * sin_theta;
    float v_beta = vd * sin_theta + vq * cos_theta;

    // αβ → abc 역변환
    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    // PWM duty cycle 변환
    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    set_pwm_duty(duty_a, duty_b, duty_c);

    printf("FixedAngle: %.2f, Va: %.2f, Duty_a: %.3f\r\n",
           fixed_electrical_angle, va, duty_a);
}
#endif

// 각도 오프셋 적용 오픈루프 FOC
#if 0
void FOC_control(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 엔코더 각도 읽기
    uint16_t raw_angle = AS5600_ReadRawAngle();
    float raw_mechanical = (4096.0f - (float)raw_angle) * TWO_PI / 4096.0f;

    // 오프셋 적용
    raw_mechanical -= 0.32f;

    // 각도 점프 제거 (부드러운 필터링)
    static float smooth_mechanical = 0;
    static uint8_t first_run = 1;

    if(first_run) {
        smooth_mechanical = raw_mechanical;
        first_run = 0;
    } else {
        // 각도 차이 계산
        float angle_diff = raw_mechanical - smooth_mechanical;

        // 2π 경계 보정
        while(angle_diff > PI) angle_diff -= TWO_PI;
        while(angle_diff < -PI) angle_diff += TWO_PI;

        // 부드러운 필터링 (각도 점프 방지)
        smooth_mechanical += angle_diff * 0.1f;  // 10%씩만 변화
    }

    mechanical_angle = smooth_mechanical;

    // 각도 정규화
    while(mechanical_angle > TWO_PI) mechanical_angle -= TWO_PI;
    while(mechanical_angle < 0) mechanical_angle += TWO_PI;

    electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);

    // FOC 제어 (속도 감소)
    float vd = 0.0f;
    float vq = 0.5f;  // 1.5f → 0.5f로 대폭 감소 (속도 줄이기)

    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);

    float v_alpha = vd * cos_theta - vq * sin_theta;
    float v_beta = vd * sin_theta + vq * cos_theta;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    set_pwm_duty(duty_a, duty_b, duty_c);

    printf("Raw: %d, SmoothMech: %.2f, ElecAngle: %.2f, Vq: %.1f\r\n",
           raw_angle, mechanical_angle, electrical_angle, vq);
}
#endif

#if 0
void FOC_control_step1_openloop(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 간단한 오픈루프 테스트
    static float test_angle = 0;
    test_angle += 0.02f;  // 매우 천천히 증가
    if(test_angle > TWO_PI) test_angle = 0;

    // 낮은 전압으로 테스트
    float vd = 0.0f;
    float vq = 1.0f;  // 매우 낮은 전압

    float cos_theta = cosf(test_angle);
    float sin_theta = sinf(test_angle);

    float v_alpha = vd * cos_theta - vq * sin_theta;
    float v_beta = vd * sin_theta + vq * cos_theta;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    set_pwm_duty(duty_b, duty_a, duty_c);  // A/B 교체 유지

    printf("Test angle: %.2f, Va: %.2f, Duty_a: %.3f\r\n", test_angle, va, duty_a);
}
#endif

#if 0
// 즉시 회전을 위한 수정 코드
void FOC_control(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 1. 간단한 오프셋 캘리브레이션 (빠르게)
    static float ia_offset = 0, ib_offset = 0;
    static uint8_t offset_calibrated = 0;
    static int offset_count = 0;

    if(!offset_calibrated && offset_count < 100) {  // 100샘플로 빠르게
        Read_Current_Sensors();
        ia_offset += current_a;
        ib_offset += current_b;
        offset_count++;

        if(offset_count >= 100) {
            ia_offset /= 100.0f;
            ib_offset /= 100.0f;
            offset_calibrated = 1;
            printf("🔧 빠른 캘리브레이션 완료: Ia=%.4f, Ib=%.4f\r\n", ia_offset, ib_offset);
        } else {
            set_pwm_duty(0.5f, 0.5f, 0.5f);
            return;
        }
    }

    // 2. 안정적인 각도 처리 (3단계와 동일)
    uint16_t raw_angle = AS5600_ReadRawAngle();
    float raw_mechanical_angle = ((float)raw_angle) * TWO_PI / 4096.0f;
    float raw_mechanical = normalize_angle(raw_mechanical_angle - electrical_offset);

    static float smooth_mechanical = 0;
    static uint8_t first_run = 1;
    static float prev_raw = 0;

    if(first_run) {
        smooth_mechanical = raw_mechanical;
        prev_raw = raw_mechanical;
        first_run = 0;
    } else {
        float angle_diff = raw_mechanical - prev_raw;

        while(angle_diff > PI) angle_diff -= TWO_PI;
        while(angle_diff < -PI) angle_diff += TWO_PI;

        if(angle_diff > 0.02f) angle_diff = 0.02f;  // 0.01f → 0.02f (조금 완화)
        else if(angle_diff < -0.02f) angle_diff = -0.02f;

        smooth_mechanical += angle_diff * 0.3f;  // 0.1f → 0.3f (더 빠른 응답)
        prev_raw = raw_mechanical;
    }

    mechanical_angle = smooth_mechanical;
    while(mechanical_angle > TWO_PI) mechanical_angle -= TWO_PI;
    while(mechanical_angle < 0) mechanical_angle += TWO_PI;

    electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);

    // 3. 전류 측정 (가벼운 필터링)
    Read_Current_Sensors();

    float ia_corrected = current_a - ia_offset;
    float ib_corrected = current_b - ib_offset;

    static float ia_filtered = 0, ib_filtered = 0;
    ia_filtered = 0.8f * ia_filtered + 0.2f * ia_corrected;  // 95% → 80% (더 빠른 응답)
    ib_filtered = 0.8f * ib_filtered + 0.2f * ib_corrected;

    float i_alpha = ia_filtered;
    float i_beta = (ia_filtered + 2.0f * ib_filtered) / 1.732f;

    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);
    float id_measured = i_alpha * cos_theta + i_beta * sin_theta;
    float iq_measured = -i_alpha * sin_theta + i_beta * cos_theta;

    // 4. ★★★ 토크 대폭 증가 ★★★
    float id_ref = 0.0f;
    float iq_ref = 2.0f;  // 0.2f → 2.0f (10배 증가!)

    static float id_integral = 0.0f;
    static float iq_integral = 0.0f;

    float id_error = id_ref - id_measured;
    float iq_error = iq_ref - iq_measured;

    // 적분 더 빠르게
    id_integral += id_error * 0.002f;  // 0.0005f → 0.002f
    iq_integral += iq_error * 0.002f;

    // 적분 제한 완화
    if(id_integral > 0.3f) id_integral = 0.3f;   // 0.05f → 0.3f
    if(id_integral < -0.3f) id_integral = -0.3f;
    if(iq_integral > 0.3f) iq_integral = 0.3f;
    if(iq_integral < -0.3f) iq_integral = -0.3f;

    // 게인 증가
    float kp = 0.1f;  // 0.02f → 0.1f
    float ki = 0.3f;  // 0.05f → 0.3f

    float vd_command = kp * id_error + ki * id_integral;
    float vq_command = kp * iq_error + ki * iq_integral;

    // 전압 제한 완화
    if(vd_command > 4.0f) vd_command = 4.0f;   // 1.0f → 4.0f
    if(vd_command < -4.0f) vd_command = -4.0f;
    if(vq_command > 4.0f) vq_command = 4.0f;
    if(vq_command < -4.0f) vq_command = -4.0f;

    // 전압 필터링 완화
    static float vd_smooth = 0, vq_smooth = 0;
    vd_smooth = 0.7f * vd_smooth + 0.3f * vd_command;  // 90% → 70%
    vq_smooth = 0.7f * vq_smooth + 0.3f * vq_command;

    // 5. 역변환 및 PWM
    float v_alpha = vd_smooth * cos_theta - vq_smooth * sin_theta;
    float v_beta = vd_smooth * sin_theta + vq_smooth * cos_theta;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    // 듀티 제한 완화
    if(duty_a > 0.9f) duty_a = 0.9f;   // 0.75f → 0.9f
    if(duty_a < 0.1f) duty_a = 0.1f;   // 0.25f → 0.1f
    if(duty_b > 0.9f) duty_b = 0.9f;
    if(duty_b < 0.1f) duty_b = 0.1f;
    if(duty_c > 0.9f) duty_c = 0.9f;
    if(duty_c < 0.1f) duty_c = 0.1f;

    set_pwm_duty(duty_b, duty_a, duty_c);

    // 6. 상태 출력 (더 자주)
    static uint32_t last_print = 0;
    if(HAL_GetTick() - last_print > 1000) {
        printf("토크증가: Raw:%d | 전류: Ia=%.3f, Ib=%.3f | DQ: Id=%.3f, Iq=%.3f | 전압: Vd=%.2f, Vq=%.2f\r\n",
               raw_angle, ia_filtered, ib_filtered, id_measured, iq_measured, vd_smooth, vq_smooth);

        // 회전 상태 확인
        if(fabsf(iq_measured) > 0.5f) {
            printf("✅ 전류 흐름 - 회전해야 함!\r\n");
        } else if(fabsf(vq_smooth) > 1.0f) {
            printf("⚡ 전압 인가 중 - 토크 생성 중!\r\n");
        } else {
            printf("⚠️ 토크 부족 - 더 증가 필요\r\n");
        }

        last_print = HAL_GetTick();
    }
}
#endif

#if 0
// FOC_control() 함수를 이것으로 교체하세요!
void FOC_control(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 1. 각도 읽기 (간단하게)
    uint16_t raw_angle = AS5600_ReadRawAngle();
    float raw_mechanical_angle = ((float)raw_angle) * TWO_PI / 4096.0f;
    float mechanical_angle = normalize_angle(raw_mechanical_angle - electrical_offset);
    float electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);

    // 2. ★★★ 최대 토크 오픈루프 ★★★
    static float torque_level = 2.0f;  // 시작 토크
    static uint32_t torque_change_time = 0;
    static uint8_t torque_started = 0;
    static uint32_t start_time = 0;
    static int torque_step = 0;

    if(start_time == 0) start_time = HAL_GetTick();

    // 2초 후 토크 증가 시작
    if(!torque_started && (HAL_GetTick() - start_time > 2000)) {
        torque_started = 1;
        torque_change_time = HAL_GetTick();
        printf("\r\n🚀 최대 토크 테스트 시작! 🚀\r\n");
    }

    // 3초마다 토크 증가
    if(torque_started && (HAL_GetTick() - torque_change_time > 3000)) {
        switch(torque_step) {
            case 0:
                torque_level = 4.0f;
                printf("\r\n⚡ 토크 4.0V - 중간 출력! ⚡\r\n");
                break;
            case 1:
                torque_level = 6.0f;
                printf("\r\n🔥 토크 6.0V - 높은 출력! 🔥\r\n");
                break;
            case 2:
                torque_level = 8.0f;
                printf("\r\n💥 토크 8.0V - 최대 출력! 💥\r\n");
                break;
            case 3:
                torque_level = 1.0f;
                printf("\r\n🔍 토크 1.0V - 최소 출력! 🔍\r\n");
                break;
            case 4:
                torque_level = 10.0f;
                printf("\r\n🚀🚀 토크 10.0V - 초최대 출력! 🚀🚀\r\n");
                break;
        }
        torque_step = (torque_step + 1) % 5;
        torque_change_time = HAL_GetTick();
    }

    // 3. 직접 전압 인가 (전류 제어 완전 우회)
    float vd = 0.0f;
    float vq = torque_started ? torque_level : 2.0f;  // 시작은 2V

    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);

    float v_alpha = vd * cos_theta - vq * sin_theta;
    float v_beta = vd * sin_theta + vq * cos_theta;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    // 안전 제한
    if(duty_a > 0.95f) duty_a = 0.95f;
    if(duty_a < 0.05f) duty_a = 0.05f;
    if(duty_b > 0.95f) duty_b = 0.95f;
    if(duty_b < 0.05f) duty_b = 0.05f;
    if(duty_c > 0.95f) duty_c = 0.95f;
    if(duty_c < 0.05f) duty_c = 0.05f;

    set_pwm_duty(duty_b, duty_a, duty_c);

    // 4. ★★★ 회전 감지 ★★★
    static uint16_t prev_raw_angle = 0;
    static uint32_t last_angle_time = 0;
    static int rotation_count = 0;
    static uint8_t angle_init = 0;

    if(!angle_init) {
        prev_raw_angle = raw_angle;
        last_angle_time = HAL_GetTick();
        angle_init = 1;
    } else {
        // 100ms마다 회전 확인
        if(HAL_GetTick() - last_angle_time > 100) {
            int16_t angle_diff = (int16_t)raw_angle - (int16_t)prev_raw_angle;

            // 4096 경계 처리
            if(angle_diff > 2048) angle_diff -= 4096;
            else if(angle_diff < -2048) angle_diff += 4096;

            if(abs(angle_diff) > 10) {  // 10 이상 변하면 회전으로 간주
                rotation_count++;
            }

            prev_raw_angle = raw_angle;
            last_angle_time = HAL_GetTick();
        }
    }

    // 5. 전류 측정 (참고용)
    Read_Current_Sensors();

    // 6. 상태 출력 (매우 자세히)
    static uint32_t last_print = 0;
    if(HAL_GetTick() - last_print > 800) {
        printf("토크:%.1fV | Raw:%d | 각도:%.1f° | 전류: Ia=%.3f, Ib=%.3f | Duty: %.3f\r\n",
               vq, raw_angle, electrical_angle * 180.0f / PI, current_a, current_b, duty_a);

        // 회전 상태 분석
        if(rotation_count > 3) {
            printf("🎉🎉 회전 감지됨! 카운트: %d 🎉🎉\r\n", rotation_count);
            rotation_count = 0;  // 리셋
        } else if(fabsf(current_a - 1.65f) > 0.1f || fabsf(current_b - 1.65f) > 0.1f) {
            printf("⚡ 전류 흐름 감지 - 토크 생성 중!\r\n");
        } else {
            printf("⚠️ 전류 부족 - 토크 더 증가 예정\r\n");
        }

        // 토크 수준별 기대 효과
        if(vq > 8.0f) {
            printf(">>> 🚀 최대 토크! 반드시 회전해야 함! <<<\r\n");
        } else if(vq > 5.0f) {
            printf(">>> ⚡ 높은 토크! 회전 가능성 높음! <<<\r\n");
        } else {
            printf(">>> 🔧 토크 증가 중... <<<\r\n");
        }

        last_print = HAL_GetTick();
    }
}
#endif

#if 0
// 높은 전류 + 속도 기반 가감속 제어
void FOC_control(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 1. 전류 센서 캘리브레이션 (검증됨)
    static float ia_offset = 0, ib_offset = 0;
    static uint8_t offset_calibrated = 0;
    static int offset_count = 0;
    static float ia_sum = 0, ib_sum = 0;

    if(!offset_calibrated && offset_count < 200) {  // 빠르게
        Read_Current_Sensors();
        ia_sum += current_a;
        ib_sum += current_b;
        offset_count++;

        if(offset_count >= 200) {
            ia_offset = ia_sum / 200.0f;
            ib_offset = ib_sum / 200.0f;
            offset_calibrated = 1;
            printf("🔧 캘리브레이션 완료 - 고전류 제어 시작!\r\n");
        } else {
            set_pwm_duty(0.5f, 0.5f, 0.5f);
            return;
        }
    }

    // 2. 각도 처리 (속도 측정을 위해 개선)
    uint16_t raw_angle = AS5600_ReadRawAngle();
    float raw_mechanical_angle = ((float)raw_angle) * TWO_PI / 4096.0f;
    float raw_mechanical = normalize_angle(raw_mechanical_angle - electrical_offset);

    static float smooth_mechanical = 0;
    static uint8_t first_run = 1;

    if(first_run) {
        smooth_mechanical = raw_mechanical;
        first_run = 0;
    } else {
        float angle_diff = raw_mechanical - smooth_mechanical;
        while(angle_diff > PI) angle_diff -= TWO_PI;
        while(angle_diff < -PI) angle_diff += TWO_PI;

        if(angle_diff > 0.05f) angle_diff = 0.05f;  // 0.02f → 0.05f (더 빠른 응답)
        else if(angle_diff < -0.05f) angle_diff = -0.05f;

        smooth_mechanical += angle_diff * 0.7f;  // 0.5f → 0.7f (더 빠른 응답)
    }

    mechanical_angle = smooth_mechanical;
    while(mechanical_angle > TWO_PI) mechanical_angle -= TWO_PI;
    while(mechanical_angle < 0) mechanical_angle += TWO_PI;

    // ★★★ 전기각 방향 확인 테스트 ★★★
    static uint8_t direction_test = 0;
    if(!direction_test) {
        electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);  // 원래 방향
        direction_test = 1;
    } else {
        electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);
        // 만약 회전이 반대라면 이것 시도:
        // electrical_angle = fmodf((-mechanical_angle) * POLE_PAIRS, TWO_PI);
        // if(electrical_angle < 0) electrical_angle += TWO_PI;
    }

    // 3. ★★★ 속도 측정 추가 ★★★
    static float prev_mechanical_angle = 0;
    static uint32_t prev_speed_time = 0;
    static float current_speed = 0;
    static uint8_t speed_init = 0;
    static float angle_accumulator = 0;

    uint32_t current_time = HAL_GetTick();

    if(!speed_init) {
        prev_mechanical_angle = mechanical_angle;
        prev_speed_time = current_time;
        speed_init = 1;
        angle_accumulator = 0;
    } else {
        // 150ms마다 속도 계산
        if(current_time - prev_speed_time >= 150) {
            float angle_change = mechanical_angle - prev_mechanical_angle;

            if(angle_change > PI) angle_change -= TWO_PI;
            else if(angle_change < -PI) angle_change += TWO_PI;

            angle_accumulator += angle_change;

            float time_diff = (current_time - prev_speed_time) / 1000.0f;
            if(time_diff > 0) {
                float new_speed = fabsf(angle_accumulator) / time_diff;  // 속도 크기
                current_speed = 0.6f * current_speed + 0.4f * new_speed;  // 필터링
            }

            prev_mechanical_angle = mechanical_angle;
            prev_speed_time = current_time;
            angle_accumulator = 0;
        } else {
            float angle_change = mechanical_angle - prev_mechanical_angle;
            if(angle_change > PI) angle_change -= TWO_PI;
            else if(angle_change < -PI) angle_change += TWO_PI;

            angle_accumulator += angle_change;
            prev_mechanical_angle = mechanical_angle;
        }
    }

    // 4. 전류 측정 (필터링 완화)
    Read_Current_Sensors();

    float ia_corrected = current_a - ia_offset;
    float ib_corrected = current_b - ib_offset;

    static float ia_filtered = 0, ib_filtered = 0;
    ia_filtered = 0.7f * ia_filtered + 0.3f * ia_corrected;  // 0.85f → 0.7f (더 빠른 응답)
    ib_filtered = 0.7f * ib_filtered + 0.3f * ib_corrected;

    float i_alpha = ia_filtered;
    float i_beta = (ia_filtered + 2.0f * ib_filtered) / 1.732f;

    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);
    float id_measured = i_alpha * cos_theta + i_beta * sin_theta;
    float iq_measured = -i_alpha * sin_theta + i_beta * cos_theta;

    // 5. ★★★ 속도 기반 가감속 제어 ★★★
    static float target_speed = 0.0f;  // 목표 속도 (rad/s)
    static uint32_t speed_change_time = 0;
    static uint8_t speed_control_started = 0;
    static uint32_t start_time = 0;
    static int speed_step = 0;

    if(start_time == 0) start_time = HAL_GetTick();

    // 3초 후 속도 제어 시작
    if(!speed_control_started && (HAL_GetTick() - start_time > 3000)) {
        speed_control_started = 1;
        speed_change_time = HAL_GetTick();
        target_speed = 1.0f;  // 첫 목표
        printf("\r\n🚀🚀 속도 기반 가감속 제어 시작! 🚀🚀\r\n");
    }

    // 6초마다 목표 속도 변경 (극명한 차이)
    if(speed_control_started && (HAL_GetTick() - speed_change_time > 6000)) {
        switch(speed_step) {
            case 0:
                target_speed = 5.0f;
                printf("\r\n🚀 고속: 5.0 rad/s - 빠른 회전!\r\n");
                break;
            case 1:
                target_speed = 0.2f;
                printf("\r\n🐌 저속: 0.2 rad/s - 천천히!\r\n");
                break;
            case 2:
                target_speed = 8.0f;
                printf("\r\n⚡ 최고속: 8.0 rad/s - 최대한 빠르게!\r\n");
                break;
            case 3:
                target_speed = 1.0f;
                printf("\r\n🚶 중간속: 1.0 rad/s - 보통!\r\n");
                break;
        }
        speed_step = (speed_step + 1) % 4;
        speed_change_time = HAL_GetTick();
    }

    // 6. ★★★ 속도 PI 제어기 → 전류 지령 ★★★
    static float speed_integral = 0;
    float speed_error = target_speed - current_speed;

    if(speed_control_started && fabsf(speed_error) < 5.0f) {
        speed_integral += speed_error * 0.005f;  // 5ms
    } else {
        speed_integral *= 0.95f;
    }

    // 적분 제한
    if(speed_integral > 3.0f) speed_integral = 3.0f;
    if(speed_integral < -3.0f) speed_integral = -3.0f;

    // 속도 제어 게인
    float speed_kp = 1.5f;   // 비례 게인
    float speed_ki = 2.0f;   // 적분 게인

    // 속도 제어기 출력 = 토크 지령
    float iq_ref_from_speed = speed_kp * speed_error + speed_ki * speed_integral;

    // 베이스 토크 + 속도 비례 토크
    float base_torque = 1.0f + target_speed * 0.3f;  // 더 높은 베이스 토크
    float total_iq_ref = base_torque + iq_ref_from_speed;

    // ★★★ 높은 토크 제한 ★★★
    if(total_iq_ref > 6.0f) total_iq_ref = 6.0f;   // 1.5f → 6.0f (4배 증가!)
    if(total_iq_ref < 0.2f) total_iq_ref = 0.2f;

    float id_ref = 0.0f;
    float iq_ref = speed_control_started ? total_iq_ref : 2.0f;  // 초기값도 증가

    // 7. 전류 제어 (게인 조정)
    static float id_integral = 0.0f;
    static float iq_integral = 0.0f;

    float id_error = id_ref - id_measured;
    float iq_error = iq_ref - iq_measured;

    float dt = 0.005f;

    if(fabsf(id_error) < 1.0f) {
        id_integral += id_error * dt;
    } else {
        id_integral *= 0.95f;
    }

    if(fabsf(iq_error) < 1.0f) {
        iq_integral += iq_error * dt;
    } else {
        iq_integral *= 0.95f;
    }

    if(id_integral > 1.0f) id_integral = 1.0f;
    if(id_integral < -1.0f) id_integral = -1.0f;
    if(iq_integral > 1.0f) iq_integral = 1.0f;
    if(iq_integral < -1.0f) iq_integral = -1.0f;

    float kp = 1.0f;   // 0.8f → 1.0f (더 강한 응답)
    float ki = 2.0f;   // 1.5f → 2.0f

    float vd_command = kp * id_error + ki * id_integral;
    float vq_command = kp * iq_error + ki * iq_integral;

    // 전압 제한 증가
    if(vd_command > 8.0f) vd_command = 8.0f;   // 5.0f → 8.0f
    if(vd_command < -8.0f) vd_command = -8.0f;
    if(vq_command > 8.0f) vq_command = 8.0f;
    if(vq_command < -8.0f) vq_command = -8.0f;

    static float vd_smooth = 0, vq_smooth = 0;
    vd_smooth = 0.7f * vd_smooth + 0.3f * vd_command;  // 필터링 완화
    vq_smooth = 0.7f * vq_smooth + 0.3f * vq_command;

    // 8. 역변환 및 PWM
    float v_alpha = vd_smooth * cos_theta - vq_smooth * sin_theta;
    float v_beta = vd_smooth * sin_theta + vq_smooth * cos_theta;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    set_pwm_duty(duty_b, duty_a, duty_c);

    // 9. ★★★ 가감속 상태 출력 ★★★
    static uint32_t last_print = 0;
    if(HAL_GetTick() - last_print > 1000) {
        if(speed_control_started) {
            printf("🎯목표속도: %.1f rad/s | 📊현재속도: %.2f rad/s | ⚡토크지령: %.1fA\r\n",
                   target_speed, current_speed, total_iq_ref);
            printf("📏실제전류: Iq=%.2fA | 🔋전압: Vq=%.1fV | Raw:%d\r\n",
                   iq_measured, vq_smooth, raw_angle);

            // 속도 제어 상태
            float speed_error_percent = fabsf(speed_error) / (target_speed + 0.1f) * 100.0f;

            if(speed_error_percent < 50.0f && current_speed > 0.1f) {
                printf("✅ 가감속 성공! 속도 변화 확인됨!\r\n");
            } else if(current_speed > 0.05f) {
                printf("🔧 가감속 조정 중... (현재 %.2f rad/s 회전 중)\r\n", current_speed);
            } else {
                printf("⚙️ 시동 중... (토크: %.1fA, 전압: %.1fV)\r\n", total_iq_ref, vq_smooth);
            }

            // 시각적 상태
            if(target_speed > 6.0f) {
                printf(">>> 🚀🚀 최고속! 빠른 회전이 보여야 함! 🚀🚀 <<<\r\n");
            } else if(target_speed < 0.5f) {
                printf(">>> 🐌🐌 저속! 천천히 도는 것이 보여야 함! 🐌🐌 <<<\r\n");
            }
        } else {
            printf("⏳ 고전류 제어 준비 중... %.1f초 후 가감속 시작\r\n",
                   (3000 - (HAL_GetTick() - start_time)) / 1000.0f);
        }
        last_print = HAL_GetTick();
    }
}
#endif

// ========================================
// 1단계: 완전 정지 및 안전 확인
// ========================================
void FOC_control_SAFE_STOP(void){
    // 모터 완전 정지
    motor_disable();
    set_pwm_duty(0.5f, 0.5f, 0.5f);
    
    static uint32_t last_print = 0;
    if(HAL_GetTick() - last_print > 2000) {
        printf("🚨 모터 완전 정지 - 과열 방지 모드\r\n");
        printf("모터 온도가 내려갈 때까지 대기하세요\r\n");
        last_print = HAL_GetTick();
    }
    return;
}

// ========================================
// 2단계: 저전압 진단 테스트
// ========================================
void FOC_control_LOW_VOLTAGE_TEST(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 전류 센서 캘리브레이션
    static float ia_offset = 0, ib_offset = 0;
    static uint8_t offset_calibrated = 0;
    static int offset_count = 0;

    if(!offset_calibrated && offset_count < 100) {
        Read_Current_Sensors();
        ia_offset += current_a;
        ib_offset += current_b;
        offset_count++;
        
        if(offset_count >= 100) {
            ia_offset /= 100.0f;
            ib_offset /= 100.0f;
            offset_calibrated = 1;
            printf("🔧 안전 진단 시작\r\n");
        } else {
            set_pwm_duty(0.5f, 0.5f, 0.5f);
            return;
        }
    }

    // 각도 읽기
    uint16_t raw_angle = AS5600_ReadRawAngle();
    float raw_mechanical_angle = ((float)raw_angle) * TWO_PI / 4096.0f;
    float mechanical_angle = normalize_angle(raw_mechanical_angle - electrical_offset);
    
    // ★★★ 여러 전기각 방법 테스트 ★★★
    static uint8_t test_method = 0;
    static uint32_t method_change_time = 0;
    static uint32_t start_time = 0;
    float electrical_angle;
    
    if(start_time == 0) start_time = HAL_GetTick();
    
    // 5초마다 다른 방법 시도
    if(HAL_GetTick() - method_change_time > 5000) {
        test_method = (test_method + 1) % 4;
        method_change_time = HAL_GetTick();
        
        switch(test_method) {
            case 0:
                printf("\r\n🔬 방법1: 기본 전기각\r\n");
                break;
            case 1:
                printf("\r\n🔬 방법2: 역방향 전기각\r\n");
                break;
            case 2:
                printf("\r\n🔬 방법3: +90도 오프셋\r\n");
                break;
            case 3:
                printf("\r\n🔬 방법4: -90도 오프셋\r\n");
                break;
        }
    }
    
    // 전기각 계산 방법별 테스트
    switch(test_method) {
        case 0:
            electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);
            break;
        case 1:
            electrical_angle = fmodf((-mechanical_angle) * POLE_PAIRS, TWO_PI);
            if(electrical_angle < 0) electrical_angle += TWO_PI;
            break;
        case 2:
            electrical_angle = fmodf(mechanical_angle * POLE_PAIRS + PI/2, TWO_PI);
            break;
        case 3:
            electrical_angle = fmodf(mechanical_angle * POLE_PAIRS - PI/2, TWO_PI);
            if(electrical_angle < 0) electrical_angle += TWO_PI;
            break;
    }

    // 전류 측정
    Read_Current_Sensors();
    float ia_corrected = current_a - ia_offset;
    float ib_corrected = current_b - ib_offset;

    float i_alpha = ia_corrected;
    float i_beta = (ia_corrected + 2.0f * ib_corrected) / 1.732f;

    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);
    float id_measured = i_alpha * cos_theta + i_beta * sin_theta;
    float iq_measured = -i_alpha * sin_theta + i_beta * cos_theta;

    // ★★★ 매우 낮은 전압으로 안전 테스트 ★★★
    float vd = 0.0f;
    float vq = 0.5f;  // 매우 낮은 전압

    float v_alpha = vd * cos_theta - vq * sin_theta;
    float v_beta = vd * sin_theta + vq * cos_theta;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    set_pwm_duty(duty_b, duty_a, duty_c);

    // 진단 출력
    static uint32_t last_print = 0;
    if(HAL_GetTick() - last_print > 1000) {
        printf("방법%d | Raw:%d | 전기각:%.2f | 전류: Ia=%.3f, Ib=%.3f | DQ: %.3f, %.3f\r\n",
               test_method+1, raw_angle, electrical_angle, ia_corrected, ib_corrected, id_measured, iq_measured);
        
        // 회전 상태 확인
        static uint16_t prev_raw = 0;
        static uint8_t prev_init = 0;
        
        if(!prev_init) {
            prev_raw = raw_angle;
            prev_init = 1;
        } else {
            int16_t raw_diff = (int16_t)raw_angle - (int16_t)prev_raw;
            if(raw_diff > 2048) raw_diff -= 4096;
            else if(raw_diff < -2048) raw_diff += 4096;
            
            if(abs(raw_diff) > 5) {
                printf("✅ 회전 감지! 방법%d가 올바름! (변화: %d)\r\n", test_method+1, raw_diff);
            } else {
                printf("❌ 회전 없음 (변화: %d)\r\n", raw_diff);
            }
            
            prev_raw = raw_angle;
        }
        
        // 전류 상태 확인
        float total_current = sqrtf(ia_corrected*ia_corrected + ib_corrected*ib_corrected);
        if(total_current > 1.0f) {
            printf("⚠️ 높은 전류! (%.2fA) - 오정렬 가능성\r\n", total_current);
        } else {
            printf("✅ 안전한 전류 수준 (%.2fA)\r\n", total_current);
        }
        
        last_print = HAL_GetTick();
    }
}

// ========================================
// 3단계: 배선 진단 테스트
// ========================================
void FOC_control_WIRING_TEST(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    static uint32_t test_time = 0;
    static uint8_t phase_test = 0;
    static uint32_t start_time = 0;
    
    if(start_time == 0) start_time = HAL_GetTick();
    
    // 3초마다 다른 상 테스트
    if(HAL_GetTick() - test_time > 3000) {
        phase_test = (phase_test + 1) % 4;
        test_time = HAL_GetTick();
        
        switch(phase_test) {
            case 0:
                printf("\r\n🔌 A상 단독 테스트\r\n");
                set_pwm_duty(0.6f, 0.5f, 0.5f);  // A상만 약간 up
                break;
            case 1:
                printf("\r\n🔌 B상 단독 테스트\r\n");
                set_pwm_duty(0.5f, 0.6f, 0.5f);  // B상만 약간 up
                break;
            case 2:
                printf("\r\n🔌 C상 단독 테스트\r\n");
                set_pwm_duty(0.5f, 0.5f, 0.6f);  // C상만 약간 up
                break;
            case 3:
                printf("\r\n🔌 모든 상 중립\r\n");
                set_pwm_duty(0.5f, 0.5f, 0.5f);  // 모든 상 중립
                break;
        }
    }

    // 전류 측정 및 분석
    Read_Current_Sensors();
    
    static uint32_t last_print = 0;
    if(HAL_GetTick() - last_print > 500) {
        printf("Phase%d | 전류: Ia=%.3f, Ib=%.3f | 온도체크 필요!\r\n",
               phase_test, current_a, current_b);
        
        // 비정상 전류 감지
        if(fabsf(current_a - 1.65f) > 0.3f || fabsf(current_b - 1.65f) > 0.3f) {
            printf("⚠️ 높은 전류 감지 - 즉시 정지 권장!\r\n");
        }
        
        last_print = HAL_GetTick();
    }
}


// 전류 폐루프 제어
#if 0
void FOC_control(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 1. 엔코더 각도 처리 (더 강한 필터링)
    uint16_t raw_angle = AS5600_ReadRawAngle();
    float raw_mechanical_angle = ((float)raw_angle) * TWO_PI / 4096.0f;
    float raw_mechanical = normalize_angle(raw_mechanical_angle - electrical_offset);

    printf("electrical_offset: %.3f\r\n",electrical_offset);
    //raw_mechanical -= 0.32f;

    static float smooth_mechanical = 0;
    static uint8_t first_run = 1;

    if(first_run) {
        smooth_mechanical = raw_mechanical;
        first_run = 0;
    } else {
        float angle_diff = raw_mechanical - smooth_mechanical;
        while(angle_diff > PI) angle_diff -= TWO_PI;
        while(angle_diff < -PI) angle_diff += TWO_PI;

        // 각도 변화 제한 (더 강하게)
        if(angle_diff > 0.05f) angle_diff = 0.05f;       // 변화량 제한
        else if(angle_diff < -0.05f) angle_diff = -0.05f;

        smooth_mechanical += angle_diff * 1.0f;  // 0.1f → 0.3f (조금 더 빠른 응답)        0.3f
    }

    mechanical_angle = smooth_mechanical;
    while(mechanical_angle > TWO_PI) mechanical_angle -= TWO_PI;
    while(mechanical_angle < 0) mechanical_angle += TWO_PI;
    electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);

    // 2. 전류 측정 및 필터링 추가
    Read_Current_Sensors();

    // 전류 필터링 (노이즈 감소)
    static float ia_filtered = 0, ib_filtered = 0;
    ia_filtered = 0.7f * current_a + 0.3f * ia_filtered;
    ib_filtered = 0.7f * current_b + 0.3f * ib_filtered;

    // Clarke 변환
    float i_alpha = ia_filtered;
    float i_beta = (ia_filtered + 2.0f * ib_filtered) / 1.732f;

    // Park 변환
    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);
    float id_measured = i_alpha * cos_theta + i_beta * sin_theta;
    float iq_measured = -i_alpha * sin_theta + i_beta * cos_theta;

    // 3. 전류 지령값
    float id_ref = 0.0f;
    float iq_ref = 5.0f;    // 0.3f → 0.25f로 조금 감소

    // 4. PID 제어 (게인 조정)
    static float id_integral = 0.0f;
    static float iq_integral = 0.0f;

    float id_error = id_ref - id_measured;
    float iq_error = iq_ref - iq_measured;

    float dt = 0.005f;
    id_integral += id_error * dt;
    iq_integral += iq_error * dt;

    // 적분 와인드업 방지
    if(id_integral > 5.0f) id_integral = 5.0f;  // 0.5f → 0.3f   0.3
    if(id_integral < -5.0f) id_integral = -5.0f;
    if(iq_integral > 5.0f) iq_integral = 5.0f;
    if(iq_integral < -5.0f) iq_integral = -5.0f;

    // PID 게인 (조금 감소)
    float kp = 0.2f;   // 1.0f → 0.8f
    float ki = 0.5f;   // 10.0f → 8.0f
    //float ki = 0.2f;

    float vd_command = kp * id_error + ki * id_integral;
    float vq_command = kp * iq_error + ki * iq_integral;

    // 전압 제한
    if(vd_command > 12.0f) vd_command = 12.0f;  // 3.0f → 2.5f             12V
    if(vd_command < -12.0f) vd_command = -12.0f;
    if(vq_command > 12.0f) vq_command = 12.0f;
    if(vq_command < -12.0f) vq_command = -12.0f;

    // 5. 역 Park 변환 및 PWM 출력
    float v_alpha = vd_command * cos_theta - vq_command * sin_theta;
    float v_beta = vd_command * sin_theta + vq_command * cos_theta;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    set_pwm_duty(duty_b, duty_a, duty_c);

    //printf("Iq_ref: %.2f, Iq_meas: %.3f, Iq_err: %.3f, Vq: %.2f, Raw: %d\r\n",
           //iq_ref, iq_measured, iq_error, vq_command, raw_angle);
}
#endif

// 속도 폐루프 제어 (수정된 버전)
// 단순하고 안정적인 속도 제어
// 단순하고 안정적인 속도 제어
// 단순하고 안정적인 속도 제어
// 실제 호출 주기를 측정하여 정확한 속도 제어
// 실제 호출 주기를 측정하여 정확한 속도 제어
#if 0
void FOC_control(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 실제 호출 주기 측정
    static uint32_t last_call_time = 0;
    static float actual_dt = 0.001f;  // 기본값
    static uint8_t dt_calibrated = 0;
    uint32_t current_time;
    float measured_dt;

    current_time = HAL_GetTick();
    if(last_call_time != 0) {
        measured_dt = (current_time - last_call_time) / 1000.0f;
        if(measured_dt > 0.0001f && measured_dt < 1.0f) {  // 유효한 범위
            if(!dt_calibrated) {
                actual_dt = measured_dt;
                dt_calibrated = 1;
                printf("Detected call period: %.3f ms (%.1f Hz)\r\n",
                       measured_dt * 1000.0f, 1.0f / measured_dt);
            } else {
                // 부드럽게 업데이트
                actual_dt = 0.95f * actual_dt + 0.05f * measured_dt;
            }
        }
    }
    last_call_time = current_time;

    // 1. 각도 계산
    static float virtual_angle = 0;
    static float angle_increment = 0.00005f;  // 초기값 (나중에 조정됨)
    static uint8_t increment_initialized = 0;

    // 첫 번째 호출 시 angle_increment 초기화
    if(!increment_initialized && dt_calibrated) {
        angle_increment = 0.05f * actual_dt;
        increment_initialized = 1;
    }

    // 가상 각도 업데이트
    virtual_angle += angle_increment;
    if(virtual_angle > TWO_PI) virtual_angle -= TWO_PI;

    electrical_angle = fmodf(virtual_angle * POLE_PAIRS, TWO_PI);

    // 2. 속도 제어
    static float speed_ref = 1.0f;
    static uint32_t speed_change_time = 0;
    static uint8_t control_started = 0;
    static uint32_t start_time = 0;
    uint32_t elapsed;
    static int speed_step = 0;

    // 5초 후 제어 시작
    if(start_time == 0) start_time = HAL_GetTick();

    if(!control_started && (HAL_GetTick() - start_time > 5000)) {
        control_started = 1;
        speed_change_time = HAL_GetTick();
        printf("Speed control started! (dt=%.3fms)\r\n", actual_dt * 1000.0f);
    }

    // 속도 변경 테스트 (8초마다, 극명한 차이)
    if(control_started) {
        elapsed = HAL_GetTick() - speed_change_time;
        if(elapsed > 8000) {  // 8초마다
            switch(speed_step) {
                case 0: speed_ref = 8.0f; break;   // 매우 빠름
                case 1: speed_ref = 0.3f; break;   // 매우 느림
                case 2: speed_ref = 12.0f; break;  // 최고 속도
                case 3: speed_ref = 1.0f; break;   // 중간 속도
                case 4: speed_ref = 0.1f; break;   // 거의 정지
            }
            speed_step = (speed_step + 1) % 5;
            speed_change_time = HAL_GetTick();
            printf("\r\n========== SPEED CHANGE ==========\r\n");
            printf("*** TARGET: %.1f rad/s ***\r\n", speed_ref);
            if(speed_ref > 8.0f) printf(">>> MAXIMUM SPEED <<<\r\n");
            else if(speed_ref < 0.5f) printf(">>> VERY SLOW/STOP <<<\r\n");
            else printf(">>> NORMAL SPEED <<<\r\n");
            printf("================================\r\n");
        }
    }

    float vq = 1.0f;

    if(control_started && dt_calibrated) {
        // 실제 dt 기반 목표 증분값 계산
        float target_increment;
        float current_speed;
        float speed_error;
        static float speed_integral = 0;
        float kp, ki;
        float control_output;
        float alpha;
        float max_increment, min_increment;
        static uint32_t last_detail_time = 0;

        target_increment = speed_ref * actual_dt;

        // 현재 속도 추정 (실제 dt 기반)
        current_speed = angle_increment / actual_dt;
        speed_error = speed_ref - current_speed;

        // 강력한 PI 제어
        speed_integral += speed_error * actual_dt;

        // 적분 제한
        if(speed_integral > 3.0f) speed_integral = 3.0f;
        if(speed_integral < -3.0f) speed_integral = -3.0f;

        // 큰 게인으로 빠른 응답
        kp = 5.0f;   // 비례 게인 더 크게
        ki = 10.0f;  // 적분 게인 더 크게

        control_output = kp * speed_error + ki * speed_integral;

        // 각도 증분값을 직접 목표값으로 수렴
        alpha = 0.3f;  // 더 빠른 수렴
        angle_increment = (1.0f - alpha) * angle_increment + alpha * target_increment;

        // 제어 출력 추가 적용
        angle_increment += control_output * actual_dt * 0.1f;

        // 증분값 제한 (실제 dt 기반)
        max_increment = 15.0f * actual_dt;  // 최대 15 rad/s
        min_increment = 0.05f * actual_dt;  // 최소 0.05 rad/s

        if(angle_increment > max_increment) angle_increment = max_increment;
        if(angle_increment < min_increment) angle_increment = min_increment;

        // 토크 조정 (속도와 오차에 비례)
        vq = 1.0f + fabsf(speed_error) * 0.5f + fabsf(speed_ref) * 0.2f;
        if(vq > 5.0f) vq = 5.0f;
        if(vq < 0.5f) vq = 0.5f;

        // 1초마다 상세 정보 출력
        if(current_time - last_detail_time > 1000) {
            printf("DT:%.2fms | Target:%.1f | Current:%.2f | Error:%.2f | Inc:%.5f | Vq:%.2f\r\n",
                   actual_dt * 1000.0f, speed_ref, current_speed, speed_error, angle_increment, vq);
            last_detail_time = current_time;
        }

    } else {
        printf("Initializing... (dt=%.3fms)\r\n", actual_dt * 1000.0f);
    }

    // 3. FOC 제어
    float vd = 0.0f;

    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);

    float v_alpha = vd * cos_theta - vq * sin_theta;
    float v_beta = vd * sin_theta + vq * cos_theta;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    // 듀티 사이클 제한
    if(duty_a > 0.9f) duty_a = 0.9f;
    if(duty_a < 0.1f) duty_a = 0.1f;
    if(duty_b > 0.9f) duty_b = 0.9f;
    if(duty_b < 0.1f) duty_b = 0.1f;
    if(duty_c > 0.9f) duty_c = 0.9f;
    if(duty_c < 0.1f) duty_c = 0.1f;

    set_pwm_duty(duty_a, duty_b, duty_c);
}
#endif
#if 0
// 가상 각도
// 토크 리플 최소화 FOC 제어
void FOC_control(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 실제 호출 주기 측정
    static uint32_t last_call_time = 0;
    static float actual_dt = 0.001f;
    static uint8_t dt_calibrated = 0;
    uint32_t current_time;
    float measured_dt;

    current_time = HAL_GetTick();
    if(last_call_time != 0) {
        measured_dt = (current_time - last_call_time) / 1000.0f;
        if(measured_dt > 0.0001f && measured_dt < 1.0f) {
            if(!dt_calibrated) {
                actual_dt = measured_dt;
                dt_calibrated = 1;
                printf("Detected call period: %.3f ms (%.1f Hz)\r\n",
                       measured_dt * 1000.0f, 1.0f / measured_dt);
            } else {
                actual_dt = 0.98f * actual_dt + 0.02f * measured_dt;  // 더 부드러운 업데이트
            }
        }
    }
    last_call_time = current_time;

    // 1. 부드러운 각도 계산
    static float virtual_angle = 0;
    static float angle_increment = 0.00005f;
    static uint8_t increment_initialized = 0;
    static float angle_velocity = 0;  // 각속도 (부드러운 변화를 위해)

    if(!increment_initialized && dt_calibrated) {
        angle_increment = 0.05f * actual_dt;
        angle_velocity = 0.05f;  // 초기 각속도 (rad/s)
        increment_initialized = 1;
    }

    // 부드러운 각도 업데이트
    virtual_angle += angle_increment;
    if(virtual_angle > TWO_PI) virtual_angle -= TWO_PI;
    if(virtual_angle < 0) virtual_angle += TWO_PI;

    // 전기각 계산 (더 부드럽게)
    electrical_angle = fmodf(virtual_angle * POLE_PAIRS, TWO_PI);

    // 2. 속도 제어
    static float speed_ref = 1.0f;
    static uint32_t speed_change_time = 0;
    static uint8_t control_started = 0;
    static uint32_t start_time = 0;
    uint32_t elapsed;
    static int speed_step = 0;

    if(start_time == 0) start_time = HAL_GetTick();

    if(!control_started && (HAL_GetTick() - start_time > 5000)) {
        control_started = 1;
        speed_change_time = HAL_GetTick();
        printf("Smooth speed control started!\r\n");
    }

    // 속도 변경 (더 부드럽게)
    if(control_started) {
        elapsed = HAL_GetTick() - speed_change_time;
        if(elapsed > 10000) {  // 10초마다
            switch(speed_step) {
                case 0: speed_ref = 6.0f; break;
                case 1: speed_ref = 0.5f; break;
                case 2: speed_ref = 10.0f; break;
                case 3: speed_ref = 2.0f; break;
                case 4: speed_ref = 0.2f; break;
            }
            speed_step = (speed_step + 1) % 5;
            speed_change_time = HAL_GetTick();
            printf("\r\n=== SMOOTH SPEED CHANGE TO %.1f rad/s ===\r\n", speed_ref);
        }
    }

    float vq = 1.0f;

    if(control_started && dt_calibrated) {
        // 부드러운 속도 제어
        static float speed_integral = 0;
        static float filtered_speed_error = 0;
        float target_velocity;
        float current_velocity;
        float speed_error;
        float kp, ki, kd;
        float control_output;
        static float prev_speed_error = 0;
        float speed_derivative;
        static uint32_t last_detail_time = 0;

        target_velocity = speed_ref;
        current_velocity = angle_velocity;  // 부드러운 각속도 사용

        speed_error = target_velocity - current_velocity;

        // 오차 필터링 (노이즈 제거)
        filtered_speed_error = 0.7f * filtered_speed_error + 0.3f * speed_error;

        // PID 제어 (미분항 추가로 더 안정적)
        speed_integral += filtered_speed_error * actual_dt;

        // 적분 와인드업 방지
        if(speed_integral > 2.0f) speed_integral = 2.0f;
        if(speed_integral < -2.0f) speed_integral = -2.0f;

        // 미분 계산
        speed_derivative = (filtered_speed_error - prev_speed_error) / actual_dt;
        prev_speed_error = filtered_speed_error;

        // 조정된 PID 게인 (더 부드럽게)
        kp = 2.0f;   // 비례 게인 적당히
        ki = 3.0f;   // 적분 게인
        kd = 0.1f;   // 미분 게인 (진동 억제)

        control_output = kp * filtered_speed_error + ki * speed_integral + kd * speed_derivative;

        // 각속도를 부드럽게 업데이트
        float velocity_change_rate = 5.0f;  // rad/s² (가속도 제한)
        float max_velocity_change = velocity_change_rate * actual_dt;

        float desired_velocity = target_velocity + control_output * 0.1f;
        float velocity_diff = desired_velocity - angle_velocity;

        // 가속도 제한 적용
        if(velocity_diff > max_velocity_change) velocity_diff = max_velocity_change;
        if(velocity_diff < -max_velocity_change) velocity_diff = -max_velocity_change;

        angle_velocity += velocity_diff;

        // 속도 제한
        if(angle_velocity > 15.0f) angle_velocity = 15.0f;
        if(angle_velocity < 0.1f) angle_velocity = 0.1f;

        // 각도 증분값 업데이트
        angle_increment = angle_velocity * actual_dt;

        // 부드러운 토크 조정
        static float filtered_vq = 1.0f;
        float target_vq = 1.0f + fabsf(filtered_speed_error) * 0.3f + fabsf(target_velocity) * 0.15f;

        if(target_vq > 3.0f) target_vq = 3.0f;
        if(target_vq < 0.5f) target_vq = 0.5f;

        // 토크도 필터링하여 부드럽게
        filtered_vq = 0.9f * filtered_vq + 0.1f * target_vq;
        vq = filtered_vq;

        // 1초마다 상세 정보
        if(current_time - last_detail_time > 1000) {
            printf("Target:%.1f | Current:%.2f | Error:%.2f | Velocity:%.3f | Vq:%.2f\r\n",
                   target_velocity, current_velocity, filtered_speed_error, angle_velocity, vq);
            last_detail_time = current_time;
        }

    } else {
        printf("Initializing smooth control...\r\n");
    }

    // 3. 부드러운 FOC 제어
    float vd = 0.0f;

    // 삼각함수 계산 (더 정밀하게)
    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);

    // 클라크 변환
    float v_alpha = vd * cos_theta - vq * sin_theta;
    float v_beta = vd * sin_theta + vq * cos_theta;

    // 인버스 클라크 변환 (정확한 계수 사용)
    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866025403f * v_beta;  // sqrt(3)/2 = 0.866025403
    float vc = -0.5f * v_alpha - 0.866025403f * v_beta;

    // 전압을 듀티로 변환 (여유있는 제한)
    float duty_a = 0.5f + va / 13.0f;  // 12V → 13V로 여유있게
    float duty_b = 0.5f + vb / 13.0f;
    float duty_c = 0.5f + vc / 13.0f;

    // 듀티 사이클 제한 (더 넓은 범위)
    if(duty_a > 0.95f) duty_a = 0.95f;
    if(duty_a < 0.05f) duty_a = 0.05f;
    if(duty_b > 0.95f) duty_b = 0.95f;
    if(duty_b < 0.05f) duty_b = 0.05f;
    if(duty_c > 0.95f) duty_c = 0.95f;
    if(duty_c < 0.05f) duty_c = 0.05f;

    set_pwm_duty(duty_a, duty_b, duty_c);
}
#endif

#if 0
void FOC_control(void){
	static uint32_t last_time = 0;
	uint32_t current_time = HAL_GetTick();
	float dt = (current_time - last_time) / 1000.0f;

	if(dt < 0.001f) return;    // 1KHz 제어 주파수로 증가

	if(!motor_enabled){
		set_pwm_duty(0.5f, 0.5f, 0.5f);
		last_time = current_time;
		return;
	}

	// 각도 읽기 변환
	uint16_t raw_angle = AS5600_ReadRawAngle();
	mechanical_angle = (float)raw_angle * TWO_PI / 4096.0f;
	electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);
	if(electrical_angle < 0) electrical_angle += TWO_PI;

	// 각도 변환 개선 (연속성 보장)
	//static float prev_raw_angle = 0;
	//static float accumulated_angle = 0;
	//static uint8_t first_angle_read = 1;

	//if(first_angle_read){
		//prev_raw_angle = raw_angle;
		//accumulated_angle = 0;
		//first_angle_read = 0;
	//}

	//float angle_diff = raw_angle - prev_raw_angle;
	//if(angle_diff > 2048) angle_diff -= 4096;      // 큰 양수 차이 -> 음수로 보정
	//else if(angle_diff < -2048) angle_diff += 4096; // 큰 음수 차이 -> 양수로 보정

	//accumulated_angle += angle_diff;  // 이 줄이 빠져있었습니다!
	//prev_raw_angle = raw_angle;       // 이 줄도 빠져있었습니다!

	//mechanical_angle = accumulated_angle * TWO_PI / 4096.0f;;
	//electrical_angle = mechanical_angle * POLE_PAIRS;
	//electrical_angle = normalize_angle(electrical_angle);

	// 속도 계산
	static float prev_mechanical_angle = 0.0f;
	static uint8_t first_run = 1;
	//static float filtered_velocity = 0.0f;

	if(first_run){
		prev_mechanical_angle = mechanical_angle;
		first_run = 0;
		current_velocity = 0.0f;
	}
	else{
		float angle_diff = mechanical_angle - prev_mechanical_angle;

		// 2π 경계에서 점프 보정
		if(angle_diff > PI) angle_diff -= TWO_PI;
		else if(angle_diff < -PI) angle_diff += TWO_PI;

		float raw_velocity = angle_diff / dt;

		//float vel_alpha = 0.15f; // 필터 상수 조정
		//filtered_velocity = vel_alpha * raw_velocity + (1.0f - vel_alpha) * filtered_velocity;
		//current_velocity = filtered_velocity;

		current_velocity = 0.1f * raw_velocity + 0.9f * current_velocity;
		prev_mechanical_angle = mechanical_angle;
	}

	// 전류 읽기 및 변환
	Read_Current_Sensors();

	// 전류 센서 오프셋 보정 추가
#if 0
	static float ia_offset = 0.0f;
	static float ib_offset = 0.0f;
	static float ic_offset = 0.0f;
	static uint8_t calibration_done = 0;
	static uint16_t calib_count = 0;

	if (!calibration_done){
		if (calib_count < 500) {
			ia_offset += current_a;
			ib_offset += current_b;
			ic_offset += current_c;
			calib_count++;
			set_pwm_duty(0.5f, 0.5f, 0.5f);
			last_time = current_time;

			if (calib_count % 100 == 0) {
				printf("Calibration progress: %d/500\n", calib_count);
			}
			return;
		}
		else{
			ia_offset /= 500.0f;  // 1000.0f → 500.0f로 변경
			ib_offset /= 500.0f;
			ic_offset /= 500.0f;
			calibration_done = 1;
			printf("Calibration done - Ia_offset:%.3f Ib_offset:%.3f Ic_offset:%.3f\n",
			                   ia_offset, ib_offset, ic_offset);
			//printf("Motor control starting...\n");  // 시작 메시지 추가
		}
	}
#endif

	// 오프셋 보정된 전류
	float ia_corrected = current_a;
	float ib_corrected = current_b;
	float ic_corrected = current_c;

	printf("Raw currents: ia=%.3f, ib=%.3f, ic=%.3f\r\n", ia_corrected, ib_corrected, ic_corrected);

	// 전류 필터링 추가
	//static float ia_filtered = 0.0f;
	//static float ib_filtered = 0.0f;
	//static float ic_filtered = 0.0f;

	//float current_filter_alpha = 0.3f; // 필터 강도 (0.1 = 강한 필터, 0.5 = 약한 필터)
	//ia_filtered = current_filter_alpha * ia_corrected + (1.0f - current_filter_alpha) * ia_filtered;
	//ib_filtered = current_filter_alpha * ib_corrected + (1.0f - current_filter_alpha) * ib_filtered;
	//ic_filtered = current_filter_alpha * ic_corrected + (1.0f - current_filter_alpha) * ic_filtered;

	i_ab = clarke_transform(ia_corrected, ib_corrected, ic_corrected);
	i_dq = park_transform(i_ab, electrical_angle);

	// 속도 제어 루프 (외부 루프) - 게인도 조정
	//static float velocity_integral = 0.0f;
	//float velocity_error = target_velocity - current_velocity;

	//float vel_kp = 0.15f;  // 0.15f → 0.3f로 증가
	//float vel_ki = 0.08f; 	// 0.08f → 0.15f로 증가

	//velocity_integral += velocity_error * dt;

	// 적분 윈드업 방지
	//if (velocity_integral > 3.0f) velocity_integral = 3.0f;
	//if (velocity_integral < -3.0f) velocity_integral = -3.0f;

	// 속도 제어기 출력 = Iq 지령값 (토크 지령)
	float target_iq = 0.2;
	float target_id = 0.0f;

	// 시동 보조 토크 추가 (더 강하게)
	//if (fabsf(current_velocity) < 0.5f && fabsf(target_velocity) > 0.1f) {
		//float startup_boost = (target_velocity > 0) ? 1.8f : -1.8f;  // 1.0f → 1.8f로 대폭 증가
	    //target_iq += startup_boost;
	//}

	// Iq 제한 (토크 제한) - 시동을 위해 증가
	//if (target_iq > 2.5f) target_iq = 2.5f;  // 1.5f → 2.5f로 증가
	//if (target_iq < -2.5f) target_iq = -2.5f;

	// Id 지령값 (자속 제어, 일반적으로 0)
	//float target_id = 0.0f;

	// 전류 제어 루프 (내부 루프) - 게인 대폭 감소
	static float id_integral = 0.0f;
	static float iq_integral = 0.0f;

	float id_error = target_id - i_dq.d;
	float iq_error = target_iq - i_dq.q;

	//float curr_kp = 0.08f; // 0.08f → 0.15f로 증가
	//float curr_ki = 0.8f;  // 0.8f → 1.5f로 증가

	// 전류 적분기
	//float dt = 0.005f;    // 5ms
	id_integral += id_error * dt;
	iq_integral += iq_error * dt;

	// 전류 적분 windup 방지 - 더 작은 범위로
	if (id_integral > 0.1f) id_integral = 0.1f;   // 1.0f → 0.5f
	if (id_integral < -0.1f) id_integral = -0.1f;
	if (iq_integral > 0.1f) iq_integral = 0.1f;   // 1.0f → 0.5f
	if (iq_integral < -0.0f) iq_integral = -0.1f;

	float curr_kp = 0.5f; // 0.08f → 0.15f로 증가
	float curr_ki = 2.0f;  // 0.8f → 1.5f로 증가

	// 전류 제어기 출력 (dq 전압 지령)
	float vd_command = curr_kp * id_error + curr_ki * id_integral; ;
	float vq_command = curr_kp * iq_error + curr_ki * iq_integral;

	// 전압 제한
	//float v_magnitude = sqrtf(vd_command*vd_command + vq_command*vq_command);
	//float v_max = 10.0f; // 최대 전압 크기

	//if (v_magnitude > v_max) {
		//vd_command = vd_command * v_max / v_magnitude;
	    //vq_command = vq_command * v_max / v_magnitude;
	//}

	if(vd_command > 2.0f) vd_command = 2.0f;   // 3.0f → 8.0f
	if(vd_command < -2.0f) vd_command = -2.0f;
    if(vq_command > 2.0f) vq_command = 2.0f;   // 3.0f → 8.0f
	if(vq_command < -2.0f) vq_command = -2.0f;

	printf("Vd: %.2f, Vq: %.2f, Id: %.3f, Iq: %.3f\r\n",
	           vd_command, vq_command, i_dq.d, i_dq.q);


	// dq → αβ 역변환
	float cos_theta = cosf(electrical_angle);
	float sin_theta = sinf(electrical_angle);

	float v_alpha = vd_command * cos_theta - vq_command * sin_theta;
	float v_beta = vd_command * sin_theta + vq_command * cos_theta;

	// αβ → abc 역변환 (Clarke 역변환)
	float va = v_alpha;
	float vb = -0.5f * v_alpha + 0.866f * v_beta;
	float vc = -0.5f * v_alpha - 0.866f * v_beta;

	// PWM duty cycle 변환 (12V 시스템 가정)
	float duty_a = 0.5f + va / 12.0f;  // ±12V 범위를 0~1로 변환
	float duty_b = 0.5f + vb / 12.0f;
	float duty_c = 0.5f + vc / 12.0f;

	// Duty cycle 제한
	if (duty_a > 0.95f) duty_a = 0.95f;
	if (duty_a < 0.05f) duty_a = 0.05f;
	if (duty_b > 0.95f) duty_b = 0.95f;
	if (duty_b < 0.05f) duty_b = 0.05f;
	if (duty_c > 0.95f) duty_c = 0.95f;
	if (duty_c < 0.05f) duty_c = 0.05f;

	set_pwm_duty(duty_a, duty_b, duty_c);

	static int debug_counter = 0;
	    if(++debug_counter >= 200) {
	        printf("Target: %.2f, Current: %.2f, MechAngle: %.2f, ElecAngle: %.2f, Iq_target: %.2f\r\n",
	               target_velocity, current_velocity, mechanical_angle, electrical_angle, target_iq);
	        debug_counter = 0;
	}
	last_time = current_time;
}
#endif
#if 0
// 엔코더 진단 및 진짜 폐루프 FOC (1)
void FOC_control(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 호출 주기 측정
    static uint32_t last_call_time = 0;
    static float actual_dt = 0.001f;
    uint32_t current_time = HAL_GetTick();

    if(last_call_time != 0) {
        actual_dt = (current_time - last_call_time) / 1000.0f;
    }
    last_call_time = current_time;

    // 1. 엔코더 원시 데이터 분석 (수정된 버전)
    static uint16_t prev_raw = 0;
    static float prev_angle = 0;
    static uint8_t first_read = 1;
    static uint32_t debug_timer = 0;

    uint16_t raw_angle = AS5600_ReadRawAngle();

    // 각도 계산 수정 (기존 방식 사용)
    float mechanical_angle = (4096.0f - (float)raw_angle) * TWO_PI / 4096.0f - 0.32f;
    while(mechanical_angle > TWO_PI) mechanical_angle -= TWO_PI;
    while(mechanical_angle < 0) mechanical_angle += TWO_PI;

    // 전기각 계산
    electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);

    // 엔코더 변화량 분석 (진단용)
    if(!first_read && (current_time - debug_timer > 200)) {  // 200ms마다 진단
        // Raw 변화량
        int32_t raw_diff = (int32_t)raw_angle - (int32_t)prev_raw;

        // 4096 경계 처리
        if(raw_diff > 2048) raw_diff -= 4096;
        else if(raw_diff < -2048) raw_diff += 4096;

        // 각도 변화량 (단순 차이)
        float angle_diff = mechanical_angle - prev_angle;

        // 2π 경계 처리
        if(angle_diff > PI) angle_diff -= TWO_PI;
        else if(angle_diff < -PI) angle_diff += TWO_PI;

        printf("Raw:%d->%d (diff:%d) Angle:%.3f->%.3f (diff:%.6f) dt:%.3f\r\n",
               prev_raw, raw_angle, raw_diff, prev_angle, mechanical_angle, angle_diff, actual_dt);

        debug_timer = current_time;
        prev_raw = raw_angle;
        prev_angle = mechanical_angle;
    }

    if(first_read) {
        first_read = 0;
        prev_raw = raw_angle;
        prev_angle = mechanical_angle;
        printf("Initial encoder: raw=%d, angle=%.3f\r\n", raw_angle, mechanical_angle);
    }

    // 2. 간단한 속도 측정 (수정된 버전)
    static float measured_speed = 0;
    static uint32_t last_speed_time = 0;
    static float last_speed_angle = 0;
    static uint8_t speed_init = 0;
    static int speed_sample_count = 0;
    static int32_t cumulative_raw_change = 0;  // 누적 Raw 변화량
    static uint16_t last_raw_for_speed = 0;

    // 1초마다 속도 측정 (더 긴 간격)
    if(current_time - last_speed_time > 1000) {
        if(!speed_init) {
            last_speed_angle = mechanical_angle;
            last_raw_for_speed = raw_angle;
            last_speed_time = current_time;
            speed_init = 1;
            printf("Speed measurement initialized\r\n");
        } else {
            float time_diff = (current_time - last_speed_time) / 1000.0f;

            // Raw 기반 속도 측정 (더 정확)
            int32_t raw_change = (int32_t)raw_angle - (int32_t)last_raw_for_speed;

            // 4096 경계 처리
            if(raw_change > 2048) raw_change -= 4096;
            else if(raw_change < -2048) raw_change += 4096;

            // Raw를 라디안으로 변환
            float angle_change_from_raw = ((float)raw_change / 4096.0f) * TWO_PI;
            // 방향 반전 (4096 - raw_angle 때문에)
            angle_change_from_raw = -angle_change_from_raw;

            float raw_based_speed = angle_change_from_raw / time_diff;

            // 속도 검증
            if(fabsf(raw_based_speed) < 15.0f) {  // 15 rad/s 이하만 유효
                measured_speed = raw_based_speed;
                speed_sample_count++;

                printf("Speed calc: raw_change=%d, angle_change=%.6f, time=%.3f, speed=%.3f\r\n",
                       raw_change, angle_change_from_raw, time_diff, raw_based_speed);
            } else {
                printf("Invalid speed: %.3f (raw_change=%d)\r\n", raw_based_speed, raw_change);
            }

            last_speed_angle = mechanical_angle;
            last_raw_for_speed = raw_angle;
            last_speed_time = current_time;
        }
    }

    // 3. 속도 제어
    static float speed_ref = 1.0f;
    static uint32_t speed_change_time = 0;
    static uint8_t control_started = 0;
    static uint32_t start_time = 0;
    static int speed_step = 0;

    if(start_time == 0) start_time = HAL_GetTick();

    // 5초 후 제어 시작 (충분한 초기화 시간)
    if(!control_started && speed_sample_count >= 3 && (HAL_GetTick() - start_time > 5000)) {
        control_started = 1;
        speed_change_time = HAL_GetTick();
        printf("\r\n=== REAL CLOSED-LOOP CONTROL STARTED ===\r\n");
    }

    // 속도 변경 (느리게, 큰 차이로)
    if(control_started) {
        uint32_t elapsed = HAL_GetTick() - speed_change_time;
        if(elapsed > 15000) {  // 15초마다
            switch(speed_step) {
                case 0: speed_ref = 3.0f; break;   // 중간 속도
                case 1: speed_ref = 0.5f; break;   // 낮은 속도
                case 2: speed_ref = 6.0f; break;   // 높은 속도
                case 3: speed_ref = 1.0f; break;   // 기본 속도
            }
            speed_step = (speed_step + 1) % 4;
            speed_change_time = HAL_GetTick();
            printf("\r\n*** CLOSED-LOOP TARGET: %.1f rad/s ***\r\n", speed_ref);
        }
    }

    float vq = 1.0f;

    if(control_started) {
        // 진짜 폐루프 제어
        static float speed_integral = 0;
        static float prev_error = 0;

        float speed_error = speed_ref - measured_speed;  // 실제 측정 속도 사용!

        // 적분 계산 (큰 오차 시 리셋)
        if(fabsf(speed_error) < 5.0f) {
            speed_integral += speed_error * 0.5f;  // 500ms 간격 고려
        } else {
            speed_integral = 0;  // 리셋
        }

        // 적분 제한
        if(speed_integral > 3.0f) speed_integral = 3.0f;
        if(speed_integral < -3.0f) speed_integral = -3.0f;

        // 미분 계산
        float speed_derivative = (speed_error - prev_error) / 0.5f;  // 500ms 간격
        prev_error = speed_error;

        // 보수적인 PID 게인
        float kp = 0.5f;   // 비례 게인
        float ki = 0.3f;   // 적분 게인
        float kd = 0.01f;  // 미분 게인

        float control_output = kp * speed_error + ki * speed_integral + kd * speed_derivative;

        // 토크 계산
        float base_torque = 1.0f + fabsf(speed_ref) * 0.15f;
        vq = base_torque + control_output;

        // 토크 제한
        if(vq > 3.0f) vq = 3.0f;
        if(vq < 0.3f) vq = 0.3f;

        // 상태 출력 (2초마다)
        static uint32_t last_print = 0;
        if(current_time - last_print > 2000) {
            printf("CLOSED-LOOP: Ref=%.1f, Meas=%.3f, Err=%.3f, Vq=%.2f\r\n",
                   speed_ref, measured_speed, speed_error, vq);
            last_print = current_time;
        }

    } else {
        // 초기화 중에는 고정 토크
        vq = 1.0f;
        if(current_time % 2000 < 100) {  // 2초마다 한번씩 출력
            printf("Initializing... samples=%d, speed=%.2f\r\n", speed_sample_count, measured_speed);
        }
    }

    // 4. FOC 제어
    float vd = 0.0f;

    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);

    float v_alpha = vd * cos_theta - vq * sin_theta;
    float v_beta = vd * sin_theta + vq * cos_theta;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866025403f * v_beta;
    float vc = -0.5f * v_alpha - 0.866025403f * v_beta;

    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    // 듀티 제한
    if(duty_a > 0.9f) duty_a = 0.9f;
    if(duty_a < 0.1f) duty_a = 0.1f;
    if(duty_b > 0.9f) duty_b = 0.9f;
    if(duty_b < 0.1f) duty_b = 0.1f;
    if(duty_c > 0.9f) duty_c = 0.9f;
    if(duty_c < 0.1f) duty_c = 0.1f;

    set_pwm_duty(duty_a, duty_b, duty_c);
}
#endif

#if 0
// 직접 전압 제어 (확실한 속도 변화)
void FOC_control(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    static uint32_t start_time = 0;
    if(start_time == 0) start_time = HAL_GetTick();

    // 1. 간단한 엔코더 처리
    uint16_t raw_angle = AS5600_ReadRawAngle();
    float raw_mechanical = (4096.0f - (float)raw_angle) * TWO_PI / 4096.0f;
    raw_mechanical -= 0.32f;

    static float smooth_mechanical = 0;
    static uint8_t first_run = 1;

    if(first_run) {
        smooth_mechanical = raw_mechanical;
        first_run = 0;
    } else {
        float angle_diff = raw_mechanical - smooth_mechanical;
        while(angle_diff > PI) angle_diff -= TWO_PI;
        while(angle_diff < -PI) angle_diff += TWO_PI;

        // 빠른 응답
        smooth_mechanical += angle_diff * 0.8f;
    }

    mechanical_angle = smooth_mechanical;
    while(mechanical_angle > TWO_PI) mechanical_angle -= TWO_PI;
    while(mechanical_angle < 0) mechanical_angle += TWO_PI;
    electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);

    // 2. 직접 전압 제어 (전류 무시)
    static float target_voltage = 1.0f;
    static uint32_t speed_change_time = 0;
    static uint8_t control_started = 0;
    static int speed_step = 0;

    if(!control_started && (HAL_GetTick() - start_time > 2000)) {
        control_started = 1;
        speed_change_time = HAL_GetTick();
        printf("=== DIRECT VOLTAGE CONTROL STARTED ===\r\n");
        printf("Bypassing current control for maximum effect!\r\n");
    }

    if(control_started) {
        uint32_t elapsed = HAL_GetTick() - speed_change_time;
        if(elapsed > 6000) {  // 6초마다 빠르게 변경
            switch(speed_step) {
                case 0:
                    target_voltage = 6.0f;
                    printf("\r\n🚀 MAXIMUM VOLTAGE: 6.0V - SHOULD SPIN FAST! 🚀\r\n");
                    break;
                case 1:
                    target_voltage = 0.3f;
                    printf("\r\n🐌 MINIMUM VOLTAGE: 0.3V - VERY SLOW! 🐌\r\n");
                    break;
                case 2:
                    target_voltage = 8.0f;
                    printf("\r\n⚡ ULTRA HIGH: 8.0V - MAXIMUM SPEED! ⚡\r\n");
                    break;
                case 3:
                    target_voltage = 2.0f;
                    printf("\r\n🚶 MEDIUM VOLTAGE: 2.0V - NORMAL SPEED 🚶\r\n");
                    break;
                case 4:
                    target_voltage = 0.1f;
                    printf("\r\n🔍 BARELY MOVING: 0.1V - ALMOST STOP! 🔍\r\n");
                    break;
            }
            speed_step = (speed_step + 1) % 5;
            speed_change_time = HAL_GetTick();
        }
    }

    if(!control_started) {
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 3. 부드러운 전압 변화
    static float smooth_voltage = 1.0f;

    float voltage_change_rate = 5.0f;  // 5V/s
    float max_change = voltage_change_rate * 0.005f;
    float voltage_error = target_voltage - smooth_voltage;

    if(voltage_error > max_change) {
        smooth_voltage += max_change;
    } else if(voltage_error < -max_change) {
        smooth_voltage -= max_change;
    } else {
        smooth_voltage = target_voltage;
    }

    // 4. 직접 FOC 전압 적용 (전류 루프 완전 우회)
    float vd = 0.0f;  // d축은 0
    float vq = smooth_voltage;  // q축에 직접 전압 적용

    // 5. 즉시 Park 역변환
    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);

    float v_alpha = vd * cos_theta - vq * sin_theta;
    float v_beta = vd * sin_theta + vq * cos_theta;

    // 6. 즉시 Clarke 역변환
    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;

    // 7. 즉시 PWM 적용 (필터링 최소화)
    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    // 안전 제한만
    if(duty_a > 0.95f) duty_a = 0.95f;
    if(duty_a < 0.05f) duty_a = 0.05f;
    if(duty_b > 0.95f) duty_b = 0.95f;
    if(duty_b < 0.05f) duty_b = 0.05f;
    if(duty_c > 0.95f) duty_c = 0.95f;
    if(duty_c < 0.05f) duty_c = 0.05f;

    set_pwm_duty(duty_a, duty_b, duty_c);

    // 8. 속도 측정 (실제 확인용)
    static uint32_t last_angle_time = 0;
    static float last_angle = 0;
    static float measured_speed = 0;

    if(HAL_GetTick() - last_angle_time > 300) {  // 300ms마다
        if(last_angle_time != 0) {
            float angle_change = mechanical_angle - last_angle;
            if(angle_change > PI) angle_change -= TWO_PI;
            else if(angle_change < -PI) angle_change += TWO_PI;

            float time_diff = (HAL_GetTick() - last_angle_time) / 1000.0f;
            if(time_diff > 0) {
                measured_speed = fabsf(angle_change) / time_diff;  // rad/s
            }
        }

        last_angle = mechanical_angle;
        last_angle_time = HAL_GetTick();
    }

    // 9. 전류 측정 (참고용만)
    Read_Current_Sensors();

    // 상태 출력 (매우 명확하게)
    static uint32_t last_print = 0;
    if(HAL_GetTick() - last_print > 800) {
        printf("DIRECT: Voltage_target=%.1fV, Voltage_smooth=%.1fV, Speed=%.2f rad/s, Current_a=%.2fA\r\n",
               target_voltage, smooth_voltage, measured_speed, current_a);

        // 시각적 표시
        if(target_voltage > 6.0f) {
            printf(">>> STATUS: 🚀 VERY FAST - Should see rapid rotation! <<<\r\n");
        } else if(target_voltage < 0.5f) {
            printf(">>> STATUS: 🐌 VERY SLOW - Should barely move! <<<\r\n");
        } else {
            printf(">>> STATUS: 🚶 NORMAL - Moderate speed <<<\r\n");
        }

        last_print = HAL_GetTick();
    }

    // 10. 전압별 예상 결과 안내 (첫 출력시에만)
    static uint8_t info_printed = 0;
    if(!info_printed && control_started) {
        printf("\r\n=== VOLTAGE CONTROL INFO ===\r\n");
        printf("0.1V-0.5V: Very slow or stopped\r\n");
        printf("1.0V-2.0V: Slow to medium speed\r\n");
        printf("3.0V-5.0V: Fast speed\r\n");
        printf("6.0V-8.0V: Very fast speed\r\n");
        printf("Watch for clear speed differences!\r\n");
        printf("=============================\r\n\r\n");
        info_printed = 1;
    }
}
#endif

// 전류 속도 제어
// 육안으로 확인되는 속도 제어
#if 0
void FOC_control(void){
    if(!motor_enabled){
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    static uint32_t start_time = 0;
    if(start_time == 0) start_time = HAL_GetTick();

    // 1. 엔코더 처리 (부드러우면서 반응성 있게)
    // 수정
    uint16_t raw_angle = AS5600_ReadRawAngle();
    float raw_mechanical_angle = (4096.0f - (float)raw_angle) * TWO_PI / 4096.0f;
    float raw_mechanical = normalize_angle(raw_mechanical_angle - electrical_offset);

    static float smooth_mechanical = 0;
    static uint8_t first_run = 1;

    if(first_run) {
        smooth_mechanical = raw_mechanical;
        first_run = 0;
    } else {
        float angle_diff = raw_mechanical - smooth_mechanical;
        while(angle_diff > PI) angle_diff -= TWO_PI;
        while(angle_diff < -PI) angle_diff += TWO_PI;

        if(angle_diff > 0.1f) angle_diff = 0.1f;      // 제한 완화
        else if(angle_diff < -0.1f) angle_diff = -0.1f;

        smooth_mechanical += angle_diff * 0.5f;       // 더 빠른 응답
    }

    mechanical_angle = smooth_mechanical;
    while(mechanical_angle > TWO_PI) mechanical_angle -= TWO_PI;
    while(mechanical_angle < 0) mechanical_angle += TWO_PI;
    electrical_angle = fmodf(mechanical_angle * POLE_PAIRS, TWO_PI);

    // 2. 전류 센서 오프셋 캘리브레이션 (빠르게)
    static float ia_offset = 0, ib_offset = 0;
    static uint8_t current_calibrated = 0;
    static uint32_t calib_start_time = 0;
    static int calib_count = 0;
    static float ia_sum = 0, ib_sum = 0;

    if(!current_calibrated) {
        if(calib_start_time == 0) {
            calib_start_time = HAL_GetTick();
            printf("Quick calibration...\r\n");
        }

        if(HAL_GetTick() - calib_start_time < 1000) {  // 1초로 단축
            set_pwm_duty(0.5f, 0.5f, 0.5f);

            Read_Current_Sensors();
            ia_sum += current_a;
            ib_sum += current_b;
            calib_count++;
            return;
        } else {
            ia_offset = ia_sum / calib_count;
            ib_offset = ib_sum / calib_count;
            current_calibrated = 1;
            printf("Quick calibration done!\r\n");
        }
    }

    // 3. 극명한 차이의 전류 제어
    static float target_iq = 0.1f;
    static uint32_t speed_change_time = 0;
    static uint8_t control_started = 0;
    static int speed_step = 0;

    if(!control_started && current_calibrated && (HAL_GetTick() - start_time > 2000)) {
        control_started = 1;
        speed_change_time = HAL_GetTick();
        printf("=== VISIBLE SPEED CONTROL STARTED ===\r\n");
    }

    if(control_started) {
        uint32_t elapsed = HAL_GetTick() - speed_change_time;
        if(elapsed > 8000) {  // 8초마다
            switch(speed_step) {
                case 0:
                    target_iq = 0.2f;
                    printf("\r\n*** SLOW (Iq=0.2A) -***\r\n");
                    break;
                case 1:
                    target_iq = 0.05f;
                    printf("\r\n*** VERY SLOW (Iq=0.05A) - ALMOST STOP! ***\r\n");
                    break;
                case 2:
                    target_iq = 2.5f;               // 1.2A   --> 3.0A 수정
                    printf("\r\n*** MAXIMUM (Iq=2.5A) - FASTEST POSSIBLE! ***\r\n");
                    break;
                case 3:
                    target_iq = 0.8f;
                    printf("\r\n*** MEDIUM (Iq=0.8A) - NORMAL SPEED ***\r\n");
                    break;
                case 4:
                    target_iq = 0.01f;
                    printf("\r\n*** CRAWLING (Iq=0.01A) - BARELY MOVING! ***\r\n");
                    break;
            }
            speed_step = (speed_step + 1) % 5;
            speed_change_time = HAL_GetTick();
        }
    }

    if(!control_started) {
        set_pwm_duty(0.5f, 0.5f, 0.5f);
        return;
    }

    // 4. 반응성 있는 전류 필터링
    Read_Current_Sensors();

    float ia_corrected = current_a - ia_offset;
    float ib_corrected = current_b - ib_offset;

    // 가벼운 필터링 (반응성 우선)
    static float ia_filtered = 0, ib_filtered = 0;
    ia_filtered = 0.6f * ia_filtered + 0.4f * ia_corrected;  // 빠른 응답
    ib_filtered = 0.6f * ib_filtered + 0.4f * ib_corrected;

    // 5. Clarke 변환
    float i_alpha = ia_filtered;
    float i_beta = (ia_filtered + 2.0f * ib_filtered) * SQRT3_2;

    // 6. Park 변환
    float cos_theta = cosf(electrical_angle);
    float sin_theta = sinf(electrical_angle);

    float id_measured = i_alpha * cos_theta + i_beta * sin_theta;
    float iq_measured = -i_alpha * sin_theta + i_beta * cos_theta;

    //iq_measured = fabsf(iq_measured);

    // 7. 빠른 전류 지령값 변화
    static float smooth_iq_ref = 0.1f;

    //float iq_change_rate = 2.0f;  // 2.0 A/s (빠르게!)
    float iq_change_rate = 5.0f;
    float max_change = iq_change_rate * 0.005f;
    float iq_error_ref = target_iq - smooth_iq_ref;

    if(iq_error_ref > max_change) {
        smooth_iq_ref += max_change;
    } else if(iq_error_ref < -max_change) {
        smooth_iq_ref -= max_change;
    } else {
        smooth_iq_ref = target_iq;
    }

    // 8. 강력한 PID 제어 (빠른 응답)
    static float id_integral = 0.0f;
    static float iq_integral = 0.0f;

    float id_ref = 0.0f;
    float id_error = id_ref - id_measured;              // 비례 제어
    float iq_error = smooth_iq_ref - iq_measured;       // 비례 제어

    // 가벼운 오차 필터링
    static float id_error_filt = 0, iq_error_filt = 0;
    id_error_filt = 0.5f * id_error_filt + 0.5f * id_error;  // 빠른 응답
    iq_error_filt = 0.5f * iq_error_filt + 0.5f * iq_error;

    float dt = 0.005f;

    // 적분
    if(fabsf(id_error_filt) < 1.0f) {
        id_integral += id_error_filt * dt;
    }
    if(fabsf(iq_error_filt) < 1.0f) {
        iq_integral += iq_error_filt * dt;
    }

    // 적분 제한  오버 슈팅 방지
    if(id_integral > 0.2f) id_integral = 0.2f;
    if(id_integral < -0.2f) id_integral = -0.2f;
    if(iq_integral > 0.2f) iq_integral = 0.2f;
    if(iq_integral < -0.2f) iq_integral = -0.2f;

    float kp = 2.0f;   // 강력한 비례 게인
    float ki = 10.0f;   // 강력한 적분 게인

    float vd_command = kp * id_error_filt + ki * id_integral;
    float vq_command = kp * iq_error_filt + ki * iq_integral;

    // 9. 넉넉한 전압 제한
    static float vd_smooth = 0, vq_smooth = 0;

    if(vd_command > 11.0f) vd_command =11.0f;
    if(vd_command < -11.0f) vd_command = -11.0f;
    if(vq_command > 11.0f) vq_command = 11.0f;
    if(vq_command < -11.0f) vq_command = -11.0f;


    // 가벼운 전압 필터링
    vd_smooth = 0.7f * vd_smooth + 0.3f * vd_command;
    vq_smooth = 0.7f * vq_smooth + 0.3f * vq_command;

    // 10. 역 Park 변환 및 반응성 있는 PWM
    float v_alpha = vd_smooth * cos_theta - vq_smooth * sin_theta;
    float v_beta = vd_smooth * sin_theta + vq_smooth * cos_theta;

    // 11. 역 Clark 변환 공식
    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.866f * v_beta;
    float vc = -0.5f * v_alpha - 0.866f * v_beta;


    // 12.PWM 출력
    float duty_a = 0.5f + va / 12.0f;
    float duty_b = 0.5f + vb / 12.0f;
    float duty_c = 0.5f + vc / 12.0f;

    // 최소한의 듀티 필터링
    static float smooth_duty_a = 0.5f, smooth_duty_b = 0.5f, smooth_duty_c = 0.5f;

    smooth_duty_a = 0.8f * smooth_duty_a + 0.2f * duty_a;  // 빠른 응답
    smooth_duty_b = 0.8f * smooth_duty_b + 0.2f * duty_b;
    smooth_duty_c = 0.8f * smooth_duty_c + 0.2f * duty_c;

    set_pwm_duty(smooth_duty_a, smooth_duty_b, smooth_duty_c);

    // 11. 추가: 간단한 속도 추정 (참고용)
#if 0
    static uint32_t last_angle_time = 0;
    static float last_angle = 0;
    static float estimated_speed = 0;

    if(HAL_GetTick() - last_angle_time > 200) {  // 200ms마다
        float angle_change = mechanical_angle - last_angle;
        if(angle_change > PI) angle_change -= TWO_PI;
        else if(angle_change < -PI) angle_change += TWO_PI;

        float time_diff = (HAL_GetTick() - last_angle_time) / 1000.0f;
        if(time_diff > 0) {
            estimated_speed = fabsf(angle_change) / time_diff;  // rad/s
        }

        last_angle = mechanical_angle;
        last_angle_time = HAL_GetTick();
    }

    // 상태 출력 (더 자세히)
    static uint32_t last_print = 0;
    if(HAL_GetTick() - last_print > 1000) {
        printf("VISIBLE: Target=%.2f, Smooth=%.2f, Meas=%.3f, Err=%.3f, Vq=%.2f, Speed~%.1f rad/s\r\n",
               target_iq, smooth_iq_ref, iq_measured, iq_error_filt, vq_smooth, estimated_speed);
        last_print = HAL_GetTick();
    }
#endif
}
#endif



AlphaBeta_t clarke_transform(float a, float b, float c){
    AlphaBeta_t result;
    result.alpha = a;
    result.beta = (a + 2.0f * b) / SQRT3;  // 원래 공식
    return result;
}

DQ_t park_transform(AlphaBeta_t ab, float theta) {
    DQ_t result;
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    result.d = ab.alpha * cos_theta + ab.beta * sin_theta;
    result.q = -ab.alpha * sin_theta + ab.beta * cos_theta;
    return result;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Enable_GPIO_Port, &GPIO_InitStruct);
}

static void MX_ADC3_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc3.Instance = ADC3;
	hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc3.Init.ContinuousConvMode = ENABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	//hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC3;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc3) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == ADC3){
		adc_conversion_complete = 1;
	}
}

static void MX_DMA_Init(void)
{
	//__HAL_RCC_DMA2_CLK_ENABLE();
	//HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
	//HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);
	__HAL_RCC_DMA2_CLK_ENABLE();
	// DMA 핸들 구성
	hdma_adc3.Instance = DMA2_Channel5;  // ADC3는 보통 DMA2_Channel5 사용
	hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc3.Init.Mode = DMA_CIRCULAR;
	hdma_adc3.Init.Priority = DMA_PRIORITY_HIGH;

	if (HAL_DMA_Init(&hdma_adc3) != HAL_OK) {
		Error_Handler();
	}
	// ADC와 DMA 연결
	__HAL_LINKDMA(&hadc3, DMA_Handle, hdma_adc3);
	// DMA 인터럽트 설정
	HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);
}

static void MX_TIM1_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 2000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	TIM_OC_InitTypeDef sConfigOC_CH3 = {0};
	sConfigOC_CH3.OCMode = TIM_OCMODE_PWM1;
	sConfigOC_CH3.Pulse = 1000;  // Period(2000)의 50% 지점
	sConfigOC_CH3.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC_CH3.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC_CH3.OCIdleState = TIM_OCIDLESTATE_RESET;

	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC_CH3, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim1);
}

static void MX_USART2_UART_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
