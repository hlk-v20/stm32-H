#include "stm32f10x.h"
#include "OLED.h"
#include "LED.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "Timer.h"

#define MAX_PWM              60
#define MIN_PWM             -60
#define MOTOR_DEAD           25

#define POSITION_TARGET_CNT  5000.0f // target position (encoder average counts)
#define ANGLE_TARGET_CNT     0.0f    // target heading (left-right count difference)
#define STOP_POS_BAND        20.0f
#define STOP_VEL_BAND        1.0f

static int16_t enc_left_delta = 0;
static int16_t enc_right_delta = 0;
static int32_t enc_left_sum = 0;
static int32_t enc_right_sum = 0;

static float position_actual = 0.0f;
static float angle_actual = 0.0f;
static float car_speed_cnt = 0.0f;

static int16_t motor_left_pwm = 0;
static int16_t motor_right_pwm = 0;

static PID_t PositionPID =
{
	.Kp = 0.030f,
	.Ki = 0.0012f,
	.Kd = 0.080f,
	.OutMax = 45.0f,
	.OutMin = -45.0f,
	.IntegralMax = 400.0f,
	.IntegralMin = -400.0f,
};

static PID_t AnglePID =
{
	.Kp = 0.90f,
	.Ki = 0.002f,
	.Kd = 0.60f,
	.OutMax = 45.0f,
	.OutMin = -45.0f,
	.IntegralMax = 120.0f,
	.IntegralMin = -120.0f,
};

static float absf_local(float x)
{
	return (x >= 0.0f) ? x : -x;
}

static int16_t clamp_dead_pwm(float pwm)
{
	int16_t out;
	if (pwm > (float)MAX_PWM) pwm = (float)MAX_PWM;
	if (pwm < (float)MIN_PWM) pwm = (float)MIN_PWM;

	out = (int16_t)pwm;
	if (out > 0 && out < MOTOR_DEAD) out = MOTOR_DEAD;
	if (out < 0 && out > -MOTOR_DEAD) out = -MOTOR_DEAD;
	return out;
}

int main(void)
{
	OLED_Init();
	LED_Init();
	Motor_Init();
	Encoder_Init();
	Timer_Init();

	PID_Init(&PositionPID);
	PID_Init(&AnglePID);

	while (1)
	{
		OLED_ShowString(1, 1, "Pos:", OLED_6X8);
		OLED_ShowSignedNum(1, 5, (int32_t)position_actual, 6, OLED_6X8);
		OLED_ShowString(2, 1, "Ang:", OLED_6X8);
		OLED_ShowSignedNum(2, 5, (int32_t)angle_actual, 6, OLED_6X8);
		OLED_ShowString(3, 1, "L:", OLED_6X8);
		OLED_ShowSignedNum(3, 3, motor_left_pwm, 4, OLED_6X8);
		OLED_ShowString(4, 1, "R:", OLED_6X8);
		OLED_ShowSignedNum(4, 3, motor_right_pwm, 4, OLED_6X8);
	}
}

void TIM4_IRQHandler(void)
{
	float pos_out;
	float ang_out;

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		enc_left_delta = Encoder_Get(1);
		enc_right_delta = Encoder_Get(2);

		enc_left_sum += enc_left_delta;
		enc_right_sum += enc_right_delta;

		position_actual = (enc_left_sum + enc_right_sum) * 0.5f;
		angle_actual = (enc_right_sum - enc_left_sum);
		car_speed_cnt = (enc_left_delta + enc_right_delta) * 0.5f;

		PositionPID.Target = POSITION_TARGET_CNT;
		PositionPID.Actual = position_actual;
		PID_Update(&PositionPID);
		pos_out = PositionPID.Out;

		AnglePID.Target = ANGLE_TARGET_CNT;
		AnglePID.Actual = angle_actual;
		PID_Update(&AnglePID);
		ang_out = AnglePID.Out;

		// differential drive synthesis:
		// position loop gives forward drive, angle loop gives steering correction
		motor_left_pwm = clamp_dead_pwm(pos_out - ang_out);
		motor_right_pwm = clamp_dead_pwm(pos_out + ang_out);

		if (absf_local(PositionPID.Target - position_actual) < STOP_POS_BAND &&
			absf_local(car_speed_cnt) < STOP_VEL_BAND)
		{
			motor_left_pwm = 0;
			motor_right_pwm = 0;
			PID_Reset(&PositionPID);
			PID_Reset(&AnglePID);
		}

		Motor_SetPWM(1, motor_left_pwm);
		Motor_SetPWM(2, motor_right_pwm);

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}
