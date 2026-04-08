#include "stm32f10x.h"
//#include "OLED.h"
#include "LED.h"
#include "Motor.h"
#include "Encoder.h"
#include "Track.h"
#include "buzzer.h"
#include "PID.h"
#include "Timer.h"

#define ENCODER_NUM          44.0f
#define CTRL_PERIOD_S        0.001f
#define MAX_PWM              60
#define MIN_PWM             -60
#define MOTOR_DEAD           25
#define RUN_BASE_PWM         34
#define ANGLE_TARGET_CNT     0.0f
#define B_DETECT_CONFIRM     4
#define B_DETECT_ARM_MS      800U
#define RUN_TIMEOUT_MS       15000U
#define STOP_BLINK_MS        200U

static int16_t enc_left_delta = 0;
static int16_t enc_right_delta = 0;
static int32_t enc_left_sum = 0;
static int32_t enc_right_sum = 0;

static float position_actual = 0.0f;
static float angle_actual = 0.0f;
static float car_speed_cnt = 0.0f;
static float motor_left_speed_rpm = 0.0f;
static float motor_right_speed_rpm = 0.0f;

static int16_t motor_left_pwm = 0;
static int16_t motor_right_pwm = 0;
static uint8_t track_mask = 0;
static uint8_t b_detect_count = 0;
static uint8_t car_stop_flag = 0;
static uint32_t run_time_ms = 0;
static uint32_t stop_blink_ms = 0;

static PID_t AnglePID =
{
	.Kp = 0.90f,
	.Ki = 0.002f,
	.Kd = 0.60f,
	.OutMax = 35.0f,
	.OutMin = -35.0f,
	.IntegralMax = 120.0f,
	.IntegralMin = -120.0f,
};

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

static void stop_with_alarm(void)
{
	motor_left_pwm = 0;
	motor_right_pwm = 0;
	car_stop_flag = 1;
	stop_blink_ms = 0;
	PID_Reset(&AnglePID);
	LED_OFF();
	buzzer_off();
}

int main(void)
	{
			//	OLED_Init();
				LED_Init();
				LED_OFF();
				buzzer_Init();
				buzzer_off();
				Track_Init();
				Encoder_Init();
				Timer_Init();

	PID_Init(&AnglePID);

	while (1)
	{
//		OLED_Clear();
//		OLED_ShowString(0, 0, "Pos:", OLED_6X8);
//		OLED_ShowSignedNum(30, 0, (int32_t)position_actual, 6, OLED_6X8);
//		OLED_ShowString(0, 16, "Ang:", OLED_6X8);
//		OLED_ShowSignedNum(30, 16, (int32_t)angle_actual, 6, OLED_6X8);
//		OLED_ShowString(0, 32, "VL:", OLED_6X8);
//		OLED_ShowSignedNum(24, 32, (int32_t)motor_left_speed_rpm, 4, OLED_6X8);
//		OLED_ShowString(0, 48, "VR:", OLED_6X8);
//		OLED_ShowSignedNum(24, 48, (int32_t)motor_right_speed_rpm, 4, OLED_6X8);
//		OLED_Update();
	}
}

void TIM4_IRQHandler(void)
{
	float ang_out;
	float base_out;

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		run_time_ms++;
		enc_left_delta = Encoder_Get(1);
		enc_right_delta = Encoder_Get(2);

		enc_left_sum += enc_left_delta;
		enc_right_sum += enc_right_delta;
		motor_left_speed_rpm = ((float)enc_left_delta * 60.0f) / (ENCODER_NUM * CTRL_PERIOD_S);
		motor_right_speed_rpm = ((float)enc_right_delta * 60.0f) / (ENCODER_NUM * CTRL_PERIOD_S);

		position_actual = (enc_left_sum + enc_right_sum) * 0.5f;
		angle_actual = (enc_right_sum - enc_left_sum);
		car_speed_cnt = (enc_left_delta + enc_right_delta) * 0.5f;
		track_mask = Track_ReadMask();

		if (car_stop_flag == 0)
		{
			AnglePID.Target = ANGLE_TARGET_CNT;
			AnglePID.Actual = angle_actual;
			PID_Update(&AnglePID);
			ang_out = AnglePID.Out;

			base_out = RUN_BASE_PWM;
			motor_left_pwm = clamp_dead_pwm(base_out - ang_out);
			motor_right_pwm = clamp_dead_pwm(base_out + ang_out);

			if (Track_IsBPoint(track_mask))
			{
				if (b_detect_count < B_DETECT_CONFIRM)
				{
					b_detect_count++;
				}
			}
			else
			{
				b_detect_count = 0;
			}

			if ((run_time_ms >= B_DETECT_ARM_MS) && (b_detect_count >= B_DETECT_CONFIRM))
			{
				stop_with_alarm();
			}

			if (run_time_ms >= RUN_TIMEOUT_MS)
			{
				stop_with_alarm();
			}
		}
		else
		{
			motor_left_pwm = 0;
			motor_right_pwm = 0;
			stop_blink_ms++;
			if (stop_blink_ms >= STOP_BLINK_MS)
			{
				stop_blink_ms = 0;
				LED_Turn();
				buzzer_turn();
			}
		}

		Motor_SetPWM(1, motor_left_pwm);
		Motor_SetPWM(2, motor_right_pwm);

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}
