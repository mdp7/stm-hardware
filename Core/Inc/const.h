/*
 * const.h
 *
 *  Created on: Sep 24, 2022
 *      Author: Bian Hengwei
 */

#ifndef INC_CONST_H_
#define INC_CONST_H_

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define FORWARD_DIR 0
#define BACKWARD_DIR 1

#define PWM_MIN 1000
#define PWM_MAX 6000  // TODO
#define SERVO_MIN 50
#define SERVO_MAX 280

// Robot car constants
#define WHEEL_R 3.4
#define WHEEL_C 21.3
#define SMALL_WHEEL_C 21.3
#define PI 3.14
#define PULSE_PER_REVOLUTION 1560

// Straight line constant
#define DEFAULT_FORWARD_SERVO 150
//#define DEFAULT_FORWARD_PWM_L 2300  // TODO: test on lab floor
//#define DEFAULT_FORWARD_PWM_R 2600  // TODO: test on lab floor
//#define DEFAULT_FORWARD_PWM_L 2200  // TODO: test on lab floor
//#define DEFAULT_FORWARD_PWM_R 2400  // TODO: test on lab floor
//#define DEFAULT_FORWARD_PWM_L 1500  // TODO: test on lab floor
//#define DEFAULT_FORWARD_PWM_R 1650  // TODO: test on lab floor
#define DEFAULT_FORWARD_PWM_L 2500  // TODO: test on lab floor
#define DEFAULT_FORWARD_PWM_R 2700  // TODO: test on lab floor
#define FORWARD_SLIDE_PULSE 0  // OFFSET


#define SMALL_FORWARD_SLIDE_PULSE 150

// x pulse/ms = (x / 1560) revolution/ms = (6 * pi * x / 1560) cm/ms
// 3 pulse/ms = 0.002 revolution/ms = 0.036 cm/ms
// 6 pulse/ms = 0.004 revolution/ms = 0.072 cm/ms = 0.725 m/s
#define FORWARD_PULSE_PER_MS 5  // 0.85 m/s
#define PPMS_THRESHOLD 0.01
#define PPMS_TO_PWM_INCREASE_RATIO 20
#define PPMS_TO_PWM_RATIO 55  // HOW FAST THE SPEED IS CHANGING
#define FORWARD_DELAY 10
#define PPMS_REDUCE_PULSE_DIST 30
#define PPMS_REDUCE_VALUE 1
#define SUDDEN_BREAK_DIST 20

// Gyro
#define GYRO_ERR_THRESHOLD 1
#define GYRO_DELAY 20

// Other constants
#define OLED_DELAY 20
/* USER CODE END Private defines */

#endif /* INC_CONST_H_ */
