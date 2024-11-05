#ifndef ROBOT_PWM_CONFIG_H
#define ROBOT_PWM_CONFIG_H

//모터(Left) 최대/최소 속도
#define L_MOTOR_MIN_SPEED -0.5f
#define L_MOTOR_MAX_SPEED 0.5f
//모터(Right) 최대/최소 속도
#define R_MOTOR_MIN_SPEED -0.5f
#define R_MOTOR_MAX_SPEED 0.5f


//Motor spec
// 8000RPM 24V MY36GP-555
// Ratio 50
// RPM 160
// 1. 초당 회전수(RPS) 계산
// 모터의 정격 속도가 125 RPM이므로, 초당 회전수는 다음과 같습니다:
// 회전수= 125/60 ≈2.083 RPS (회전/초)
// 2. 바퀴 둘레 계산
// 바퀴의 지름이 𝐷cm인 경우, 바퀴의 둘레 𝐶는 다음과 같습니다:
// C=π×D
// 3. 이동 속도 계산
// 바퀴 둘레와 회전수를 곱하면 초당 이동 거리가 됩니다:
// 이동 속도 (cm/s)=C×회전수=π×D×2.083

// example
// 예를 들어, 바퀴 지름이 10cm인 경우:
// C=π×10≈31.4 cm
// 이동 속도=31.4×2.083≈65.4 cm/s
// 이는 약 0.654 m/s에 해당합니다.

//1회 회전당 pulse의 갯수일듯?
//1CH - 16PPR
//2Channel Multiply
//HIGH EDGE, LOW EDGE 2 Signal
//Planetary gear 모터로 50:1의 비율
// 16 x 2 x 2 x 50 = 3200 PPR (1Tick 당 0.225도 )
//3200 PPR 모터의 경우 1 tick당 회전 각도는 0.1125도입니다.
#define ROBOT_MOTOR_PPR 3200.0f
//바퀴 반지름
#define ROBOT_WHEEL_RADIUS 0.050f
//축간거리
#define ROBOT_WHEEL_SEPARATION 0.34f
//Motor 직선 속도, Motor 각속도
// #define ROBOT_MAX_LINEAR_M_S 0.08
// #define ROBOT_MIN_LINEAR_M_S -0.08
// #define ROBOT_MAX_ANGULAR_R_S 0.25
// #define ROBOT_MIN_ANGULAR_R_S -0.25
#define ROBOT_MAX_LINEAR_M_S 0.1
#define ROBOT_MIN_LINEAR_M_S -0.1
#define ROBOT_MAX_ANGULAR_R_S 0.25
#define ROBOT_MIN_ANGULAR_R_S -0.25

#endif //ROBOT_PWM_CONFIG_H