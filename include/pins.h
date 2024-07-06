
//클래스를 정의할 때 선언과 정의를 분리하는데 이때 헤더파일에 선언을 넣고, 
//다른 CPP 파일에 정의를 넣는다.

//include의 중복을 방지하기 위해서 다음과 같은 전처리기를 사용한다.
#ifndef ROBOT_PWM_PINS_H
#define ROBOT_PWM_PINS_H

//해당 프로젝트에 LED로 사용되었다.
//RPi pico는 LED pin을 25으로 할당해서 사용할 수 있다
#define LED_PIN 25

//첫번째 모터의 PWM컨트롤 핀
#define M1_PWM_PIN 2
//모터의 방향을 결정하는 Enable A, Enable B
//Caution!! M1 motor Enalble A, Enable B should be opposite!
#define M1_DIR_PIN 3

//두번째 모터의 PWM컨트롤 핀
#define M2_PWM_PIN 6
//모터의 방향을 결정하는 Enable A, Enable B
#define M2_DIR_PIN 7

//버튼핀
#define POT_PIN 26
//첫 번째 모터에 부착된 엔코더 인터럽트 핀(2채널)
#define M1_ENC_A_PIN 10
#define M1_ENC_B_PIN 11
//두 번째 모터에 부착된 엔코더 인터럽트 핀(2채널)
#define M2_ENC_A_PIN 12
#define M2_ENC_B_PIN 13
//true, false를 정의한 define
//Not used define
#define M1_ENC_INVERTED false
#define M2_ENC_INVERTED true

#endif //ROBOT_PWM_PINS_H
