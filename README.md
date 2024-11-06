## Raspberry pi pico PWM dual motor controller

** developer : GH  
** uploaded : 2024.06.28  


### Bill of Material  

* MCU : Raspberry pi pico (1ea)  
* Motor Driver : MD10C R3 (2ea)  
* Motor : 36GP-555 24V/160RPM (2ea)  
* Battery : 24V Li (1ea)  

### Build

```
cmake .. && make && sudo mount /dev/sda1 /media/pico/ && sudo cp robot_pwm.uf2 /media/pico/ && sudo umount /media/pico/

```

### Issue
1. 보통 모터는 각각 PWM에 따른 속도의 차이가 존재한다. 해당 모터는 약 +/-5%의 오차율이 존재한다고 하였다. 해당 오차를 극복하기 위해서 PID 및 Loopback control을 통해 두 모터사이에 tick difference를 최대하 줄여야 한다. 특히 직진을 할 때, 모터사이에 속도차(모터 특성에 따른)가 발생하면 전진이 아닌 약간 비스듬하게 전진을 하는 경우가 많이 발생한다.  

<P>
** 현재는 무부하시 1분간 전진을 할 경우 : 약 50 tick diff  <br>
***    (Linearity : 0.2f 기준)   <br>
***    (PID Left : 0.1721, 0.0001, 0.0001)  <br>
***    (PID Right : 0.17, 0.0001, 0.0001)  <br>
</p>

2. 이제 시리얼로 Linearity를 제어하여 원격제어하는 로직을 구현한다.

3. 해당 로직이 구현되면, 신호수집을 위한 ROS2 코드를 구성한다. 

### v1.0  
1. 현재의 PID 세팅값은 아래와 같다.
<P>
** 현재는 무부하시 1분간 전진을 할 경우 : 약 50 tick diff  <br>
***    (Linearity : 0.05f 기준)   <br>
***    (PID Left : 0.17, 0.01, 0.0001)  <br>
***    (PID Right : 0.1712, 0.01, 0.0001)  <br>
</p>

2. 이제 시리얼로 Linearity를 제어하여 원격제어하는 로직구현되어 있다.
3. 해당 로직이 구현되면, 신호수집을 위한 ROS2 코드가 구현되어 있다.  

### v1.1  
### Date: 2024-11-02
1. CRC를 적용하여 데이터 전송에 안정성을 확보하였다.  
2. USB Serial 데이터 통신에 불안정한 요소는 여전히 존재하나 물리적인 케이블 교체 및 Baudrate 변경을 통해 디버깅이 필요하다.  
3. 새로 구매한 24V DC 모터(같은 모델)를 적용하여 PID 수정이 필요하다.  

### v1.2  
### Date: 2024-11-06
1. ctx와 ext를 4byte씩 할당하여 전체적인 메시지의 구조체를 구현하였다.   
```
#pragma pack(push, 1)  // 구조체 패딩을 제거하여 정확한 바이트 배열을 유지
struct Message {
    uint16_t ctx;             // 2바이트, 시작 표시자 (예: 0x1234)
    float l_speed;            // 4바이트, 왼쪽 바퀴 속도
    float r_speed;            // 4바이트, 오른쪽 바퀴 속도
    float x_pos;              // 4바이트, x 좌표
    float y_pos;              // 4바이트, y 좌표
    float theta;              // 4바이트, 방향 각도
    float v;                  // 4바이트, 직선 속도
    float w;                  // 4바이트, 각속도
    uint16_t crc;             // 2바이트, CRC-16-CCITT
    uint16_t ext;             // 2바이트, 종료 표시자 (예: 0x5678)
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ControlMessage {
    uint16_t ctx;      // 2바이트, 시작 표시자 (예: 0x1234)
    float linear;      // 4바이트, 직선 속도 명령
    float angular;     // 4바이트, 각속도 명령
    uint16_t crc;      // 2바이트, CRC-16-CCITT
    uint16_t ext;      // 2바이트, 종료 표시자 (예: 0x5678)
};
#pragma pack(pop)

```

2. 메시지의 본문에 해당하는 내용만 CRC로 계산하였다.  
3. 라즈베리파이와 메시지 연동하는 과정에서 에러에 의한 누락이 거의 제거되었다.  


### v1.3
### Date: 2024-11-07

1. ControlMessage를 command라는 속성을 추가하여 command에 따라 speed command, pid update 두가지 기능으로 분화하였다.  
```
#pragma pack(push, 1)
struct ControlMessage {
    uint16_t ctx;        // 2바이트
    uint16_t command_type; // 2바이트
    union {
        struct {
            float linear;      // 4바이트
            float angular;     // 4바이트
            uint16_t crc;        // 2바이트
            uint16_t ext;        // 2바이트
        } speed_command;
        struct {
            float kp_left;     // 4바이트
            float ki_left;     // 4바이트
            float kd_left;     // 4바이트
            float kp_right;    // 4바이트
            float ki_right;    // 4바이트
            float kd_right;    // 4바이트
            uint16_t crc;        // 2바이트
            uint16_t ext;        // 2바이트
        } pid_update;
    };
};
#pragma pack(pop)

```
2. 물론 receiveMessage 역시 해당 구조에 따라 변경되었다.  