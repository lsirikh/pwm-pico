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

