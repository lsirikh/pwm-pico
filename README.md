## Raspberry pi pico PWM dual motor controller

** developer : GH  
** uploaded : 2024.06.28  


### Build

```
cmake .. && make && sudo mount /dev/sda1 /media/pico/ && sudo cp robot_pwm.uf2 /media/pico/ && sudo umount /media/pico/

```