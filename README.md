# code base_controller mod for stm32f411

- STM32 is complicated than Teensy so much. Because It demand to config hardware before using. The file config name is ```rosserial_test_stf411.ioc```. I configed it with Timer 2 and Timer 5 to read Encoder (quadrature x4 mode). And Timer 3 and Timer 4 to control PWM signal for two motors. And USART 1 for rosserial.
- All file header *.h I moved to **include** folder, with all libraries of rosserial_stm32. All file *.cpp are on **src** folder.
- I removed _Adafruit_feather_wing_ file, because I just using with _L298_motor_driver_ to direct control motors.
- The main file now is ```mainpp.cpp```. It's included to ```main.cpp``` via ```mainpp.h```, the main file of STM32.
