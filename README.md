# Bare-Metal template for STM32G431RB

Project template for STM32G431RB microcontroller.  
Hardware: [board](https://www.st.com/en/evaluation-tools/nucleo-g431rb.html)  
Library: [STM32 HAL](https://github.com/STMicroelectronics/STM32CubeG4)

## Features

- [x] Digital input pins;
- [x] Digital output pins;
- [x] Analog input pins;
- [x] Comparator (DAC threshold level);
- [x] DAC DC output and triangle(2.2kHz)/noise/sawtooth(9kHz) signals generation;
- [x] Complementary PWM signal (1 channel);
- [x] RNG;
- [x] CRC-32/ISO-HDLC;
- [x] Independent WDT;
- [x] MCU-to-PC USART connection;
- [x] MCU-to-Device USART connection;

## MCU Settings

1) Clock: HSI(16MHz) -> PLL -> SYSCLK(144MHz) -> HCLK(144MHz) -> PCLK1(36MHz) / PCLK2(36MHz);
2) Digital input pins: PC13 (pull-down);
3) Digital output pins: PA8/PA9/PA10/PB5 (pull-down), PA5 (pull-down) - commented;
4) SystemTick timer: 1kHz;
5) Measurement timer: 4kHz;
6) PWM: 10kHz, channel 1, PC6/PC10/PB7 (pull-down), break input 1 - active HIGH, dead time - 64;
7) ADC: 12-bits, right, software start, 92.5 cycles, PA0/PA1/PB0 + temperature sensor;
8) DAC: DC output - channel 1 (PA4), generator - channel 2 (PA5)
9) Comparator: PA7, hysteresis - 40mV, output - non-inverted, DAC3 - channel 2;
10) USART: USART1 - PC4/PC5, USART2 - PA2/PA3, 115200, 8N1, TX/RX, FIFO - disabled;
11) CRC: input data - bytes, polynomial - 0x04C11DB7, init value - 0xFFFFFFFF, input inversion, output inversion;
12) WDT: 1 second, prescaler - 8, reload value = 0x0FFF;

## Project structure

| Folder name | Description                                             |
|:-----------:|:--------------------------------------------------------|
|     app     | Source code                                             |
|    core     | CMSIS and core ARM/Cortex header files                  |
|   lib/hal   | Hardware abstraction layer(HAL) source and header files |
|   startup   | Linker files                                            |
|   system    | System source and header files                          |

## Project settings

- CMakeLists.txt file
