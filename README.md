# Bare-Metal template for STM32G431RB

Project template for STM32G431RB microcontroller.  
Hardware: [board](https://www.st.com/en/evaluation-tools/nucleo-g431rb.html)  
Library: [STM32 HAL](https://github.com/STMicroelectronics/STM32CubeG4)

## Features

- [x] Button;
- [x] LED;
- [x] 2 analog input pins;
- [x] Build-in temperature sensor;
- [x] Comparator;
- [x] DAC;
- [x] PWM;
- [x] MCU-to-PC UART connection;
- [x] CRC-32/ISO-HDLC;
- [x] Independent WDT;

## MCU Settings

1) Clock: HSI(16MHz) -> PLL -> SYSCLK(144MHz) -> HCLK(144MHz) -> PCLK1(36MHz) / PCLK2(36MHz);
2) Digital input pins: PC13 (pull-down);
3) Digital output pins: PA5 (pull-down);
4) SystemTick timer: 1kHz;
5) Measurement timer: Timer 15, 4kHz;
6) ADC: 12-bits, right, software start, 92.5 cycles, PA0/PA1 + temperature sensor;
7) Comparator: PA7, hysteresis - 40mV, output - non-inverted, DAC3 - channel 2;
8) DAC: DC output, channel 1, PA4;
9) PWM: Timer 16, 10kHz, channel 1, complementary, PB4/PB6, dead time - 24 (330 ns);
10) UART: UART1 - PC4/PC5, 115200, 8N1, TX/RX, FIFO - disabled;
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
