#include "stm32g4xx_hal.h"

#include "settings.h"

MCUDef Mcu;

static int initPeripheral(void);

static void application(void);

int main(void) {
    HAL_Init();
    initPeripheral();

    while (true) {
        application();
    }
    return 1;
}

static int initPeripheral(void) {
    initialization(&Mcu);
    return 0;
}

static void application(void) {
    if (Mcu.flags.isSysTickTriggered) {
        Mcu.flags.isSysTickTriggered = false;
    }
}
