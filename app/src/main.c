#include "stm32g4xx_hal.h"

#include "variables.h"

static int initPeripheral(void);

static void application(void);

int main(void) {

    initPeripheral();

    while (true) {
        application();
    }
    return 1;
}

static int initPeripheral(void) {

    return 0;
}

static void application(void) {

}
