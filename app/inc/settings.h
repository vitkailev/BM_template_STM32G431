#ifndef STM32G431_SETTINGS_H
#define STM32G431_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "variables.h"

enum Settings_Constants {
    SETTING_SUCCESS,
    SETTING_ERROR = -1
};

int initialization(MCUDef *mcu);

#ifdef __cplusplus
}
#endif

#endif // STM32G431_SETTINGS_H
