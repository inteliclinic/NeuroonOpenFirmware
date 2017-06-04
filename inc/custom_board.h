/**
 * @file    custom_board.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    May, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#include <stdint.h>


#define LEDS_NUMBER    0

#define BUTTONS_NUMBER 1
#define BUTTON_START   12
#define BUTTON_1       12
#define BUTTON_STOP    12
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1}

#define RX_PIN_NUMBER  19
#define TX_PIN_NUMBER  20
#define HWFC           false

#define SPIS_MISO_PIN  22    // SPI MISO signal.
#define SPIS_MOSI_PIN  23    // SPI MOSI signal.
#define SPIS_SCK_PIN   6    // SPI SCK signal.

#ifdef S210
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#else
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
#endif

//extern uint32_t m_app_ticks_per_100ms;

#endif /* !CUSTOM_BOARD_H */
