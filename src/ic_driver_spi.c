/**
 * @file    ic_driver_spi.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    czerwiec, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_driver_spi.h"
#include "ic_config.h"

#include "app_error.h"
#define NRF_LOG_MODULE_NAME "SPI"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

void ic_spi_init(const nrf_drv_spi_t *const instance, uint8_t pin, void(*func)()){
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = pin;
    spi_config.miso_pin = IC_SPI_MISO_PIN;
    spi_config.mosi_pin = IC_SPI_MOSI_PIN;
    spi_config.sck_pin  = IC_SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(instance, &spi_config, func));
}
