/**
 * @file
 * @brief 
 *     
 *     details...
 *
 * @internal
 * @par Modification history
 * - 1.00 2017年7月29日  noodlefighter, first implementation
 * @endinternal
 */

#include <stdint.h>

#include "fc_spi.h"

/**
 * Define IRQ an CE PORT/PAD
 * Make sure to setup the IRQ pad with the EXT driver as well
 */
#define _NRF_PORT_CE_IRQ    IOPORT1
#define _NRF_PIN_IRQ        2
#define _NRF_PIN_CE         3

/**
 * SPI exchange data
 */
void nrf_hw_spi_exchange (const uint8_t *p_tx, uint8_t *p_rx, size_t size)
{
    spi1_exchange(p_tx, p_rx, size);
}

/**
 * SPI tx a byte and rx several bytes, rx_size > 0
 */
void nrf_hw_spi_tx_rx (uint8_t tx_byte, uint8_t *p_rx, size_t rx_size)
{
    spi1_tx_and_rx(tx_byte, p_rx, rx_size);
}

/**
 * SPI tx a byte and tx several bytes, tx_size > 0
 */
void nrf_hw_spi_tx_tx (uint8_t tx_byte, const uint8_t *p_tx, size_t tx_size)
{
    spi1_tx_and_tx(tx_byte, p_tx, tx_size);
}

/**
 * NRF pin CE set low
 */
void nrf_hw_ce_low (void)
{
    palClearPad(_NRF_PORT_CE_IRQ, _NRF_PIN_CE);
}

/**
 * NRF pin CE set high
 */
void nrf_hw_ce_high (void)
{
    palSetPad(_NRF_PORT_CE_IRQ, _NRF_PIN_CE);
}

/**
 * Pulses the CE to nRF24L01 for at least 10 us
 */
void nrf_hw_ce_pulse (void)
{
    nrf_hw_ce_high();

    /* Cortex-M3 72MHz, 10us */
    us = 72 * 10;
    while (--us);

    nrf_hw_ce_low();
}
