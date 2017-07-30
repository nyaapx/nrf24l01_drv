/**
 * @file
 * @brief
 *          nrf24l01 HAL hardware interface
 *
 * @internal
 * @par Modification history
 * - 1.00   2017-07-29 noodlefighter, copy from Nordic nAN24-12, modified the
 *                     hardware interface for efficiency.
 * @endinternal
 */

#ifndef __HAL_NRF_HW_H
#define __HAL_NRF_HW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * SPI exchange data
 */
void nrf_hw_spi_exchange (const uint8_t *p_tx, uint8_t *p_rx, size_t size);

/**
 * SPI tx a byte and rx several bytes, rx_size > 0
 */
void nrf_hw_spi_tx_rx (uint8_t tx_byte, uint8_t *p_rx, size_t rx_size);

/**
 * SPI tx a byte and tx several bytes, tx_size > 0
 */
void nrf_hw_spi_tx_tx (uint8_t tx_byte, const uint8_t *p_tx, size_t tx_size);

/**
 * NRF pin CE set low
 */
void nrf_hw_ce_low (void);

/**
 * NRF pin CE set high
 */
void nrf_hw_ce_high (void);

/**
 * Pulses the CE to nRF24L01 for at least 10 us
 */
void nrf_hw_ce_pulse (void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __HAL_NRF_HW_H */
