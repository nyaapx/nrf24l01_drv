/**
 * @file
 * @brief 
 *     
 *     details...
 *
 * @internal
 * @par Modification history
 * - 1.00 2017年7月1日  noodlefighter, first implementation
 * @endinternal
 */

#ifndef NRF24L01_CFG_H_
#define NRF24L01_CFG_H_

/**
 * @brief   RX/TX Address field width  \ref NRF24L01_CFG_ADDR_LENGTH_*
 */
#define NRF24L01_CFG_ADDR_LENGTH        NRF24L01_CFG_ADDR_LENGTH_5BYTE

/**
 * @brief   Air Data Rate   \ref NRF24L01_CFG_DATA_RATE_*
 */
#define NRF24L01_CFG_DATA_RATE          NRF24L01_CFG_DATA_RATE_2MBPS

/**
 * @brief CRC encoding scheme  \ref NRF24L01_CFG_CRCO_*
 */
#define NRF24L01_CFG_CRCO               NRF24L01_CFG_CRCO_1BYTE

/**
 * @brief Auto Retransmit Delay
 */
#define NRF24L01_CFG_ARD                NRF24L01_CFG_ARD_250US

/**
 * @brief   Auto Retransmit Count
 * @details Up to N Re-Transmit on fail of auto ack
 */
#define NRF24L01_CFG_ARC                3

/**
 * @brief Setup LNA gain
 */
#define NRF24L01_CFG_LNA_ENABLE         1

#define NRF24L01_CFG_


/* SPI interface adapt -------------------------------------------------------*/

#include "fc_spi.h"

/*
 * exchange data
 */
static inline
void nrf24l01_spi_exchange (const uint8_t *p_tx, uint8_t *p_rx, size_t size)
{
    spi1_exchange(p_tx, p_rx, size);
}

/*
 * tx a byte and rx several bytes, rx_size > 0
 */
static inline
void nrf24l01_spi_tx_rx (uint8_t tx_byte, uint8_t *p_rx, size_t rx_size)
{
    spi1_tx_and_rx(tx_byte, p_rx, rx_size);
}

/*
 * tx a byte and tx several bytes, tx_size > 0
 */
static inline
void nrf24l01_spi_tx_tx (uint8_t tx_byte, const uint8_t *p_tx, size_t tx_size)
{
    spi1_tx_and_tx(tx_byte, p_tx, tx_size);
}

/* NRF24L01 Pin Control ------------------------------------------------------*/
#include "hal.h"

/*
 * Define IRQ an CE PORT/PAD
 * Make sure to setup the IRQ pad with the EXT driver as well
 */
#define NRF24L01_PORT_CE_IRQ    IOPORT1
#define NRF24L01_PIN_IRQ        2
#define NRF24L01_PIN_CE         3

/*
 * set pin CE to output push-pull mode
 */
static inline
void nrf24l01_pin_ce_out_pp_set (void)
{
    palSetPadMode(NRF24L01_PORT_CE_IRQ, NRF24L01_PIN_CE, PAL_MODE_OUTPUT_PUSHPULL); /* EC, OUTPUT to change send/receive mode */
}

static inline
void nrf24l01_pin_ce_set (uint8_t val)
{
    if (val) {
        palSetPad(NRF24L01_PORT_CE_IRQ, NRF24L01_PIN_CE);
    } else {
        palClearPad(NRF24L01_PORT_CE_IRQ, NRF24L01_PIN_CE);
    }
}

/* Common --------------------------------------------------------------------*/

static inline
void nrf24l01_delay_us (int us)
{

    /*
     * Cortex-M3 72MHz
     */
    us = us*72;
    while (--us);
}


#endif /* NRF24L01_CFG_H_ */
