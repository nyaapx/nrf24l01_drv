/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/
#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <stdint.h>
#include <errno.h>

/* defines -------------------------------------------------------------------*/
typedef enum {
    NRF24L01_MODE_PRX,
    NRF24L01_MODE_PTX,
} nrf24l01_mode_t;

typedef enum {
    NRF24L01_DATA_RATE_250KBPS,
    NRF24L01_DATA_RATE_1MBPS,
    NRF24L01_DATA_RATE_2MBPS
} nrf24l01_data_rate_t;

typedef struct {
    nrf24l01_mode_t      mode;          /**< Enhance ShockBurst Mode */
    nrf24l01_data_rate_t data_rate;     /**< Date Rate */
} nrf24l01_init_cfg_t;

/**
 * @brief Pipe configuration
 */
typedef struct {
    uint8_t  p_addr[5];       /**< address of rx pipe */
    uint8_t  pipe_enable;     /**< Piep enable */
    uint8_t  auto_ack_enable; /**< Auto ACK enable */
} nrf24l01_pipe_cfg_t;

/**
 * @brief RF output power in TX mode
 */
typedef enum {
    NRF24L01_TX_POWER_M18,  /**< tx power -18dBm  */
    NRF24L01_TX_POWER_M12,  /**< tx power -12dBm  */
    NRF24L01_TX_POWER_M6,   /**< tx power -6dBm  */
    NRF24L01_TX_POWER_0     /**< tx power 0dBm  */
} nrf24l01_tx_power_t;

typedef enum {
    NRF24L01_RETRY_DELAY_250US,
    NRF24L01_RETRY_DELAY_500US,
    NRF24L01_RETRY_DELAY_750US,
    NRF24L01_RETRY_DELAY_1000US,
    NRF24L01_RETRY_DELAY_1250US,
    NRF24L01_RETRY_DELAY_1500US,
    NRF24L01_RETRY_DELAY_1750US,
    NRF24L01_RETRY_DELAY_2000US,
    NRF24L01_RETRY_DELAY_2250US,
    NRF24L01_RETRY_DELAY_2500US,
    NRF24L01_RETRY_DELAY_2750US,
    NRF24L01_RETRY_DELAY_3000US,
    NRF24L01_RETRY_DELAY_3250US,
    NRF24L01_RETRY_DELAY_3500US,
    NRF24L01_RETRY_DELAY_3750US,
    NRF24L01_RETRY_DELAY_4000US
} nrf24l01_retry_delay_t;

/* function prototypes -------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief       Initialize the NRF24L01 RF chip.
 * @param[in]   p_init_cfg      initialization config
 * @return      0 on success, negative error code on fail
 */
int nrf24l01_init (nrf24l01_init_cfg_t *p_init_cfg);

/**
 * @brief       Write and read a register to test whether the chip works
 * @return      0 on success, negative error code on fail
 */
int nrf24l01_test (void);

/**
 * @brief       NRF24L01 PRX/PTX Mode Setting
 * @param[in]   mode        PRX or PTX mode
 * @retval      0 on success, negative error code on fail
 */
int nrf24l01_mode_set (nrf24l01_mode_t mode);

/**
 * @brief       Set power down
 * @retval      0 on success, negative error code on fail
 */
int nrf24l01_power_down (void);

/**
 * @brief       Config a rx pipe.
 * @param[in]   pipe        number of  pipe
 * @param[in]   cfg         pipe configuration
 * @retval      0 on success, negative error code on fail
 */
int nrf24l01_pipe_cfg (int pipe, const nrf24l01_pipe_cfg_t *p_cfg);

/**
 * @brief       Set RF Channel
 * @details     Channel     Frequency is F0 = 2400 + RF_CH [MHz]
 * @param[in]   chan        channel number
 * @retval      0 on success, negative error code on fail
 */
int nrf24l01_set_chan (int chan);

/**
 * @brief       Set tx power
 * @param[in]   pwr     power \ref nrf24l01_tx_power_t
 * @retval      0 on success, negative error code on fail
 */
int nrf24l01_tx_power_set (nrf24l01_tx_power_t pwr);

/**
 * @brief       NRF24L01 Send Data
 * @param[in]   p_data      data
 * @param[in]   size        size of data
 * @param[in]   retry       max re-transmit on fail of auto ack
 * @retval      0 on success, negative error code on fail
 */
int nrf24l01_tx (const uint8_t         *p_data,
                 int                    size,
                 int                    retry,
                 nrf24l01_retry_delay_t retry_delay);

/**
 * @brief       NRF24L01 Receive
 *
 * @details     function to receive data from FIFO,
 *              blocks until data is available or timeout.
 *
 * @param[in]   timeout         timeout, unit ms
 * @param[out]  p_pipe_num      pipe number of rx pipe
 * @param[out]  p_out_buf       rx data, must to be 32 bytes wide
 *
 * @return  size of rx data, negative on fail.
 */
int nrf24l01_rx (int timeout, int *p_pipe_num, uint8_t *p_out_buf);

/**
 * @brief       NRF24L01 Set ACK Payload Data
 * @param[in]   pipe        payload data fill to specified pipe
 * @param[in]   p_data      data
 * @param[in]   size        size of data
 * @retval      0 on success, negative error code on fail
 */
int nrf24l01_set_ack_data (int pipe, const uint8_t *p_data, int size);

/**
 * @brief       Post IRQ Event
 * @details     Call this function in ISR to report NRF24L01 interrupt.
 */
void nrf24l01_report_irq (void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  /* NRF24L01_H_ */
