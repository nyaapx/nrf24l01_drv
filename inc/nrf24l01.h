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

#include "nrf24l01_cfg.h"

/* defines for nrf24l01_cfg.h ------------------------------------------------*/
/*
 * address length
 */
#define NRF24L01_CFG_ADDR_LENGTH_3BYTE      0
#define NRF24L01_CFG_ADDR_LENGTH_4BYTE      1
#define NRF24L01_CFG_ADDR_LENGTH_5BYTE      2

#ifndef NRF24L01_CFG_ADDR_LENGTH
#define NRF24L01_CFG_ADDR_LENGTH            NRF24L01_CFG_ADDR_LENGTH_5BYTE
#else

    #if ((NRF24L01_CFG_ADDR_LENGTH != NRF24L01_CFG_ADDR_LENGTH_3BYTE) &&    \
         (NRF24L01_CFG_ADDR_LENGTH != NRF24L01_CFG_ADDR_LENGTH_4BYTE) &&    \
         (NRF24L01_CFG_ADDR_LENGTH != NRF24L01_CFG_ADDR_LENGTH_5BYTE))
        #error "NRF24L01_CFG_ADDR_LENGTH error!"
    #endif

#endif

/*
 * data rate
 */
#define NRF24L01_CFG_DATA_RATE_1MBPS        0
#define NRF24L01_CFG_DATA_RATE_2MBPS        1

#ifndef NRF24L01_CFG_DATA_RATE
#define NRF24L01_CFG_DATA_RATE              NRF24L01_CFG_DATA_RATE_2MBPS
#else
    #if ((NRF24L01_CFG_DATA_RATE != NRF24L01_CFG_DATA_RATE_1MBPS) &&        \
         (NRF24L01_CFG_DATA_RATE != NRF24L01_CFG_DATA_RATE_2MBPS))
        #error "NRF24L01_CFG_DATA_RATE error!"
    #endif
#endif

/*
 * CRCO CRC encoding scheme
 */
#define NRF24L01_CFG_CRCO_1BYTE             0
#define NRF24L01_CFG_CRCO_2BYTE             1

#ifndef NRF24L01_CFG_CRCO
#define NRF24L01_CFG_CRCO                   NRF24L01_CFG_CRCO_1BYTE
#else
    #if ((NRF24L01_CFG_CRCO != NRF24L01_CFG_CRCO_1BYTE) &&        \
         (NRF24L01_CFG_CRCO != NRF24L01_CFG_CRCO_2BYTE))
        #error "NRF24L01_CFG_CRCO error!"
    #endif
#endif

/*
 * Auto Retransmit Delay
 */
#define NRF24L01_CFG_ARD_250US              0
#define NRF24L01_CFG_ARD_500US              1
#define NRF24L01_CFG_ARD_750US              2
#define NRF24L01_CFG_ARD_1000US             3
#define NRF24L01_CFG_ARD_1250US             4
#define NRF24L01_CFG_ARD_1500US             5
#define NRF24L01_CFG_ARD_1750US             6
#define NRF24L01_CFG_ARD_2000US             7
#define NRF24L01_CFG_ARD_2250US             8
#define NRF24L01_CFG_ARD_2500US             9
#define NRF24L01_CFG_ARD_2750US             10
#define NRF24L01_CFG_ARD_3000US             11
#define NRF24L01_CFG_ARD_3250US             12
#define NRF24L01_CFG_ARD_3500US             13
#define NRF24L01_CFG_ARD_3750US             14
#define NRF24L01_CFG_ARD_4000US             15

#ifndef NRF24L01_CFG_ARD
#define NRF24L01_CFG_ARD                    NRF24L01_CFG_ARD_250US
#else
    #if (NRF24L01_CFG_ARD > NRF24L01_CFG_ARD_4000US)
        #error "NRF24L01_CFG_ARD error!"
    #endif
#endif

/*
 * Auto Retransmit Count
 */
#ifndef NRF24L01_CFG_ARC
#define NRF24L01_CFG_ARC                    3
#else
    #if (NRF24L01_CFG_ARC > 15)
        #error "NRF24L01_CFG_ARC error!"
    #endif
#endif

/*
 * Setup LNA gain
 */
#ifndef NRF24L01_CFG_LNA_ENABLE
#define NRF24L01_CFG_LNA_ENABLE             1
#endif

/* defines -------------------------------------------------------------------*/
#define NRF24L01_PIPE_CFG_ENABLE        (1 << 0)
#define NRF24L01_PIPE_CFG_DYNAMIC       (1 << 1)
#define NRF24L01_PIPE_CFG_AUTO_ACK      (1 << 2)

/**
 * @brief Pipe configuration
 */
typedef struct {
    uint8_t  pipe_enable;     /**< Piep enable */
    uint8_t  dynamic_enbale;  /**< Dynamic Payload Length enable */
    uint8_t  auto_ack_enable; /**< Auto ACK enable */
    uint8_t  payload_width;   /**< payload width, unavailable in dynamic mode */
    uint8_t *p_addr;          /**< address of rx pipe */
} nrf24l01_pipe_cfg_t;

/**
 * @brief RF output power in TX mode
 */
typedef enum {
    NRF24L01_TX_POWER_M18,  /**< tx power -18dBm  */
    NRF24L01_TX_POWER_M12,  /**< tx power -12dBm  */
    NRF24L01_TX_POWER_M6,   /**< tx power -6dBm  */
    NRF24L01_TX_POWER_0,    /**< tx power 0dBm  */
} nrf24l01_tx_power_t;

/* function prototypes -------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   Initialize the NRF24L01 RF chip.
 * @return  0 on success, negative error code on fail
 */
int nrf24l01_init (void);

//int nrf24l01_tx_cfg ();
//
//int nrf24l01_rx_cfg ();

/**
 * @brief   Write and read a register to test whether the chip works
 * @return  0 on success, negative error code on fail
 */
int nrf24l01_test (void);

/**
 * @brief   Set power down
 * @retval  0 on success, negative error code on fail
 */
int nrf24l01_power_down (void);

/**
 * @brief       Config a rx pipe.
 * @param[in]   pipe        number of  pipe
 * @param[in]   cfg         pipe configuration
 * @retval      0 on success, negative error code on fail
 */
int nrf24l01_pipe_cfg (uint8_t pipe, const nrf24l01_pipe_cfg_t *p_cfg);

/**
 * @brief       Enable or disable a rx pipe.
 * @param[in]   pipe        number of  pipe
 * @param[in]   enable      '0' for disable and '1' for enable
 * @retval      0 on success, negative error code on fail
 */
int nrf24l01_pipe_enable (uint8_t pipe, uint8_t enable);

/**
 * @brief       Set RF Channel
 * @details     Channel     Frequency is F0 = 2400 + RF_CH [MHz]
 * @param[in]   chan        channel number
 * @retval      0 on success, negative error code on fail
 */
int nrf24l01_set_chan (uint8_t chan);


int nrf24l01_tx_power_set (nrf24l01_tx_power_t pwr);

/*
 * Function to send data
 * This functions blocks until data is available
 * The send output buffer needs to be NRF_FIFO_BYTES(32) bytes wide
 */
int nrf24l01_tx (const uint8_t *p_data, uint8_t size);

/**
 * @brief   NRF24L01 Receive
 *
 * @details function to receive data from FIFO,
 *          blocks until data is available or timeout.
 *
 * @param[in]   timeout         timeout, unit ms
 * @param[out]  p_pipe_num      pipe number of rx pipe
 * @param[out]  p_out_buf       rx data, must to be 32 bytes wide
 *
 * @return  size of rx data, negative on fail.
 */
int nrf24l01_rx (uint32_t timeout, uint8_t *p_pipe_num, uint8_t *p_out_buf);

/*
 * Routine to unlock IRQ handling.
 */
void nrf24l01_report_irq (void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  /* NRF24L01_H_ */
