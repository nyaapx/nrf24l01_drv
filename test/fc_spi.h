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

#ifndef FC_SPI_H_
#define FC_SPI_H_

#include <stdint.h>

/**
 * @brief   Initialize the SPI interface
 */
void spi1_plfm_init (void);

/**
 * @brief   SPI sync send
 */
void spi1_tx (uint8_t *p_tx, size_t size);

/**
 * @brief   SPI sync receive
 */
void spi1_rx (uint8_t *p_rx, size_t size);

/**
 * @brief   SPI sync exchange
 */
void spi1_exchange (const uint8_t *p_tx, uint8_t *p_rx, size_t size);

void spi1_tx_and_rx (uint8_t tx_byte, uint8_t *p_rx, size_t rx_size);

void spi1_tx_and_tx (uint8_t tx_byte, const uint8_t *p_tx, size_t tx_size);

#endif /* FC_SPI_H_ */
