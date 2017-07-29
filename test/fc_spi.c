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

#include "ch.h"
#include "hal.h"

#include "fc_spi.h"

#define _SPI1_PORT       IOPORT1
#define _SPI1_PIN_SCK    5
#define _SPI1_PIN_MISO   6
#define _SPI1_PIN_MOSI   7
#define _SPI1_PIN_NSS    4

/*
 * Maximum speed SPI configuration (18MHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig _spi1_cfg = {
    NULL,
    _SPI1_PORT,
    _SPI1_PIN_NSS,
    0x0020,                  /* STM32 SPI CR0, BR[5:3]=4 (72/16=4.5MHz) */
    0x0000                   /* STM32 SPI CR1 */
};

/******************************************************************************/
void spi1_plfm_init (void)
{
    palSetPadMode(_SPI1_PORT, _SPI1_PIN_SCK, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    palSetPadMode(_SPI1_PORT, _SPI1_PIN_MISO, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
    palSetPadMode(_SPI1_PORT, _SPI1_PIN_MOSI, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    palSetPadMode(_SPI1_PORT, _SPI1_PIN_NSS, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(_SPI1_PORT, _SPI1_PIN_NSS);
}

/******************************************************************************/
void spi1_exchange (const uint8_t *p_tx, uint8_t *p_rx, size_t size)
{
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &_spi1_cfg);
    spiSelect(&SPID1);
    spiExchange(&SPID1, size, p_tx, p_rx);
    spiUnselect(&SPID1);
    spiReleaseBus(&SPID1);
}

/******************************************************************************/
void spi1_tx (uint8_t *p_tx, size_t size)
{
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &_spi1_cfg);
    spiSelect(&SPID1);
    spiSend(&SPID1, size, p_tx);
    spiUnselect(&SPID1);
    spiStop(&SPID1);
    spiReleaseBus(&SPID1);
}

/******************************************************************************/
void spi1_rx (uint8_t *p_rx, size_t size)
{
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &_spi1_cfg);
    spiSelect(&SPID1);
    spiReceive(&SPID1, size, p_rx);
    spiUnselect(&SPID1);
    spiReleaseBus(&SPID1);
}

void spi1_tx_and_rx (uint8_t tx_byte, uint8_t *p_rx, size_t rx_size)
{
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &_spi1_cfg);
    spiSelect(&SPID1);
    spiSend(&SPID1, 1, &tx_byte);
    spiReceive(&SPID1, rx_size, p_rx);
    spiUnselect(&SPID1);
    spiReleaseBus(&SPID1);
}

void spi1_tx_and_tx (uint8_t tx_byte, const uint8_t *p_tx, size_t tx_size)
{
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &_spi1_cfg);
    spiSelect(&SPID1);
    spiSend(&SPID1, 1, &tx_byte);
    spiSend(&SPID1, tx_size, p_tx);
    spiUnselect(&SPID1);
    spiReleaseBus(&SPID1);
}
