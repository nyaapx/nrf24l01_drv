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

#include "nrf24l01.h"

#include "nrf24l01_regs.h"
#include "nrf24l01_cfg.h"

#include "ch.h"
#include "hal.h"

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>


/*
 * The number of bytes the NRF24L01 RX FIFO is going to hold
 * This can be 32 bytes MAX
 */
#define _FIFO_BYTES 32

#define _BUF_SIZE   128

/*
 * Binary semaphore, to lock access to the send and receive queue of the NRF24L01
 */
static binary_semaphore_t _g_sem_irq, _g_sem_rx, _g_sem_tx;

/*
 * Status flags
 */
typedef struct {
    uint8_t max_rt; /* Set the flag when the max retransmit is reached. */
} _stat_t;
static _stat_t _g_stat;

/*
 * Flag to signal if init is done
 */
static bool _g_init_done_flg;

static void _start_thread (void);

/*
 * Reverse buffer (Used to reverse addresses)
 */
//static
//void _reverse_buf (uint8_t in[], uint8_t out[], size_t size)
//{
//    size_t count=0;
//
//    for(count=0; count < size; count++) {
//        out[count]=in[size-(count+1)];
//    }
//}

static
void _reg_read (uint8_t reg, uint8_t *p_buf, size_t size)
{
    uint8_t first_byte;

    /*
     * Build command to read register
     */
    first_byte = _CMD_READREG | reg;

    nrf24l01_spi_tx_rx(first_byte, p_buf, size);
}

/*
 * ACTIVATE SPI command followed by data 0x73, enable/disable features:
 *  R_RX_PL_WID
 *  W_ACK_PAYLOAD
 *  W_TX_PAYLOAD_NOACK
 */
static
void _activate_cmd (void)
{
    uint8_t tx_buf[2], rx_buf[2];

    tx_buf[0] = _CMD_ACTIVATE;     /* ACTIVATE command */
    tx_buf[1] = 0x73;
    nrf24l01_spi_exchange(tx_buf, rx_buf, 2);
}

/*
 * Read single byte of register
 */
static
uint8_t _reg_read_byte (uint8_t reg)
{
    uint8_t tx_buf[2], rx_buf[2];

    /*
     * exchange 2 bytes, the second byte is the value of specified register
     */
    tx_buf[0] = _CMD_READREG | reg;     /* read register command */
    tx_buf[1] = 0;                      /* useless byte */
    nrf24l01_spi_exchange(tx_buf, rx_buf, 2);
    return rx_buf[1];
}

/*
 * Write several bytes to register
 */
static
void _reg_write (uint8_t reg, const uint8_t *p_val, uint8_t size)
{
    uint8_t first_byte;

    first_byte = _CMD_WRITEREG | reg;           /* write register command */
    nrf24l01_spi_tx_tx(first_byte, p_val, size);
}

/*
 * Write single byte to register
 */
static
void _reg_write_byte (uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[2], rx_buf[2];

    tx_buf[0] = _CMD_WRITEREG | reg;            /* write register command */
    tx_buf[1] = val;
    nrf24l01_spi_exchange(tx_buf, rx_buf, 2);
}

/*
 * Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO.
 */
static
uint8_t _rx_payload_width_read (void)
{
    uint8_t tx_buf[2], rx_buf[2];

    tx_buf[0] = _CMD_WRITEREG | reg;            /* write register command */
    tx_buf[1] = 0;
    nrf24l01_spi_exchange(tx_buf, rx_buf, 2);
    return rx_buf[1];
}

int nrf24l01_set_chan (uint8_t chan)
{
    if ((chan & _RF_CH) != chan) {
        return -EINVAL;
    }
    _reg_write_byte(_REG_RF_CH, chan);
}

/*
 * Set the address to the receiver pipe
 * Normaly pipe  is used to receive the ack packets by shockburst
 * Use pipe 1 as the first data receive pipe
 * @Arguments
 * pipe                Pipe number to set the address to
 * addr_size           The size of the address in bytes
 * addr                Byte array holding the addr, LSB first
 */
static
void _rx_addr_set (uint8_t pipe, uint8_t *p_addr, uint8_t addr_size)
{
    uint8_t pipeAddr;

    /*
     * Create command
     */
    pipeAddr = _CMD_WRITEREG | (_REG_RX_ADDR_P0 + pipe);

    _reg_write(pipeAddr, p_addr, addr_size);
}

/*
 * Set the address to the receiver pipe
 * @Arguments
 * pipe                Pipe number to set the address to
 * addr_size    The size of the address in bytes
 * addr                Byte array holding the address, LSB first
 */
void _tx_addr_set (uint8_t *p_addr, uint8_t addr_size)
{
    uint8_t pipeAddr;
    
    /*
     * Set pipe 0 address identical to send address,
     * this to enable the automatic shockburst handling of ack's
     */
    pipeAddr = _CMD_WRITEREG | _REG_RX_ADDR_P0;
    _reg_write(pipeAddr, p_addr, addr_size);
    
    /*
     * Set the TX pipe address
     */
    pipeAddr = _CMD_WRITEREG | _REG_TX_ADDR;
    _reg_write(pipeAddr, p_addr, addr_size);
}

/*
 * Get Status from inside interrupt routine
 */
static
uint8_t _status_get (void)
{
    uint8_t command[2], result[2];
    
    /*
     * Set NOP and receive the STATUS register
     */
    command[0] = _CMD_NOP;
    nrf24l01_spi_exchange(command, result, 2);
    
    return result[0];
}

/*
 * Reset status flags inside interrupt routine
 */
static
void _status_reset (uint8_t stat_mask)
{
    uint8_t command[2], result[2];
    
    /*
     * Set NOP and receive the STATUS register
     */
    command[0] = _CMD_WRITEREG | _REG_STATUS;
    command[1] = stat_mask;
    nrf24l01_spi_exchange(command, result, 2);
}

/*
 * Flush the TX Queue
 */
static
void _flush_tx (void)
{
    uint8_t command;
    uint8_t result;
    
    /*
     * Set NOP and receive the STATUS register
     */
    command = _CMD_FLUSH_TX;
    nrf24l01_spi_exchange(&command, &result, 1);
}

/*
 * Handle the IRQ signal, unlock the Semaphores or set flags. 
 */
static
void _irq_handler (void)
{
    uint8_t reset_flags = 0;
    uint8_t status = 0;
    
    /*
     * Wait for the semaphore
     */
    chBSemWait(&_g_sem_irq);
    
    /*
     * Execute NOP to retrieve the status register
     */
    status = _status_get();
    
    /*
     * Data ready in FIFO
     * Signal binary semaphore data can be retrieved
     */
    if ((_STATUS_RX_DR & status) != 0) {
        reset_flags |= _STATUS_RX_DR; /* Set flag for reset */
        
        chBSemSignal(&_g_sem_rx);
    }
    
    /*
     * Max number of TX retries, set flag
     * Also clear flag or no further communication is possible
     */
    if ((_STATUS_MAX_RT & status) != 0) {
        reset_flags |= _STATUS_MAX_RT; /* Set flag for reset */
        
        /*
         * Set the MAX TX flag
         * Flush out the queue, because we can't send them anyway.
         */
        _g_stat.max_rt = 1;
        _flush_tx();
        
        /*
         * Signal the semaphore because sending has failed
         */
        chBSemSignal(&_g_sem_tx);
    }
    
    /*
     * Called when the TX_FIFO is full
     * Take TX semaphore, until a data sent interrupt signals the semaphore again
     */
    if ((_STATUS_TX_FULL & status) != 0) {
        reset_flags |= _STATUS_TX_FULL; /* Set flag for reset */
        
        /*
         * Lock the semaphore, because the TX queue is full
         */
        chBSemWait(&_g_sem_tx);
        
    } else {
        /*
         * TX Queue not full.
         */
        chBSemSignal(&_g_sem_tx);
    }
    
    /*
     * Reset the asserted interrupt flags in the register
     */
    _status_reset(reset_flags);
}

void nrf24l01_report_irq (void)
{
    if (_g_init_done_flg) {
        chBSemSignalI(&_g_sem_irq);     /* Unlock the IRQ Semaphore */
    }
}

/*
 * NRF read RX FIFO, p_out_buf 32 bytes
 * return the size of data
 */
static
uint8_t _rx_fifo_read (uint8_t pipe, uint8_t *p_out_buf)
{
    uint8_t command, len;
    
    /*
     * Read the rx fifo payload wide of specified pipe
     */
    len = //todo: 动态

    /*
     * Build command and send retreive RX command to NRF24L01
     */
    command = _CMD_RX_PAYLOAD;
    nrf24l01_spi_tx_rx(command, p_out_buf, len);

    return len;
}

/*
 * NRF Write TX FIFO
 */
static
void _tx_fifo_write (const uint8_t *p_in_buf)
{
    uint8_t command[33], bogus[33];
    
    /*
     * Build command and send retrieve RX command to NRF24L01
     */
    command[0] = _CMD_TX_PAYLOAD;
    memcpy(command + 1, p_in_buf, _FIFO_BYTES); //todo: dynamic length
    nrf24l01_spi_exchange(command, bogus, _FIFO_BYTES);
}

/*
 * NRF RX FIFO empty
 * Returns FALSE if full, TRUE if empty
 */
static
bool _rx_fifo_empty (void)
{
    uint8_t command = 0, result = 0;
    
    /*
     * Build command to read status register
     */
    command = _CMD_READREG | _REG_FIFO_STATUS;
    nrf24l01_spi_exchange(&command, &result, 1);
    
    if ((result & _FIFO_STATUS_RX_EMPTY) != 0) {
        return TRUE; /* FIFO empty */
    } else {
        return FALSE;
    }
}

/*
 * Function to receive data from FIFO
 * This functions blocks until data is available
 * The output buffer needs to be _FIFO_BYTES(32) bytes wide
 */
int nrf24l01_rx (uint32_t timeout, uint8_t *p_pipe_num, uint8_t *p_out_buf)
{
    systime_t timeout_systime;
    uint8_t   statusReg = 0;

    /*
     * Set NRF24L01 to receive mode.
     */
    nrf24l01_pin_ce_set(1);
    
    /*
     * Wait for binary semaphore NRFSemRX
     */
    if (0 == timeout) {
        timeout_systime = TIME_INFINITE;
    } else {
        timeout_systime = MS2ST(timeout);
    }

    if (MSG_TIMEOUT == chBSemWaitTimeout(&_g_sem_rx, timeout_systime)) {
        return -1;
    }
    
    /*
     * Bring CE down, in order to execute the read operation
     */
    nrf24l01_pin_ce_set(0);
    
    /*
     * Get the status register and distill the RX PIPE number
     */
    statusReg = _status_get();
    *p_pipe_num = (statusReg & _STATUS_RX_R_NO) >> 1;
    
    /*
     * Retreive data from fifo
     */
    _rx_fifo_read(p_out_buf);
    
    /*
     * Bring CE down, in order to execute the read operation
     */
    nrf24l01_pin_ce_set(1);
    
    /*
     * When there is still data in the pipe 1, 
     * signal the semaphore
     * When the FIFO is empty the semaphore stays taken until the next interrupt
     */
    if (!_rx_fifo_empty()) {
        chBSemSignal(&_g_sem_rx);
    }

    return 0;
}

/*
 * Function to send data
 * This functions blocks until data is available
 * The send output buffer needs to be _FIFO_BYTES(32) bytes wide
 */
int nrf24l01_tx (const uint8_t *p_data, uint8_t size)
{
    /*
     * Wait for binary semaphore NRFSemTX
     */
    chBSemWait(&_g_sem_tx);
    
    /*
     * Put CE low.
     */
    nrf24l01_pin_ce_set(0);
    
    /*
     * Send the data to pipe 1 (the out pipe)
     */
    _tx_fifo_write(p_data);
    
    /*
     * Toggle the CE Pad in order to send data packet
     */
    nrf24l01_pin_ce_set(1);
    nrf24l01_delay_us(10);
    nrf24l01_pin_ce_set(0);

    return 0;
}

/*
 * Initialize the NRF24L01 chip
 */
int nrf24l01_init (void)
{
    /*
     * Initialize the FIFO semaphores 
     */
    chBSemObjectInit(&_g_sem_irq, TRUE);  /* Locks the thread until an IRQ arrives */
    chBSemObjectInit(&_g_sem_rx,  TRUE);  /* Semaphore initialized as taken, because no data is ready yet */
    chBSemObjectInit(&_g_sem_tx,  FALSE); /* Semaphore initialized as free, because transmit channel is open */
    
    /*
     * Set configuration registers
     */
    _reg_write_byte(_REG_CONFIG, 0b00001110); /* POWER_UP, ENABLE CRC */
    _reg_write_byte(_REG_EN_AA,  0b00000011); /* Enhanced ShockBurst on channel 0,1 */
    _reg_write_byte(_REG_EN_RXADDR, 0b00000011); /* Enable data pipe 0,1 */
    _reg_write_byte(_REG_SETUP_AW, 0b00000011); /* 5 bytes address width */
    _reg_write_byte(_REG_SETUP_RETR, 0b00010011); /* Up to 3 Re-Transmit, Wait 500μS */
    _reg_write_byte(_REG_RF_SETUP, 0b00000111); /* Sets up the channel we work on */
    _reg_write_byte(_REG_STATUS, 0b01110000); /* Reset the IRQ registers. */
//    _reg_write_byte(_REG_RX_PW_P0, _FIFO_BYTES); /* Pipe 0 FIFO holds 32 bytes. */
//    _reg_write_byte(_REG_RX_PW_P1, _FIFO_BYTES); /* Pipe 1 FIFO holds 32 bytes. */

    _reg_write_byte(_REG_FEATURE, _FEATURE_EN_DPL);
    _reg_write_byte(_REG_DYNPD, _DYNPD_DPL_P0 | _DYNPD_DPL_P1);
    
    _activate_cmd();    /* enable "features", for Dynamic Layload Length */

    /*
     * Setup the pads for the EC pin
     */
    nrf24l01_pin_ce_out_pp_set();
    
    /*
     * Set CE to high, to put NRF24L01 into Receive mode.
     */
    nrf24l01_pin_ce_set(1);
    
    //todo: read id to do test

    _g_init_done_flg = TRUE;

    _start_thread();

    return 0;
}

int nrf24l01_test (void)
{
    uint8_t old_val, new_val;

    /*
     * Operate the register 0x05(RF_CH, RF Channel) to do the test.
     * 1. read, save its value to 'old_val'
     * 2. write, set a new value
     * 3. read, check if the value is modified
     * 4. write, recover from 'old_val'
     * 5. read, check again
     */
    old_val = _reg_read_byte(_REG_RF_CH);

    if (old_val == 100) {
        new_val = 50;
    } else {
        new_val = 100;
    }

    new_val = (old_val == 100) ? 50 : 100;

    _reg_write_byte(_REG_RF_CH, new_val);

    if ((_reg_read_byte(_REG_RF_CH) & _RF_CH) != new_val) {
        return -1;
    }

    _reg_write_byte(_REG_RF_CH, old_val);

    if ((_reg_read_byte(_REG_RF_CH) & _RF_CH) != old_val) {
        return -1;
    }

    return 0;
}

/* Driver Thread -------------------------------------------------------------*/

static THD_WORKING_AREA(NRFThreadWA, 256);

static
void _drv_thread (void *p_arg)
{
    (void) p_arg;

    chRegSetThreadName("NRF24L01");
    while (TRUE) {
        _irq_handler();
    }
}

/*
 * Start Driver Thread
 */
static
void _start_thread (void)
{
    chThdCreateStatic(NRFThreadWA,              /* working area */
                      sizeof(NRFThreadWA),      /* size of working area */
                      NORMALPRIO,               /* priority of thread */
                      _drv_thread,              /* entry point of thread */
                      NULL);                    /* p_arg of entry point */
}

