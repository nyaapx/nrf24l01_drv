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
#include "hal_nrf.h"

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

int nrf24l01_set_chan (uint8_t chan)
{
    if (chan <= 0x7F) {
        hal_nrf_set_rf_channel(chan);
        return 0;
    } else {
        return -1;
    }
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
    _rx_fifo_read(*p_pipe_num, p_out_buf);
    
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

int nrf24l01_tx (const uint8_t *p_data, uint8_t size)
{
    if (size > _FIFO_BYTES) {
        return -EINVAL;
    }

    chBSemWait(&_g_sem_tx);         /* Wait for binary semaphore NRFSemTX */
    
    nrf24l01_pin_ce_set(0);         /* Clear CE, ready for a positive pulse */
    _tx_fifo_write(p_data, size);   /* Write the data to FIFO */
    
    /*
     * CE positive pulse to send data packet
     */
    nrf24l01_pin_ce_set(1);
    nrf24l01_delay_us(10);
    nrf24l01_pin_ce_set(0);

    return 0;
}

int nrf24l01_init (nrf24l01_init_cfg_t *p_init_cfg)
{
    /*
     * Initialize the FIFO semaphores 
     */
    chBSemObjectInit(&_g_sem_irq, TRUE);  /* Locks the thread until an IRQ arrives */
    chBSemObjectInit(&_g_sem_rx,  TRUE);  /* Semaphore initialized as taken, because no data is ready yet */
    chBSemObjectInit(&_g_sem_tx,  FALSE); /* Semaphore initialized as free, because transmit channel is open */
    
    /*
     * Init CE
     */
    nrf24l01_pin_ce_out_pp_set();   /* CE pin push-pull */
    nrf24l01_pin_ce_set(0);         /* Cli CE to keep idle mode */

    /*
     * Set configuration registers:
     */
    _reg_write_byte(_REG_CONFIG,     _CONFIG_PWR_UP | _CONFIG_EN_CRC);  /* power up; enable crc */
    _reg_write_byte(_REG_SETUP_AW,   _SETUP_AW_5BYTES);                 /* address width 5 byte */

    /*
     * Reset the IRQ registers, write 1 to clear bit.
     */
    _reg_write_byte(_REG_STATUS, _STATUS_RX_DR | _STATUS_TX_DS | _STATUS_MAX_RT);

    /*
     * Config pipe 0 for rx
     */
    _reg_write_byte(_REG_EN_AA,      _ENAA_P0); /* Enhanced ShockBurst enable */
    _reg_write_byte(_REG_EN_RXADDR,  _ERX_P0);  /* Pipe enable */
    _reg_write_byte(_REG_RX_PW_P0,   0);        /* no payload */

    /*
     * Enhanced ShockBurst Config
     */
    _reg_write_byte(_REG_SETUP_RETR, (_SETUP_RETR_ARD & (1 << 4)) |
                                     (_SETUP_RETR_ARC & (0x03)  )); /* Up to 3 Re-Transmit, Wait 500μS */
    _reg_write_byte(_REG_RF_SETUP,   _RF_SETUP_LNA_HCURR   |
                                     _RF_SETUP_RF_PWR_0DBM);
    _reg_write_byte(_REG_DYNPD, _DYNPD_DPL_P1 | _DYNPD_DPL_P2 | _DYNPD_DPL_P3 |
                                _DYNPD_DPL_P4 | _DYNPD_DPL_P5);
    _reg_write_byte(_REG_FEATURE, _FEATURE_EN_DPL);
    _activate_cmd();    /* enable "features", for Dynamic Layload Length */

    /*
     * Start a thread for handle IRQ
     */
    _start_thread();
    
    _g_init_done_flg = TRUE;
    return 0;
}

int nrf24l01_test (void)
{
    uint8_t old_val, new_val;

    /*
     * IO Test
     */
    //todo:

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

void nrf24l01_report_irq (void)
{
    if (_g_init_done_flg) {
        chBSemSignalI(&_g_sem_irq);     /* Unlock the IRQ Semaphore */
    }
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

