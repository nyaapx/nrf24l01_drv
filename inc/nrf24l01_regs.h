
/*
 * Macros
 */
#define _BIT(n)                          (1 << n)

/*
 * NRF24L01 commands
 */
#define _CMD_READREG                     0x00             /* 0b000A AAAA read register  A AAAA */
#define _CMD_WRITEREG                    0x20             /* 0b001A AAAA write register A AAAA */
#define _CMD_RX_PAYLOAD                  0x60             /* Read RX-payload */
#define _CMD_TX_PAYLOAD                  0xA0             /* Write TX-payload */
#define _CMD_FLUSH_TX                    0xE1             /* Flush TX FIFO */
#define _CMD_FLUSH_RX                    0xE2             /* Flush RX FIFO */
#define _CMD_REUSE_TX                    0xE3             /* Reuse last transmitted payload */
#define _CMD_ACTIVATE                    0x50             /* Activate the features */
#define _CMD_R_RX_PL_WID                 0x60             /* Read RX-payload width */
#define _CMD_W_ACK_PAYLOAD               0x58             /* 0b10101PPP valid from 000 to 101 */
#define _CMD_W_TX_PAYLOAD_NOACK          0xB0             /* Disables AUTOACK on packet */
#define _CMD_NOP                         0xFF             /* NOP (No operation), read STATUS reg */

/*
 * NRF24L01 registers
 */
#define _REG_CONFIG                      0x00             /* Configuration Register */
#define _REG_EN_AA                       0x01             /* Auto Acknowledgment */
#define _REG_EN_RXADDR                   0x02             /* Enabled RX Addresses */
#define _REG_SETUP_AW                    0x03             /* Setup of Address Widths */
#define _REG_SETUP_RETR                  0x04             /* Setup of Automatic Retransmission */
#define _REG_RF_CH                       0x05             /* RF Channel */
#define _REG_RF_SETUP                    0x06             /* RF Setup Register */
#define _REG_STATUS                      0x07             /* Status Register */
#define _REG_OBSERVE_TX                  0x08             /* Transmit observe register */
#define _REG_CD                          0x09             /* Carrier Detect */
#define _REG_RX_ADDR_P0                  0x0A             /* Receive address data pipe 0 */
#define _REG_RX_ADDR_P1                  0x0B             /* Receive address data pipe 1 */
#define _REG_RX_ADDR_P2                  0x0C             /* Receive address data pipe 2 */
#define _REG_RX_ADDR_P3                  0x0D             /* Receive address data pipe 3 */
#define _REG_RX_ADDR_P4                  0x0E             /* Receive address data pipe 4 */
#define _REG_RX_ADDR_P5                  0x0F             /* Receive address data pipe 5 */
#define _REG_TX_ADDR                     0x10             /* Transmit address */
#define _REG_RX_PW_P0                    0x11             /* Number of bytes in RX payload */
#define _REG_RX_PW_P1                    0x12             /* Number of bytes in RX payload */
#define _REG_RX_PW_P2                    0x13             /* Number of bytes in RX payload */
#define _REG_RX_PW_P3                    0x14             /* Number of bytes in RX payload */
#define _REG_RX_PW_P4                    0x15             /* Number of bytes in RX payload */
#define _REG_RX_PW_P5                    0x16             /* Number of bytes in RX payload */
#define _REG_FIFO_STATUS                 0x17             /* FIFO Status Register */
#define _REG_DYNPD                       0x1C             /* Enable dynamic payload length */
#define _REG_FEATURE                     0x1D             /* Feature Register */

/*
 * configuration register
 */
#define _CONFIG_MASK_RX_DR               _BIT(6)          /* Mask interrupt caused by RX_RD */
#define _CONFIG_MASK_TX_DS               _BIT(5)          /* Mask interrupt caused by TX_DS */
#define _CONFIG_MASK_MAX_RT              _BIT(4)          /* Mask interrupt caused by MAX_RT */
#define _CONFIG_EN_CRC                   _BIT(3)          /* Enable CRC */
#define _CONFIG_CRCO                     _BIT(2)          /* CRC encoding scheme */
#define _CONFIG_PWR_UP                   _BIT(1)          /* 1: POWER UP, 0:POWER DOWN */
#define _CONFIG_PRIM_RX                  _BIT(0)          /* 1: PRX, 0: PTX */

/*
 * enable auto acknowledgment
 */
#define _ENAA_P5                         _BIT(5)          /* Pipe 5 Auto ACK Enable */
#define _ENAA_P4                         _BIT(4)          /* Pipe 4 Auto ACK Enable */
#define _ENAA_P3                         _BIT(3)          /* Pipe 3 Auto ACK Enable */
#define _ENAA_P2                         _BIT(2)          /* Pipe 2 Auto ACK Enable */
#define _ENAA_P1                         _BIT(1)          /* Pipe 1 Auto ACK Enable */
#define _ENAA_P0                         _BIT(0)          /* Pipe 0 Auto ACK Enable */

/*
 * enable rx addresses
 */
#define _ERX_P5                          _BIT(5)          /* Enable data pipe 5 */
#define _ERX_P4                          _BIT(4)          /* Enable data pipe 4 */
#define _ERX_P3                          _BIT(3)          /* Enable data pipe 3 */
#define _ERX_P2                          _BIT(2)          /* Enable data pipe 2 */
#define _ERX_P1                          _BIT(1)          /* Enable data pipe 1 */
#define _ERX_P0                          _BIT(0)          /* Enable data pipe 0 */

/*
 * setup of address width (for all data pipes)
 */
#define _SETUP_AW                        0x03             /* mask of address width */
#define _SETUP_AW_3BYTES                 1                /* 3 bytes address width */
#define _SETUP_AW_4BYTES                 2                /* 4 bytes address width */
#define _SETUP_AW_5BYTES                 3                /* 5 bytes address width */

/*
 * setup of auto re-transmission mask
 */
#define _SETUP_RETR_ARD                  0xF0             /* Auto Retransmit Delay mask, wait (250*(val+1)+86)us */
#define _SETUP_RETR_ARC                  0x0F             /* Auto Retransmit Count mask, set 0 to disable retransmit */

/*
 * RF Channel
 */
#define _RF_CH                           0x7F             /* the frequency channel */

/*
 * RF setup register
 */
#define _RF_SETUP_PLL_LOCK               _BIT(4)          /* Force PLL lock signal */
#define _RF_SETUP_RF_DR                  _BIT(3)          /* Data Rate: '0'-1Mbps, '1'-2Mbps  */
#define _RF_SETUP_RF_PWR                 0x03             /* RF output power in TX mode mask */
#define _RF_SETUP_RF_PWR_N18DBM          0                /* TX power -18dBm */
#define _RF_SETUP_RF_PWR_N12DBM          1                /* TX power -12dBm */
#define _RF_SETUP_RF_PWR_N6DBM           2                /* TX power -6dBm */
#define _RF_SETUP_RF_PWR_0DBM            3                /* TX power 0dBm */
#define _RF_SETUP_LNA_HCURR              _BIT(0)          /* Setup LNA gain */

/*
 * status register masks
 */
#define _STATUS_RX_DR                    _BIT(6)          /* Data ready RX FIFO interrupt */
#define _STATUS_TX_DS                    _BIT(5)          /* Data sent interrupt */
#define _STATUS_MAX_RT                   _BIT(4)          /* Maximum number of TX retries */
#define _STATUS_RX_R_NO                  0x0E             /* Data pipe number for payload */
#define _STATUS_TX_FULL                  _BIT(0)          /* TX FIFO full flag */

/*
 * transmit observe register masks
 */
#define _OBSERVE_TX_PLOS_CNT             0xF0             /* Packet Loss Counter */
#define _OBSERVE_TX_ARC_CNT              0x0F             /* Current value on resent counter */

/*
 * Carrier Detect
 */
#define _CD                              _BIT(0)          /* Carrier Detect */

/*
 * FIFO status mask
 */
#define _FIFO_STATUS_TX_REUSE            _BIT(6)          /* Reuse lost sent data packet high */
#define _FIFO_STATUS_TX_FULL             _BIT(5)          /* TX FIFO full flag */
#define _FIFO_STATUS_TX_EMPTY            _BIT(4)          /* TX FIFO empty flag */
#define _FIFO_STATUS_RX_FULL             _BIT(1)          /* RX FIFO full flag */
#define _FIFO_STATUS_RX_EMPTY            _BIT(0)          /* RX FIFO empty flag */

#define _DYNPD_DPL_P5                    _BIT(5)          /* Enable dyn. payload length data pipe 5 */
#define _DYNPD_DPL_P4                    _BIT(4)          /* Enable dyn. payload length data pipe 4 */
#define _DYNPD_DPL_P3                    _BIT(3)          /* Enable dyn. payload length data pipe 3 */
#define _DYNPD_DPL_P2                    _BIT(2)          /* Enable dyn. payload length data pipe 2 */
#define _DYNPD_DPL_P1                    _BIT(1)          /* Enable dyn. payload length data pipe 1 */
#define _DYNPD_DPL_P0                    _BIT(0)          /* Enable dyn. payload length data pipe 0 */

#define _FEATURE_EN_DPL                  _BIT(2)          /* Enables Dynamic Payload Length */
#define _FEATURE_EN_ACK_PAY              _BIT(1)          /* Enables Payload with ACK */
#define _FEATURE_EN_DYN_ACK              _BIT(0)          /* Enables the W_TX_PAYLOAD_NOACK command */
