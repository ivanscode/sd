/*
DEV_ID
*/
#define DEV_ID 0x00
#define EUI 0x01
/*
PAN Identifier and Short Address
*/
#define PANADR 0x03
/*
SYS_CFG System configuration bitmap
*/
#define SYS_CFG 0x04
/*
SYS_TIME System Time counter (5 octet)
*/
#define SYS_TIME 0x06
/*
TX_FCTRL Transmit frame control
*/
#define TX_FCTRL 0x08
#define TX_FCTRL_BR_OFF 13
#define TX_FCTRL_TR_OFF 15
#define TX_FCTRL_PRF_OFF 16
#define TX_FCTRL_PSR_OFF 18
#define TX_FCTRL_OFF_OFF 22
/*
TX_BUFFER Transmit Data buffer
*/
#define TX_BUFFER 0x09
/*
DX_TIME Delayed send or receive time
*/
#define DX_TIME 0x0A
/*
RX_FWTO Receive Frame Wait timeout period
*/
#define RX_FWTO 0x0C
/*
SYS_CTRL System Control register
*/
#define SYS_CTRL 0x0D
#define SYS_CTRL_TXSTRT_OFF 1
#define SYS_CTRL_TXDLYS_OFF 2
#define SYS_CTRL_WAIT4RESP_OFF 7
#define SYS_CTRL_RXENAB_OFF 8
#define SYS_CTRL_RXDLYE_OFF 9
/*
SYS_MASK System event mask register
*/
#define SYS_MASK 0x0E
/*
SYS_STATUS System event status register
*/
#define SYS_STATUS 0x0F
#define SYS_STATUS_TXFRB 0x10
#define SYS_STATUS_TXPRS 0x20
#define SYS_STATUS_TXPHS 0x40
#define SYS_STATUS_TXFRS 0x80
#define SYS_STATUS_ALL_TX (SYS_STATUS_TXFRB | SYS_STATUS_TXPRS | SYS_STATUS_TXPHS | SYS_STATUS_TXFRS)
#define SYS_STATUS_RXPRD 0x100
#define SYS_STATUS_RXSFDD 0x200
#define SYS_STATUS_LDEDONE 0x400
#define SYS_STATUS_RXPHD 0x800
#define SYS_STATUS_RXDFR 0x2000
#define SYS_STATUS_RXFCS 0x4000
#define SYS_STATUS_RXPHE 0x1000
#define SYS_STATUS_RXFCE 0x8000
#define SYS_STATUS_RXRFSL 0x10000
#define SYS_STATUS_RXSFDTO 0x4000000
#define SYS_STATUS_AFFREJ 0x20000000
#define SYS_STATUS_LDEERR 0x40000
#define SYS_STATUS_ALL_RX_ERR  (SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL | SYS_STATUS_RXSFDTO | SYS_STATUS_AFFREJ | SYS_STATUS_LDEERR)
/*
RX_FINFO RX frame information - included in swinging set
*/
#define RX_FINFO 0x10
/*
RX_BUFFER RX frame data buffer
*/
#define RX_BUFFER 0x11
/*
RX_FQUAL RX frame quality information
*/
#define RX_FQUAL 0x12
/*
RX_TTCKI Receiver time tracking interval
*/
#define RX_TTCKI 0x13
/*
RX_TTCKO Receiver time tracking offset
*/
#define RX_TTCKO 0x14
/*
RX_TIME Receive time stamp
*/
#define RX_TIME 0x15
/*
TX_TIME transmit time stamp
*/
#define TX_TIME 0x17
/*
TX_ANTD delay from transmt to antenna
*/
#define TX_ANTD 0x18
/*
SYS_STATE System state information
*/
#define SYS_STATE 0x19
#define SYS_STATE_RX_OFF 8
/*
ACK_RESP_T Acknowledgement time and response time
*/
#define ACK_RESP_T 0x1A
/*
RX_SNIFF Sniff mode configuration
*/
#define RX_SNIFF 0x1D
/*
TX_POWER TX power control
*/
#define TX_POWER 0x1E
/*
CHAN_CTRL channel control register
*/
#define CHAN_CTRL 0x1F
#define CHAN_CTRL_TXC_OFF 0
#define CHAN_CTRL_RXC_OFF 4
#define CHAN_CTRL_DWSFD_OFF 17
#define CHAN_CTRL_RXPRF_OFF 18
#define CHAN_CTRL_TNSSFD_OFF 20
#define CHAN_CTRL_RNSSFD_OFF 21
#define CHAN_CTRL_TXP_OFF 22
#define CHAN_CTRL_RXP_OFF 27
/*
USR_SFD User-specified short/long TX/RX SFD sequeces
*/
#define USR_SFD 0x21
/*
AGC automatic gain control configuration
*/
#define AGC_CTRL 0x23
#define AGC_CTRL_CTRL1 0x02
#define AGC_CTRL_TUNE1 0x04
#define AGC_CTRL_TUNE2 0x0C
#define AGC_CTRL_TUNE3 0x12
/*
EC_CTRL
*/
#define EC_CTRL 0x24
/*
ACC_MEM read access to accumulator data memory
*/
#define ACC_MEM 0x25
/*
GPIO_CTRL peripheral register-bus 1 access - GPIO control
*/
#define GPIO_CTRL 0x26
/*
DRX_CONF Digital receiver configuratio
*/
#define DRX_CONF 0x27
#define DRX_CONF_TUNE0b 0x02
#define DRX_CONF_TUNE1a 0x04
#define DRX_CONF_TUNE1b 0x06
#define DRX_CONF_TUNE2 0x08
#define DRX_CONF_SFDTOC 0x20
#define DRX_CONF_PRETOC 0x24
#define DRX_CONF_TUNE4H 0x26
/*
RF_CONF analog RF configuration
*/
#define RF_CONF 0x28
#define RF_CONF_RXCTRLH 0x0B
#define RF_CONF_TXCTRL 0x0C
#define RF_CONF_LDOTUNE 0x30
/*
TX_CAL transmitter calibration block
*/
#define TX_CAL 0x2A
#define TX_CAL_SARL 0x03
#define TX_CAL_SARW 0x06
#define TX_CAL_PG_CTRL 0x08
#define TX_CAL_PGDELAY 0x0B
/*
FS_CTRL frequency synthesiser control block
*/
#define FS_CTRL 0x2B
#define FS_PLLCFG 0x07
#define FS_PLLTUNE 0x0B
#define FS_XTALT 0x0E
/*
AON always on system control interface block
*/
#define AON 0x2C
#define AON_CTRL 0x02
#define AON_RDAT 0x03
#define AON_ADDR 0x04
#define AON_CFG0 0x06
#define AON_CFG1 0x0A
/*
OTP_IF one time programmable memory interface
*/
#define OTP_IF 0x2D
#define OTP_ADDR 0x04
#define OTP_CTRL 0x06
#define OTP_RDAT 0x0A
#define OTP_SF 0x12
/*
LDE_IF leading edge detection interface
*/
#define LDE_IF 0x2E
#define LDE_CFG1 0x0806
#define LDE_RXANTD 0x1804
#define LDE_CFG2 0x1806
#define LDE_REPC 0x2804
/*
DIG_DIAG digital diagnostics interface
*/
#define DIG_DIAG 0x2F
/*
PMSC power management system control block
*/
#define PMSC 0x36
#define PMSC_CTRL0 0x00
#define PMSC_CTRL1 0x04
#define PMSC_LEDC 0x28

// Defines for enable_clocks function
#define SYS_XTI  0
#define AUTO_CLK 1
#define SYS_PLL  2

#define TIME_UNITS (1.0/499.2e6/128.0)
