#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#define WIFI_SSID "slowboi" 
#define WIFI_PASS "slowestofbois"    
#define MAXIMUM_RETRY  4
#define PORT 25565

#define PIN_NUM_MISO 27 //PCB: 27 //DEV: 19
#define PIN_NUM_MOSI 14 //14 //18
#define PIN_NUM_CLK 26//26 //5
#define PIN_NUM_CS 12 //12 //21
#define DMA_CHAN 2

#define DWM_TX_BUFFER 0x09
#define DWM_SYS_CFG 0x04
#define DWM_SYS_CTRL 0x0D
#define DWM_SYS_STATUS 0x0F
#define DWM_RX_FINFO 0x10
#define DWM_CHAN_CTRL 0x1F
#define DWM_TX_FCTRL 0x08
#define DWM_TX_FCTRL_BR 13
#define DWM_TX_FCTRL_PRF 16
#define DWM_TX_FCTRL_PSR 18
#define DWM_TX_FCTRL_PE 20
#define DWM_TXBOFFS 22
#define DWM_TXSTRT 0x02
#define DWM_RXENAB 8
#define DWM_TXFRS 7
#define DWM_RXDFR 13
#define DWM_RXPRF 18
#define DWM_PCODE 22
#define DWM_DRX 0x27
#define DWM_DRX_TUNE2 0x08
#define DWM_DRX_TUNE1b 0x06
#define DWM_DRX_TUNE1a 0x04
#define DWM_DRX_TUNE0b 0x02
#define DWM_DRX_PRETOC 0x24
#define DWM_DRX_SFDTOC 0x20
#define DWM_SYS_STATUS_RXPHE 0x1000
#define DWM_SYS_STATUS_RXFCE 0x8000
#define DWM_SYS_STATUS_RXRFSL 0x10000
#define DWM_SYS_STATUS_RXSFDTO 0x4000000
#define DWM_SYS_STATUS_AFFREJ 0x20000000
#define DWM_SYS_STATUS_LDEERR 0x40000
#define DWM_SYS_STATUS_ALL_RX_ERR  (DWM_SYS_STATUS_RXPHE | DWM_SYS_STATUS_RXFCE | DWM_SYS_STATUS_RXRFSL | DWM_SYS_STATUS_RXSFDTO | DWM_SYS_STATUS_AFFREJ | DWM_SYS_STATUS_LDEERR)
#define DWM_AGC 0x23
#define DWM_AGC_TUNE1 0x04
#define DWM_AGC_TUNE2 0x0C
#define DWM_AGC_TUNE1_64PRF 0x889B
#define DWM_AGC_TUNE2_CONST 0x2502A907
#define DWM_LDEIF 0x2E
#define DWM_TXANTD 0x18
#define DWM_LDEIF_RXANTD 0x1804
#define DWM_LDEIF_CFG2 0x1806
#define DWM_LDEIF_REPC 0x2804
#define DWM_ACK_RESP_T 0x1A
#define DWM_RX_FWTO 0x0C
#define DWM_LDE_REPC_CODE9 (0x28F4 >> 3)
#define DWM_FSCTRL 0x2B
#define DWM_FSCTRL_PLLCFG 0x07
#define DWM_FSCTRL_PLLTUNE 0x0B
#define DWM_FSCTRL_PLLCFG_CH5 0x0800041D
#define DWM_FSCTRL_PLLTUNE_CH5 0xBE
#define DWM_RFCONF 0x28
#define DWM_RFCONF_RXCTRL 0x0B
#define DWM_RFCONF_TXCTRL 0x0C
#define DWM_RFCONF_RXCTRL_CH5 0xD8
#define DWM_RFCONF_TXCTRL_CH5 0x1E3FE0
#define DWM_USR_SFD 0x21
#define DWM_PMSC 0x36
#define DWM_PMSC_SOFT 0x03
#define DWM_RXTIME 0x15
#define DWM_TXTIME 0x17
#define DWM_DXTIME 0x0A
#define DWM_TIME_UNITS (1.0/499.2e6/128.0)
#define DWM_TC 0x2A
#define DWM_TC_PGDELAY 0x0B
#define DWM_AON 0x2C

// Defines for enable_clocks function
#define FORCE_SYS_XTI  0
#define ENABLE_ALL_SEQ 1
#define FORCE_SYS_PLL  2
#define READ_ACC_ON    7
#define READ_ACC_OFF   8
#define FORCE_OTP_ON   11
#define FORCE_OTP_OFF  12
#define FORCE_TX_PLL   13
#define FORCE_LDE      14


#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
#define SPEED_OF_LIGHT 299702547
static uint8_t poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8_t final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint64_t poll_rx_ts;
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;
static uint64_t final_rx_ts;
static uint64_t resp_tx_ts;

typedef struct {
    uint8_t cmd[8];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} radio_cmd_t;

spi_device_handle_t spi;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "Node";

void scan_task();

static int s_retry_num = 0;

int sock;

/* DWM1000 Stuff



 */

void dwm_read(uint8_t addr, uint16_t index, int expected, uint8_t * result){
    spi_transaction_t t;          
    memset(&t, 0, sizeof(t));  

    uint8_t cmd[2 + expected];
    cmd[0] = addr | 0x40;
    cmd[1] = index;

    for(int i = 2; i < (2 + expected); i++)
        cmd[i] = 0x00;    

    t.length = 16 + expected * 8;                 
    t.tx_buffer = cmd;
    t.rx_buffer = result;

    spi_device_polling_transmit(spi, &t);
}

void dwm_write(uint8_t addr, uint16_t index, uint8_t * data, uint8_t len){
    uint8_t to_send[2 + len];

    spi_transaction_t t;          
    memset(&t, 0, sizeof(t));

    to_send[0] = addr | 0xC0;
    to_send[1] = index;

    for(int i = 2; i < 2 + len; i++)
        to_send[i] = data[i - 2];
    
    t.length = 16 + len * 8;
    t.tx_buffer = to_send;

    spi_device_polling_transmit(spi, &t);
}

void dwm_write8reg(uint8_t addr, uint16_t index, uint8_t data){
    uint8_t value[1] = {data};

    dwm_write(addr, index, value, 1);
}

void dwm_write16reg(uint8_t addr, uint16_t index, uint16_t data){
    uint8_t value[2] = {data & 0xFF, (data >> 8) & 0xFF};

    dwm_write(addr, index, value, 2);
}

void dwm_write24reg(uint8_t addr, uint16_t index, uint32_t data){
    uint8_t value[3] = {data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF};

    dwm_write(addr, index, value, 3);
}

void dwm_write32reg(uint8_t addr, uint16_t index, uint32_t data){
    uint8_t to_send[4] = {data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF};

    dwm_write(addr, index, to_send, 4);
}

uint8_t dwm_read8reg(uint8_t addr, uint16_t index){
    uint8_t result[3];
    dwm_read(addr, index, 1, result);

    return result[2];
}

uint16_t dwm_read16reg(uint8_t addr, uint16_t index){
    uint8_t result[4];
    dwm_read(addr, index, 2, result);

    return *(uint16_t*)&result[2];
}

uint32_t dwm_read32reg(uint8_t addr, uint16_t index){
    uint8_t result[6];
    dwm_read(addr, index, 4, result);

    return *(uint32_t*)&result[2];
}

void dwm_set_txbuffer(uint8_t * data, uint8_t len){
    dwm_write(DWM_TX_BUFFER, 0, data, len);
}

void send32(uint32_t val){
    char msg[4];
    for(int i = 0; i < 4; i++){
        msg[i] = (val >> (i * 8)) & 0xFF;
    }
    send(sock, msg, 4, 0);
}

void dwm_getID(){
    uint32_t id = dwm_read32reg(0, 0);
    send32(id);
}

void dwm_setrxaftertxdelay(uint32_t delay){
    uint32_t val = dwm_read32reg(DWM_ACK_RESP_T, 0);

    val &= ~(0xFFFFF) ; // Clear 20 bit timer
    val |= (delay & 0xFFFFF);

    dwm_write32reg(DWM_ACK_RESP_T, 0, val);
}

void dwm_setrxtimeout(uint16_t time){
    uint8_t temp = dwm_read8reg(DWM_SYS_CFG, 3);

    if(time > 0){
        dwm_write16reg(DWM_RX_FWTO, 0, time);
        temp |= 0x10;
        dwm_write8reg(DWM_SYS_CFG, 3, temp);
    }else{
        temp &= ~(1 << 4);
        dwm_write8reg(DWM_SYS_CFG, 3, temp);
    }
}

uint64_t dwm_readtime_tx(){
    uint64_t result = 0;
    uint8_t r[7];
    dwm_read(DWM_TXTIME, 0, 5, r);

    for (int i = 6; i >= 2; i--){
        result <<= 8;
        result |= r[i];
    }

    return result;
}

uint64_t dwm_readtime_rx(){
    uint64_t result = 0;
    uint8_t r[7];
    dwm_read(DWM_RXTIME, 0, 5, r);

    for (int i = 6; i >= 2; i--){
        result <<= 8;
        result |= r[i];
    }

    return result;
}

void write_ts(uint8_t * arr, uint64_t ts){
    for(int i = 0; i < 4; i++){
        arr[i] = (uint8_t) ts;
        ts >>= 8;
    }
}

void get_ts(uint8_t *arr, uint32_t *ts){
    *ts = 0;
    for (int i = 0; i < 4; i++){
        *ts += arr[i] << (i * 8);
    }
}

void dwm_enableclocks(int clocks){
    uint8_t r[4];
    uint8_t reg[2];

    dwm_read(DWM_PMSC, 0, 2, r);
    reg[0] = r[2];
    reg[1] = r[3];
    switch(clocks){
        case ENABLE_ALL_SEQ:{
            reg[0] = 0x00 ;
            reg[1] = reg[1] & 0xfe;
        }
        break;
        case FORCE_SYS_XTI:{
            // System and RX
            reg[0] = 0x01 | (reg[0] & 0xfc);
        }
        break;
        case FORCE_SYS_PLL:{
            // System
            reg[0] = 0x02 | (reg[0] & 0xfc);
        }
        break;
        case READ_ACC_ON:{
            reg[0] = 0x48 | (reg[0] & 0xb3);
            reg[1] = 0x80 | reg[1];
        }
        break;
        case READ_ACC_OFF:{
            reg[0] = reg[0] & 0xb3;
            reg[1] = 0x7f & reg[1];
        }
        break;
        case FORCE_OTP_ON:{
            reg[1] = 0x02 | reg[1];
        }
        break;
        case FORCE_OTP_OFF:{
            reg[1] = reg[1] & 0xfd;
        }
        break;
        case FORCE_TX_PLL:{
            reg[0] = 0x20 | (reg[0] & 0xcf);
        }
        break;
        case FORCE_LDE:{
            reg[0] = 0x01;
            reg[1] = 0x03;
        }
        break;
        default:
        break;
    }


    // Need to write lower byte separately before setting the higher byte(s)
    dwm_write8reg(DWM_PMSC, 0, reg[0]);
    dwm_write8reg(DWM_PMSC, 0x1, reg[1]);

}

uint32_t dwm_otpread(uint16_t address){
    uint32_t result;

    // Write the address
    dwm_write16reg(0x2D, 0x04, address);

    // Perform OTP Read - Manual read mode has to be set
    dwm_write8reg(0x2D, 0x06, 0x03);
    dwm_write8reg(0x2D, 0x06, 0x00); // OTPREAD is self clearing but OTPRDEN is not

    // Read read data, available 40ns after rising edge of OTP_READ
    result = dwm_read32reg(0x2D, 0x0A);

    // Return the 32bit of read data
    return result;
}

void dwm_rxreset(){
    // Set RX reset
    dwm_write8reg(DWM_PMSC, DWM_PMSC_SOFT, 0xE0);

    // Clear RX reset
    dwm_write8reg(DWM_PMSC, DWM_PMSC_SOFT, 0xF0);
}

void dwm_reset(){
	// Enable GPIO used for DW1000 reset
    gpio_pad_select_gpio(25);
    gpio_set_direction(25, GPIO_MODE_OUTPUT);
    gpio_set_level(25, 0);

	//put the pin back to tri-state ... as input
	gpio_set_direction(25, GPIO_MODE_INPUT);

    usleep(20000);
}

void dwm_softreset(){
    //Disable sequencing
    dwm_enableclocks(FORCE_SYS_XTI); // Set system clock to XTI
    dwm_write16reg(DWM_PMSC, 0x04, 0); // Disable PMSC ctrl of RF and RX clk blocks

    // Clear any AON auto download bits (as reset will trigger AON download)
    dwm_write16reg(DWM_AON, 0, 0x00);
    // Clear the wake-up configuration
    dwm_write16reg(DWM_AON, 0x06, 0x00);
    // Upload the new configuration
    dwm_write8reg(DWM_AON, 0x02, 0x00); // Clear the register
    dwm_write8reg(DWM_AON, 0x02, 0x02);

    // Reset HIF, TX, RX and PMSC
    dwm_write8reg(DWM_PMSC, 0x03, 0);

    // DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
    // Could also have polled the PLL lock flag, but then the SPI needs to be < 3MHz !! So a simple delay is easier
    usleep(200);

    // Clear reset
    dwm_write8reg(DWM_PMSC, 0x03, 0xF0);

    // Set system clock to XTI
    dwm_enableclocks(FORCE_SYS_XTI); 
}

void dwm_initialize(){
	dwm_softreset();
}

void sayHello(){
    //Set transmit buffer
    uint8_t hello[9] = {0xC5, 0, 'h', 'e', 'l', 'l', 'o', 0, 0};
    dwm_set_txbuffer(hello, 7);

    //Read transmit config
    uint32_t config = dwm_read32reg(DWM_TX_FCTRL, 0);
    ESP_LOGI(TAG, "Read config %X", config);

    //Change config to have frame of message and no offset while keeping the rest as is
    config = (~0xFF & config) | 9; //Size should be +2 for CRC - check docs
    config |= config & ~(0xFFC00000); //Clear offset

    ESP_LOGI(TAG, "New config %X", config);
    dwm_write32reg(DWM_TX_FCTRL, 0, config);

    config = dwm_read32reg(DWM_TX_FCTRL, 0);

    ESP_LOGI(TAG, "Confirming new config %X", config);

    send32(config);

    //Transmit!
    dwm_write8reg(DWM_SYS_CTRL, 0, DWM_TXSTRT);
    ESP_LOGI(TAG, "Transmitting and waiting for confirmation");

    while(!(dwm_read32reg(DWM_SYS_STATUS, 0) & (1 << DWM_TXFRS))){}

    ESP_LOGI(TAG, "Sent!");

    uint32_t clear = dwm_read32reg(DWM_SYS_STATUS, 0);
    clear &= ~(1 << DWM_TXFRS);
    dwm_write32reg(DWM_SYS_STATUS, 0, clear);
}

void range_respond(){
    uint32_t status_reg = 0;

    dwm_setrxtimeout(0);

    dwm_write32reg(DWM_SYS_CTRL, 0, 0x100); //Start RX

    while (!((status_reg = dwm_read32reg(DWM_SYS_STATUS, 0)) & (1 << 14 | 1 << 17 | 1 << 21 | DWM_SYS_STATUS_ALL_RX_ERR))){}; //FCS, FTO, PTO

    if(status_reg & (1 << 14)){ //Good rx
        dwm_write32reg(DWM_SYS_STATUS, 0, 1 << 14); //Clear rx event

        uint8_t rx_len = (uint8_t)(dwm_read32reg(DWM_RX_FINFO, 0) & 0x7F);

        uint32_t resp_tx_time;

        poll_rx_ts = dwm_readtime_rx();

        resp_tx_time = (poll_rx_ts + (2600 * 65536)) >> 8;
        dwm_write32reg(DWM_DXTIME, 1, resp_tx_time);

        dwm_setrxaftertxdelay(500);
        dwm_setrxtimeout(3300);

        dwm_set_txbuffer(resp_msg, sizeof(resp_msg) - 2);

        uint32_t config = dwm_read32reg(DWM_TX_FCTRL, 0);
        config = (~0xFF & config) | sizeof(resp_msg); //Size should be +2 for CRC - check docs
        config |= config & ~(0xFFC00000); //Clear offset
        config |= (1 << 15); //Enable ranging
        dwm_write32reg(DWM_TX_FCTRL, 0, config);

        dwm_write8reg(DWM_SYS_CTRL, 0, 0x86); //Delayed start, response expected

        while (!((status_reg = dwm_read32reg(DWM_SYS_STATUS, 0)) & (1 << 14 | 1 << 17 | 1 << 21 | DWM_SYS_STATUS_ALL_RX_ERR))){}; //FCS, FTO, PTO

        if(status_reg & (1 << 14)){ //Good rx
            dwm_write32reg(DWM_SYS_STATUS, 0, 1 << 14 | 1 << DWM_TXFRS); //Clear rx event

            uint8_t rx_len = (uint8_t)(dwm_read32reg(DWM_RX_FINFO, 0) & 0x7F);

            uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
            uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
            double Ra, Rb, Da, Db;
            int64_t tof_dtu;

            /* Retrieve response transmission and final reception timestamps. */
            resp_tx_ts = dwm_readtime_tx();
            final_rx_ts = dwm_readtime_rx();

            uint8_t data[rx_len];
            dwm_read(0x11, 0, rx_len, data);

            /* Get timestamps embedded in the final message. */
            get_ts(&data[10], &poll_tx_ts);
            get_ts(&data[14], &resp_rx_ts);
            get_ts(&data[18], &final_tx_ts);

            /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
            poll_rx_ts_32 = (uint32_t)poll_rx_ts;
            resp_tx_ts_32 = (uint32_t)resp_tx_ts;
            final_rx_ts_32 = (uint32_t)final_rx_ts;
            Ra = (double)(resp_rx_ts - poll_tx_ts);
            Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
            Da = (double)(final_tx_ts - resp_rx_ts);
            Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
            tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

            double tof = tof_dtu * DWM_TIME_UNITS;
            double distance = tof * SPEED_OF_LIGHT;

            /* Display computed distance on LCD. */
            ESP_LOGI(TAG, "Distance is %3.2f m", distance);
        }else{
            ESP_LOGI(TAG, "Something went wrong in reading, STATUS %X", dwm_read32reg(DWM_SYS_STATUS, 0));
            //Clear errors
            dwm_write32reg(DWM_SYS_STATUS, 0, 1 << 17 | 1 << 21 | DWM_SYS_STATUS_ALL_RX_ERR);
            
            //Soft reset
            dwm_rxreset();
        }
    }else{
        ESP_LOGI(TAG, "Something went wrong in reading, STATUS %X", dwm_read32reg(DWM_SYS_STATUS, 0));
        //Clear errors
        dwm_write32reg(DWM_SYS_STATUS, 0, 1 << 17 | 1 << 21 | DWM_SYS_STATUS_ALL_RX_ERR);
        
        //Soft reset
        dwm_rxreset();
    }
}

void range_init(){
    //Set response delay
    dwm_setrxaftertxdelay(150);

    //Set response timeout
    dwm_setrxtimeout(2700);

    dwm_set_txbuffer(poll_msg, sizeof(poll_msg) - 2);

    uint32_t config = dwm_read32reg(DWM_TX_FCTRL, 0);
    config = (~0xFF & config) | sizeof(poll_msg); //Size should be +2 for CRC - check docs
    config |= config & ~(0xFFC00000); //Clear offset
    config |= (1 << 15); //Enable ranging
    dwm_write32reg(DWM_TX_FCTRL, 0, config);

    dwm_write8reg(DWM_SYS_CTRL, 0, 0x82); //Wait for response and start transmission

    uint32_t status_reg = 0;

    while(!((status_reg = dwm_read32reg(DWM_SYS_STATUS, 0)) & (1 << 14 | 1 << 17 | 1 << 21 | DWM_SYS_STATUS_ALL_RX_ERR))){}; //FCS, FTO, PTO

    if(status_reg & (1 << 14)){ //Good rx
        //Clear good RX frame event and TX frame sent in the DW1000 status register
        dwm_write32reg(DWM_SYS_STATUS, 0, 1 << 14 | 1 << DWM_TXFRS);

        uint8_t rx_len = (uint8_t)(dwm_read32reg(DWM_RX_FINFO, 0) & 0x7F);

        uint32_t final_tx_time;
        
        poll_tx_ts = dwm_readtime_tx();
        resp_rx_ts = dwm_readtime_rx();

        final_tx_time = (resp_rx_ts + (3100 * 65536)) >> 8;
        dwm_write32reg(DWM_DXTIME, 1, final_tx_time);

        final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        //Write timestamps into message
        write_ts(&final_msg[10], poll_tx_ts);
        write_ts(&final_msg[14], resp_rx_ts);
        write_ts(&final_msg[18], final_tx_ts);

        dwm_set_txbuffer(final_msg, sizeof(final_msg));

        config = dwm_read32reg(DWM_TX_FCTRL, 0);
        config = (~0xFF & config) | sizeof(final_msg); //Size should be +2 for CRC - check docs
        config |= config & ~(0xFFC00000); //Clear offset
        config |= (1 << 15); //Enable ranging
        dwm_write32reg(DWM_TX_FCTRL, 0, config);

        dwm_write8reg(DWM_SYS_CTRL, 0, 0x06); //Delayed and start

        while (!(dwm_read32reg(DWM_SYS_STATUS, 0) & (1 << DWM_TXFRS))){ };
        
        dwm_write32reg(DWM_SYS_STATUS, 0, 1 << DWM_TXFRS);

    }else{
        ESP_LOGI(TAG, "Something went wrong in reading, STATUS %X", dwm_read32reg(DWM_SYS_STATUS, 0));
        //Clear errors
        dwm_write32reg(DWM_SYS_STATUS, 0, 1 << 17 | 1 << 21 | DWM_SYS_STATUS_ALL_RX_ERR);
        
        //Soft reset
        dwm_rxreset();
    }

}

void dwm_rx(){
    dwm_write32reg(DWM_SYS_CTRL, 0, 0x100);

    //while(!(dwm_read32reg(DWM_SYS_STATUS, 0) & (DWM_SYS_STATUS_ALL_RX_ERR | 1 << 13))){}

    usleep(10000);

    ESP_LOGI(TAG, "System status is %X", dwm_read32reg(DWM_SYS_STATUS, 0));

    uint8_t rx_len = (uint8_t)(dwm_read32reg(DWM_RX_FINFO, 0) & 0x7F);

    ESP_LOGI(TAG, "Received %d bytes of data", rx_len);

    uint8_t data[rx_len];
    dwm_read(0x11, 0, rx_len, data);

    for(int i = 0; i < rx_len; i++){
        ESP_LOGI(TAG, "Data received: %d %c", data[i], data[i]);
    }
}

void dwm_configure(){
    //Set antenna delays
    dwm_write16reg(DWM_LDEIF, DWM_LDEIF_RXANTD, RX_ANT_DLY);
    dwm_write16reg(DWM_TXANTD, 0, TX_ANT_DLY);

    //Set preamble detect timeout
    dwm_write16reg(DWM_DRX, DWM_DRX_PRETOC, 0); //0 - disabled 8 - recommendedd

    //Set SFD timeout
    dwm_write16reg(DWM_DRX, DWM_DRX_SFDTOC, (1025 + 64 - 32)); //Preamble length + 1 + SFD - PAC

    //Set LDE replica coefficient
    dwm_write16reg(DWM_LDEIF, DWM_LDEIF_REPC, DWM_LDE_REPC_CODE9);

    //Set channel 5 PLL CFG and Tune
    dwm_write32reg(DWM_FSCTRL, DWM_FSCTRL_PLLCFG, DWM_FSCTRL_PLLCFG_CH5);
    dwm_write8reg(DWM_FSCTRL, DWM_FSCTRL_PLLTUNE, DWM_FSCTRL_PLLTUNE_CH5);

    //Configure TX and RX RF blocks
    dwm_write8reg(DWM_RFCONF, DWM_RFCONF_RXCTRL, DWM_RFCONF_RXCTRL_CH5);
    dwm_write24reg(DWM_RFCONF, DWM_RFCONF_TXCTRL, DWM_RFCONF_TXCTRL_CH5);

    //Set SFD length
    dwm_write8reg(DWM_USR_SFD, 0, 64);

    //LDE tune
    dwm_write16reg(DWM_LDEIF, DWM_LDEIF_CFG2, 0x0607);
    
    //TC channel 5
    dwm_write8reg(DWM_TC, DWM_TC_PGDELAY, 0xC0);

    //LDELOAD
    dwm_write16reg(DWM_PMSC, 0, 0x0301);
    dwm_write16reg(DWM_PMSC, 0x06, 0x8000);
    usleep(200);
    dwm_write16reg(DWM_PMSC, 0, 0x0200);

    //Check if tuning microcode is loaded
    uint32_t ldotune = dwm_otpread(0x04);
    if((ldotune & 0xFF) != 0){
        ESP_LOGI(TAG, "Tune found");
        dwm_write8reg(0x2D, 0x12, 0x02);
    }else{
        ESP_LOGI(TAG, "No tune found");
    }

    uint32_t sys_cfg = dwm_read32reg(DWM_SYS_CFG, 0);
    ESP_LOGI(TAG, "SYS_CFG configured to %X", sys_cfg);
    sys_cfg |= (1 << 22);
    sys_cfg &= ~(0x03 << 16);
    dwm_write32reg(DWM_SYS_CFG, 0, sys_cfg);
    ESP_LOGI(TAG, "New SYS_CFG configured to %X", dwm_read32reg(DWM_SYS_CFG, 0));

    //Set channel PRF to 64MHz and preamble code to 9
    uint32_t chan_ctrl = dwm_read32reg(DWM_CHAN_CTRL, 0);
    chan_ctrl &= ~(0x03 << DWM_RXPRF); //Clear PRF settings
    chan_ctrl &= ~(0xFFE00000); //Clear preamble codes
    chan_ctrl |= (0x02 << DWM_RXPRF); //Set PRF to 64 MHz
    chan_ctrl |= (0x09 << DWM_PCODE | 0x09 << (DWM_PCODE + 5));  //Preamble code = 9 on TX and RX
    chan_ctrl |= (1 << 17); //Enable non-standard SFD
    chan_ctrl |= (0x03 << 20); //Enable non-standard SFD on TX and RX
    dwm_write32reg(DWM_CHAN_CTRL, 0, chan_ctrl);

    ESP_LOGI(TAG, "Channel CTRL configured to %X", dwm_read32reg(DWM_CHAN_CTRL, 0));

    //Tune AGC
    dwm_write16reg(DWM_AGC, DWM_AGC_TUNE1, DWM_AGC_TUNE1_64PRF);
    dwm_write32reg(DWM_AGC, DWM_AGC_TUNE2, DWM_AGC_TUNE2_CONST);

    //Set SFD config for RX tune 0b
    dwm_write16reg(DWM_DRX, DWM_DRX_TUNE0b, 0x0016);

    //Set PAC size
    dwm_write32reg(DWM_DRX, DWM_DRX_TUNE2, 0x353B015E);

    //Set BR to 110kbps
    dwm_write16reg(DWM_DRX, DWM_DRX_TUNE1b, 0x0064);

    //Set tune to 64 MHz PRF
    dwm_write16reg(DWM_DRX, DWM_DRX_TUNE1a, 0x008D);

    //Configure TX frame control
    uint32_t fctrl = dwm_read32reg(DWM_TX_FCTRL, 0);
    fctrl &= ~(0x03 << DWM_TX_FCTRL_BR | 0x03 << DWM_TX_FCTRL_PRF | 0x03 << DWM_TX_FCTRL_PSR | 0x03 << DWM_TX_FCTRL_PE);
    fctrl |= (0x02 << DWM_TX_FCTRL_PRF | 0x02 << DWM_TX_FCTRL_PSR); //PRF to 64 MHz and PSR to 1024 (bits to 10 each)
    dwm_write32reg(DWM_TX_FCTRL, 0, fctrl);

    ESP_LOGI(TAG, "New FCTRL config is %X", dwm_read32reg(DWM_TX_FCTRL, 0));

    //Apparently fixes some bug with SFD
    dwm_write8reg(DWM_SYS_CTRL, 0, 0x42); // Request TX start and TRX off at the same time

}

void radio_get_temperature(){
    dwm_write8reg(0x28, 0x11, 0x80);
    dwm_write8reg(0x28, 0x12, 0x0A);
    dwm_write8reg(0x28, 0x12, 0x0F);
    dwm_write8reg(0x2A, 0x00, 0x00);
    dwm_write8reg(0x2A, 0x00, 0x01);

    usleep(100);

    uint8_t temp = dwm_read8reg(0x2A, 0x03);
    uint8_t volt = dwm_read8reg(0x2A, 0x04);

    dwm_write8reg(0x2A, 0x00, 0x00);

    char msg[2] = {volt, temp};
    send(sock, msg, 2, 0);
}

/*

Socket handler

*/

static void do_retransmit()
{
    int len;
    char rx_buffer[128];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

            if(!strcmp(rx_buffer, "on")){
                gpio_set_level(13, 1);
            }

            if(!strcmp(rx_buffer, "off")){
                gpio_set_level(13, 0);
            }

            if(!strcmp(rx_buffer, "spin")){ //spin a lil'
                for(int i = 0; i < 800; i++){
                    gpio_set_level(14, 1);
                    usleep(1000);
                    gpio_set_level(14, 0);
                    usleep(1000);
                }
                
            }

            if(!strcmp(rx_buffer, "hello")){
                sayHello();
            }

            if(!strcmp(rx_buffer, "temp")){
                radio_get_temperature();
            }

            if(!strcmp(rx_buffer, "id")){
                dwm_getID();
            }

            if(!strcmp(rx_buffer, "rx")){
                dwm_rx();
            }

            if(!strcmp(rx_buffer, "init")){
                range_init();
            }

            if(!strcmp(rx_buffer, "range")){
                range_respond();
            }

            if(!strcmp(rx_buffer, "measure")){
                //xTaskCreate(scan_task, "scan_task", 4024, NULL, 5, NULL);
                scan_task();
            }

            if(!strcmp(rx_buffer, "collect")){
                send(sock, "ok", 2, 0);

                scan_task();
            }
            

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation. 
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
        }
    } while (len > 0);
}

static void tcp_server_task(void *pvParameters){
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in source_addr; 
        uint addr_len = sizeof(source_addr);
        sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Convert ip address to string
        if (source_addr.sin_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();

    esp_event_loop_create_default();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);

    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);

    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler);
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler);
    vEventGroupDelete(s_wifi_event_group);
}

void radio_spi_pre_transfer_callback(spi_transaction_t *t)
{
    //int dc=(int)t->user;
    //gpio_set_level(PIN_NUM_DC, dc);
    //ESP_LOGI(TAG, "Transfer on SPI");
}

void step_one(){
    gpio_set_level(14, 1);
    usleep(1000);
    gpio_set_level(14, 0);
    usleep(1000);
}

uint16_t getMeasurement(){
    uint8_t data[8];
    while (1){
        uart_read_bytes(UART_NUM_2, data, 8, 50);
        //ESP_LOGI(TAG, "%02x", (char)byte[0]);
        if(data[0] == 0x59 && data[1] == 0x59){
            break;
        }
    }

    uint16_t out =  data[2] + (data[3] << 8);

    return out;
}

//void *pvParameters
void scan_task(){
    char tx_buffer[2];
    for(int i = 0; i < 800; i++){
        uint16_t dist = getMeasurement();
        ESP_LOGI(TAG, "Measurement: %d", dist);

        tx_buffer[0] = dist & 0xFF;
        tx_buffer[1] = dist >> 8;

        send(sock, tx_buffer, 2, 0);

        step_one();
    }
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    gpio_pad_select_gpio(13);
    gpio_set_direction(13, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(14);
    gpio_set_direction(14, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(15);
    gpio_set_direction(15, GPIO_MODE_OUTPUT);
    gpio_set_level(15, 0);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, 16, 17, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    QueueHandle_t uart_queue;
    uart_driver_install(UART_NUM_2, 1024, 1024, 10, &uart_queue, 0);

    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4094
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz=10*1000,           //Clock out at 0.01 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num = PIN_NUM_CS,              //CS pin
        .queue_size = 7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=radio_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };


    ESP_ERROR_CHECK(spi_bus_initialize(1, &buscfg, DMA_CHAN));

    ESP_ERROR_CHECK(spi_bus_add_device(1, &devcfg, &spi));

    dwm_reset();

    dwm_configure();

    xTaskCreate(tcp_server_task, "tcp_server", 4024, NULL, 5, NULL);
}
