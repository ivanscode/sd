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
#include <unistd.h>
#include "driver/spi_master.h"
#include "driver/uart.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

//#define WIFI_SSID "slowboi" 
//#define WIFI_PASS "slowestofbois"    
#define WIFI_SSID "buttstuff?" 
#define WIFI_PASS "seabass!"    
#define MAXIMUM_RETRY  4
#define PORT 25565
#define UART_NUM UART_NUM_2
//2 sets of 9 bits in queue
#define uart_buffer_size (128 * 9 * 2)
static QueueHandle_t uart_queue;

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 18
#define PIN_NUM_CLK 5
#define PIN_NUM_CS 21
#define DMA_CHAN 2

typedef struct {
    uint8_t cmd[8];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} radio_cmd_t;

DRAM_ATTR static const radio_cmd_t cmds[]={
    {{0xE8, 0x11, 0x80}, 3},
    {{0xE8, 0x12, 0x0A}, 3},
    {{0xE8, 0x12, 0x0F}, 3},
    {{0xAA, 0x01}, 2},
    {{0xAA, 0x00}, 2},
    {{0x6A, 0x04}, 2} //Read temperature
};

spi_device_handle_t spi;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static QueueHandle_t uart_queue;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "Node";

void scan_task();
void step_one();

static int s_retry_num = 0;

int sock;


static void config_single_shot() {
    //enter param config mode
    uart_flush(UART_NUM_2);
    uint8_t config_arr[8] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x02 };
    uart_write_bytes(UART_NUM_2, config_arr, sizeof(uint8_t) * 8);
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, 10)); //wait till tx is empty or 100 rtos ticks

    /* uart_event_t event;
    if(xQueueReceive(uart_queue, (void * )&event, 100)) {
        ESP_LOGI(TAG, "uart event type: %d", event.type);       #only event we will ever recieve in single shot mode is data, this becomes unnecessary 
        printf("%d\n", UART_DATA);
    } else {
        printf("No event\n");
    } */

    int num_bytes_received = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&num_bytes_received));
    //printf("recieved %d bytes\n", num_bytes_received);
    while(num_bytes_received < 8) {
        uart_get_buffered_data_len(UART_NUM_2, (size_t*)&num_bytes_received);
        //printf("recieved %d bytes", num_bytes_received);
    }
    uint8_t compare_arr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uart_read_bytes(UART_NUM_2, compare_arr, 8, 100);
    printf("01 is success, FF is fail: %X\n", compare_arr[3]);
    printf("%X, %X, %X, %X, %X, %X, %X, %X\n", compare_arr[0], compare_arr[1], compare_arr[2], compare_arr[3], compare_arr[4], compare_arr[5], compare_arr[6], compare_arr[7]);

    //set trigger to external 
    memcpy(config_arr, (uint8_t[]){0x42,0x57,0x02,0x00,0x00,0x00,0x00,0x40} ,8);
    uart_write_bytes(UART_NUM_2, config_arr, sizeof(uint8_t) * 8);
    //ESP_LOGI(TAG, "Wrote %d bytes\n", num_bytes_sent);
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, 100)); //wait till tx is empty or 100 rtos ticks

    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&num_bytes_received));
    //printf("recieved %d bytes\n", num_bytes_received);
    while(num_bytes_received < 8) {
        uart_get_buffered_data_len(UART_NUM_2, (size_t*)&num_bytes_received);
    }
    uart_read_bytes(UART_NUM_2, compare_arr, 8, 100);
    printf("01 is success, FF is fail: %X\n", compare_arr[3]);
    printf("%X, %X, %X, %X, %X, %X, %X, %X\n", compare_arr[0], compare_arr[1], compare_arr[2], compare_arr[3], compare_arr[4], compare_arr[5], compare_arr[6], compare_arr[7]);
    printf("UART configuration complete\n");
    return;
}


uint16_t pull_trig() {
    //send config commmand
    uint8_t config_arr[8] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x02 };
    uart_write_bytes(UART_NUM_2, config_arr, sizeof(uint8_t) * 8);
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, 10)); //wait till tx is empty or 100 rtos ticks
    int num_bytes_received = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&num_bytes_received));
    while(num_bytes_received < 8) {
        uart_get_buffered_data_len(UART_NUM_2, (size_t*)&num_bytes_received);
    }
    uint8_t compare_arr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uart_read_bytes(UART_NUM_2, compare_arr, 8, 100);
    printf("01 is success, FF is fail: %X\n", compare_arr[3]);
    //pull trig
    memcpy(config_arr, (uint8_t[]){0x42,0x57,0x02,0x00,0x00,0x00,0x00,0x41} ,8);
    uart_write_bytes(UART_NUM_2, config_arr, sizeof(uint8_t) * 8);
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_2, 100)); //wait till tx is empty or 100 rtos ticks
    while(num_bytes_received < 9) {
        uart_get_buffered_data_len(UART_NUM_2, (size_t*)&num_bytes_received);
    }
    uint8_t result_arr[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uart_read_bytes(UART_NUM_2, result_arr, 9, 100);
    printf("%X, %X, %X, %X, %X, %X, %X, %X, %X\n", result_arr[0], result_arr[1], result_arr[2], result_arr[3], result_arr[4], result_arr[5], result_arr[6], result_arr[7], result_arr[8]);
    uint16_t ret = compare_arr[2] | (compare_arr[3] << 8);
    return ret;
}


void go_brrrrr() {
    char tx_buffer[1600];
    for(int i = 0; i < 800; i++){
        uint16_t dist = pull_trig();
        ESP_LOGI(TAG, "Measurement: %02x", dist);

        tx_buffer[i * 2] = (uint8_t) dist;
        tx_buffer[i * 2 + 1] = (uint8_t)(dist >> 8);

        step_one();
    }

    send(sock, tx_buffer, 1600, 0);
}


void reset_dwm(){
    gpio_pad_select_gpio(32);
    gpio_set_direction(32, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(32, 0);

    usleep(1);

    gpio_pulldown_dis(32);
    gpio_set_level(32, 1);

    sleep(2);
}


void write_off(uint8_t addr, uint8_t off, uint8_t val){
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    uint8_t cmd[3] = {(addr | 0xC0), off, val};
    t.length=24;
    t.tx_buffer=cmd;

    spi_device_polling_transmit(spi, &t);
}


void initialize_dwm(){
    write_off(0x24, 0x00, 0x04);
}


void radio_tx(spi_device_handle_t spi, const uint8_t *cmd, int len){
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=cmd;               //Data
    t.flags = SPI_TRANS_USE_RXDATA;
    spi_device_polling_transmit(spi, &t);  //Transmit!

    if(cmd[0] == 0x6A){
        int temp = *(uint32_t*)t.rx_data;
        ESP_LOGI(TAG, "UWB Temperature is %d", temp);
    }
}


void radio_get_temperature(spi_device_handle_t spi){
    for(int i = 0; i < 6; i++){
        radio_tx(spi, cmds[i].cmd, cmds[i].databytes&0x1F);
    }
}


static void do_retransmit() {
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
            if(!strcmp(rx_buffer, "read")){
                // Read data from UART.
                uint8_t data[128];
                int length = 0;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, (size_t*)&length));
                length = uart_read_bytes(UART_NUM, data, length, 100);
                printf("data: %d\n", data[0]);
                printf("length: %d\n", length);
                uart_flush(UART_NUM);
            }

            if(!strcmp(rx_buffer, "spin")){ //spin a lil'
                for(int i = 0; i < 800; i++){
                    gpio_set_level(14, 1);
                    usleep(1000);
                    gpio_set_level(14, 0);
                    usleep(1000);
                }
            }
            if(!strcmp(rx_buffer, "temp")){
                radio_get_temperature(spi);
            }
            if(!strcmp(rx_buffer, "measure")){
                go_brrrrr();
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


void radio_spi_pre_transfer_callback(spi_transaction_t *t) {
    //int dc=(int)t->user;
    //gpio_set_level(PIN_NUM_DC, dc);
    ESP_LOGI(TAG, "Transfer on SPI");
}

void step_one() {
    gpio_set_level(14, 1);
    usleep(20);
    gpio_set_level(14, 0);
    usleep(20);
}


void app_main(void) {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    //internal LED
    gpio_pad_select_gpio(13);
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    //step
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
    
    uart_driver_install(UART_NUM_2, 1024, 1024, 10, &uart_queue, 0);

    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz=10*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num = PIN_NUM_CS,               //CS pin
        .queue_size = 7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=radio_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    spi_bus_initialize(1, &buscfg, DMA_CHAN);

    spi_bus_add_device(1, &devcfg, &spi);

    reset_dwm();

    initialize_dwm();

    config_single_shot();

    xTaskCreate(tcp_server_task, "tcp_server", 4024, NULL, 5, NULL);}
