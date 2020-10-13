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
#include "driver/uart.h"
#include <unistd.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#define WIFI_SSID "slowboi"
#define WIFI_PASS "slowestofbois"    
#define MAXIMUM_RETRY  4
#define PORT 25565
#define UART_NUM UART_NUM_2
//2 sets of 9 bits in queue
#define uart_buffer_size (128 * 9 * 2)
static QueueHandle_t uart_queue;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "Node";
static int s_retry_num = 0;


static void config_single_shot() {
    //enter param config mode
    uint8_t config_arr[8] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x02 };
    int num_bytes_sent = uart_write_bytes(UART_NUM, config_arr, sizeof(uint8_t) * 8);
    ESP_LOGI(TAG, "Wrote %d bytes\n", num_bytes_sent);
    printf("test0\n");
    //this line below should wait until the shit is pushed to the fifo queue, and it returns fine
    //however the scope says nothing is actually output 
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM, 10)); //wait till tx is empty or 100 rtos ticks
    printf("test1\n");
    int num_bytes_received = 0;

    uart_event_t event;
    if(xQueueReceive(uart_queue, (void * )&event, 100)) {
        ESP_LOGI(TAG, "uart event type: %d", event.type);
    } else {
        printf("No event\n");
    }

    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, (size_t*)&num_bytes_received));
    printf("recieved %d bytes\n", num_bytes_received);
    while(num_bytes_received < 8) {
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //this doesn't read anything so rtos times out on it
        uart_get_buffered_data_len(UART_NUM, (size_t*)&num_bytes_received);
        //printf("recieved %d bytes", num_bytes_received);
    }
    uint8_t compare_arr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    if(uart_read_bytes(UART_NUM, compare_arr, num_bytes_received, 100) != num_bytes_received) {
        printf("Something went horribly wrong!\n");
        return;
    } 
    uart_flush(UART_NUM);
    printf("01 is success, FF is fail: %X\n", compare_arr[3]);
    //set to use external trigger
    memcpy(config_arr, (uint8_t[]){0x42,0x57,0x02,0x00,0x00,0x00,0x00,0x41} ,8);
    num_bytes_sent = uart_write_bytes(UART_NUM, config_arr, sizeof(uint8_t) * 8);
    ESP_LOGI(TAG, "Wrote %d bytes\n", num_bytes_sent);
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM, 100)); //wait till tx is empty or 100 rtos ticks
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, (size_t*)&num_bytes_received));
    while(num_bytes_received < 8) {
        uart_get_buffered_data_len(UART_NUM, (size_t*)&num_bytes_received);
    }
    if(uart_read_bytes(UART_NUM, compare_arr, num_bytes_received, 100) != num_bytes_received) {
        printf("Something went horribly wrong again!\n");
        return;
    } 
    uart_flush(UART_NUM);
    printf("01 is success, FF is fail: %X\n", compare_arr[3]);
    printf("I'd be shocked if I make it this far\n");
}

static void do_retransmit(const int sock)
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
            if(!strcmp(rx_buffer, "turn")) {
                for(int fuck = 0; fuck < 500; fuck++) {
                    gpio_set_level(14, 1);
                    //sleep(1);
                    usleep(100);
                    gpio_set_level(14, 0);
                    //sleep(1);
                    printf("turning, %d\n", fuck);
                    usleep(100);
                }
            }
            //if(!strcmp(rx_buffer, "UART")){
            //   config_single_shot();
            //}
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
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
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

    //internal LED
    gpio_pad_select_gpio(13);
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    //step
    gpio_pad_select_gpio(14);
    gpio_set_direction(14, GPIO_MODE_OUTPUT);

    //dir
    gpio_pad_select_gpio(15);
    gpio_set_direction(15, GPIO_MODE_OUTPUT);
    gpio_set_level(15, 0);

    //UART params
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    //Install driver for UART
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 18, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    // Set UART pins(TX: IO16 (UART2 default), RX: IO17 (UART2 default), RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_pattern_queue_reset(UART_NUM, 18);

    config_single_shot();

    xTaskCreate(tcp_server_task, "tcp_server", 8192, NULL, 12, NULL); //4096, 5


}
