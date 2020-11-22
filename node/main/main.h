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

/*
Wi-FI Settings
*/
#define WIFI_SSID "KOUZMINE"
#define WIFI_PASS "kouzmine563!"
#define MAXIMUM_RETRY  4
#define PORT 25565

/*
Pinout
*/
#define PIN_NUM_MISO 19 //PCB: 27 //DEV: 19
#define PIN_NUM_MOSI 18 //14 //18
#define PIN_NUM_CLK 5 //26 //5
#define PIN_NUM_CS 27 //12 //27
#define PIN_NUM_RST 25 //25 //27
