#include "stdint.h"
#include "driver/gpio.h"
#include "stdio.h"
#include "nvs_flash.h"
#include "ctype.h"
#include "inttypes.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#define MAC_KEY_NAME "nvsMacKey"
#define ESP_NOW_CHANNEL_KEY "nvsEnowChKey"
#define USER_BTN_KEY "nvsUsrBtnKey"
#define WIFI_CHANNEL_KEY "nvsWifiChKey"
#define SENSOR_1_KEY "nvsSensor1Key"
#define SENSOR_2_KEY "nvsSensor2Key"
#define LED_KEY "nvsLedKey"
size_t millis(void);
typedef struct alarmTimeStruct
{
    size_t pulseCount;
    size_t windowStartTime;
    size_t windowCurrentTime;
    size_t lastAlarmTime;
    bool isAlarmTriggered;
}alarmTimeStruct;
bool nvsCheckCredentialsPresent(void);

void getMacAddr(void);
void getGpios(void);
void getWifiChannel(void);
void espConsoleInit(void);
void gpioIsrInit(gpio_num_t sensor1, gpio_num_t sensor2, gpio_num_t user, gpio_num_t led);
 void initEspNow(uint8_t _wifiChannel);