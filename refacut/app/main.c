#include <stdio.h>
#include "esp_now.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "string.h"
#include "esp_system.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "ctype.h"
#include "stdint.h"
#include <inttypes.h>
#include "userOpt.h"
#include "esp_timer.h"
#define ALARM_SIGNAL 0xDEAD // signal for telling the ESP32 that the alarm has been detected
#define KEEPALIVE_MS 2000
/****************************************/
uint32_t pulseCountLimit = 0; //for the vibration sensors...

size_t sysCurrTime = 0;
size_t lastKaTime = 0;
uint8_t sensorOnePin = -1;
uint8_t sensorTwoPin = -1;
uint8_t userPin = -1;
uint8_t ledPin = -1;
uint8_t macAddr[6] = {0};
uint8_t wifiChannel = 1;
uint8_t buzzerPin = 0;//pin to be connected to the Relay
QueueHandle_t xGpioIsrQueue = NULL;
nvs_handle_t nvsHandle = 0;
uint16_t rx_byte = 0x0000;//for receiving the data via ESP-NOW
bool alarmTriggered = false;
static const char* TAG = "main.c";
char i;

alarmTimeStruct sensorOne;
alarmTimeStruct sensorTwo;
/****************************************/


static void inputUser(void* arg)
{
    while(1)
    {
        scanf("%c", &i);
        
        if(i == 'R')
        {
            printf("Resetare configuratie!\n");
            nvs_flash_erase();
            esp_restart();
        }while (getchar() != '\n');
        printf("printed data:%c\n",i);
       
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    vTaskDelete(NULL);
}
static void triggerBuzzer(void)
{
  gpio_set_level(buzzerPin, 0);
}
static void stopBuzzer(void)
{
    gpio_set_level(buzzerPin, 1);
}
static void sendAlarmSignal(alarmTimeStruct* sensorStruct)
{
    sensorStruct->isAlarmTriggered = false;
    alarmTriggered = false;
    sensorStruct->lastAlarmTime = millis();
    printf("Alarma detectata!");
    uint16_t tx_byte = ALARM_SIGNAL;
    esp_now_send(macAddr, &tx_byte, 2);
}
void sendPeriodicKA(void)
{
  //  printf("Sending Keep-Alive message!\n");
    esp_err_t err = ESP_OK;
    uint16_t keepalive = 0x4F4B;
    err = esp_now_send(macAddr, &keepalive, 2);        
    if(err != ESP_OK)
    {
        printf("ESP-NOW TX KA Fail!\n");
    }
}
void app_main(void)
{
    espConsoleInit();   

    nvsCheckCredentialsPresent(); 
    initEspNow(wifiChannel);
    esp_now_peer_info_t peerInfo = {0};
    peerInfo.channel = 11;
    peerInfo.encrypt = false;
    memcpy(peerInfo.peer_addr, macAddr, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    xTaskCreate(inputUser, "inputUser", 2048, NULL, 5, NULL);
   
    uint16_t tx_byte = 12;
    uint64_t timeMS =0;
    while(1)
    {
        
        sysCurrTime = millis();
        if(sysCurrTime - lastKaTime > KEEPALIVE_MS)
        {
            sendPeriodicKA();
            lastKaTime = sysCurrTime;
        }
        if(sensorOne.isAlarmTriggered == true)
        {
            sendAlarmSignal(&sensorOne);
        }
        if(sensorTwo.isAlarmTriggered == true)
        {
            sendAlarmSignal(&sensorTwo);
        }
        if(sysCurrTime - sensorOne.windowStartTime > 2000 && sensorOne.windowStartTime != 0)
        {
            if(sensorOne.pulseCount < 300)
            {
                sensorOne.windowStartTime = 0;
                sensorOne.pulseCount = 0;
            }
        }
        if(sysCurrTime - sensorTwo.windowStartTime > 2000 && sensorTwo.windowStartTime != 0)
        {
            if(sensorTwo.pulseCount < 300)
            {
                sensorTwo.windowStartTime = 0;
                sensorTwo.pulseCount = 0;
            }
        }
      
        vTaskDelay(pdMS_TO_TICKS(8));
        
    }
}
