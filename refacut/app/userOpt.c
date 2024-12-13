#include "userOpt.h"
extern nvs_handle_t nvsHandle;
extern uint8_t sensorOnePin;
extern uint8_t sensorTwoPin;
extern uint8_t userPin;
extern uint8_t ledPin;
extern uint8_t macAddr[6];
extern uint8_t wifiChannel;
extern QueueHandle_t xGpioIsrQueue;
extern size_t sysCurrTime;
extern alarmTimeStruct sensorOne;
extern alarmTimeStruct sensorTwo;
#define TAG "userOpt.c"
size_t millis(void)
{
    return (esp_timer_get_time() * 0.001);
}
void esp_now_recv_callback(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len)
{
 printf("received data : %.*s", data_len, data);
}
void esp_now_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
   if(status != ESP_NOW_SEND_SUCCESS)
   {
       printf("Error sending ESP-NOW data");
   }
}
 void initEspNow(uint8_t _wifiChannel)
{
    esp_err_t ret = ESP_OK;
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_ERROR_CHECK(esp_wifi_start());
    uint8_t localMac[6] = {0x00};
    esp_read_mac(localMac, ESP_MAC_WIFI_STA);
    printf("Adresa MAC Locala: %02X:%02X:%02X:%02X:%02X:%02X\n", localMac[0], localMac[1], localMac[2], localMac[3], localMac[4], localMac[5]);
    ret |= esp_now_init();
    ret |= esp_now_register_recv_cb(esp_now_recv_callback);
    ret |= esp_now_register_send_cb(esp_now_send_callback);
    printf("ESP-NOW Init Status: %s\n", esp_err_to_name(ret));
     esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
}
bool nvsCheckCredentialsPresent(void)
{
    esp_err_t ret = ESP_OK;
    size_t len = 6;
    ret |= nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        ret |= nvs_flash_init();
    }
    ret |= nvs_open("storage", NVS_READWRITE, &nvsHandle);
    if(ret != ESP_OK )printf("Eroare la nvs_open: %s\n", esp_err_to_name(ret));
    //nvs_get_blob(nvsHandle, MAC_KEY_NAME, NULL, &len);
     ret = nvs_get_blob(nvsHandle, MAC_KEY_NAME, (char*)macAddr, &len) |
           nvs_get_u8(nvsHandle, WIFI_CHANNEL_KEY, &wifiChannel) |
           nvs_get_u8(nvsHandle, SENSOR_1_KEY, &sensorOnePin) |
           nvs_get_u8(nvsHandle, SENSOR_2_KEY, &sensorTwoPin) |
           nvs_get_u8(nvsHandle, USER_BTN_KEY, &userPin) |
           nvs_get_u8(nvsHandle, LED_KEY, &ledPin);
    if(ret != ESP_OK)
    {
        printf("Inca nu ai configurat alarma, hai sa incepem!\n");
        getMacAddr();
        getWifiChannel();
        getGpios();
        gpioIsrInit(sensorOnePin, sensorTwoPin, userPin,ledPin);
        
        return false;
    }
    else
    {
        printf("Dispozitivul e deja configurat! \n");
        printf("Configuratiile curente:\n");
        printf("Adresa MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
        printf("Canalul WiFi: %d\n", wifiChannel);
        printf("Pin-ul primului senzor de vibratie: %d\n", sensorOnePin);
        printf("Pin-ul celui de-al doilea senzor de vibratie: %d\n", sensorTwoPin);
        printf("Pin-ul butonului utilizator: %d\n", userPin);
        printf("Pin-ul ledului: %d\n", ledPin);
        gpioIsrInit(sensorOnePin, sensorTwoPin, userPin,ledPin);
        printf("poti apasa 'R' pentru a reseta configuratia\n");
    }
    return true;
}
void getMacAddr(void)
{
    printf("--------------------------------------\n");
    printf("Introdu adresa MAC a celeilate placi (exemplu: 30:AE:A4:4D:16:94 ):\n");
    char temp;
    bool ok = true;


    do {
        ok = true;
        for (int i = 0; i < 6; i++) {
            macAddr[i] = 0;
        }
        if (scanf("%2hhx%c%2hhx%c%2hhx%c%2hhx%c%2hhx%c%2hhx", 
                  &macAddr[0], &temp, 
                  &macAddr[1], &temp, 
                  &macAddr[2], &temp, 
                  &macAddr[3], &temp, 
                  &macAddr[4], &temp, 
                  &macAddr[5]) != 11) {
         while (getchar() != '\n');
         printf("Format Adresa MAC incorecta, mai incearca\n");
         ok = false;
        continue;
        }
         if (ok) {
            printf("Adresa MAC corecta: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                   macAddr[0], macAddr[1], macAddr[2], 
                   macAddr[3], macAddr[4], macAddr[5]);
            esp_err_t err = nvs_set_blob(nvsHandle, MAC_KEY_NAME, macAddr, sizeof(macAddr));
            if (err == ESP_OK) 
            {
                nvs_commit(nvsHandle);
            }
            break;
        }
    } while (1);
}
void getGpios(void)
{
   printf("Introdu pin-ul primului senzor de vibratie (0 - 39):\n");
    do
    {
        scanf("%"PRId8"", &sensorOnePin);
        if(sensorOnePin < 0 && sensorOnePin > 39)
        {
            printf("Pin %"PRId8" incorect, mai incearca!\n", sensorOnePin);
        }
        else
        {
            ESP_ERROR_CHECK(nvs_set_u8(nvsHandle, SENSOR_1_KEY, sensorOnePin));
            ESP_ERROR_CHECK(nvs_commit(nvsHandle));
            break;
        }
    }while(1);
    printf("Pin-ul primului senzor de vibratie activat: %"PRId8"\n", sensorOnePin);
    printf("Introdu pin-ul celui de-al doilea senzor de vibratie (0 - 39):\n");

    do
    {
        scanf("%"PRId8"", &sensorTwoPin);
        if(sensorTwoPin < 0 && sensorTwoPin > 39)
        {
            printf("Pin %"PRId8" incorect, mai incearca!\n", sensorTwoPin);
        }
        else
        {
            ESP_ERROR_CHECK(nvs_set_u8(nvsHandle, SENSOR_2_KEY, sensorTwoPin));
            ESP_ERROR_CHECK(nvs_commit(nvsHandle));
            break;
        }
    }while(1);
    printf("Pin-ul celui de-al doilea senzor de vibratie activat: %"PRId8"\n", sensorTwoPin);
    printf("La ce pin e conectat butonul de utilizator? (de obicei 0 sau 2):\n");
    do
    {
        scanf("%"PRId8"", &userPin);
        if(userPin < 0 && userPin > 39)
        {
            printf("Pin utlizator %"PRId8" incorect, mai incearca!\n", userPin);
        }
        else
        {
            ESP_ERROR_CHECK(nvs_set_u8(nvsHandle, USER_BTN_KEY, userPin));
            ESP_ERROR_CHECK(nvs_commit(nvsHandle));
            break;
        }
    }while(1);
    printf("Pin-ul butonului de utilizator activat: %"PRId8"\n", userPin);
    printf("La ce pin e conectat LED-ul? (de obicei 0 sau 2):\n");
    do
    {
        scanf("%"PRId8"", &ledPin);
        if(ledPin < 0 && ledPin > 39)
        {
            printf("Pin utlizator %"PRId8" incorect, mai incearca!\n", ledPin);
        }
        else
        {
            ESP_ERROR_CHECK(nvs_set_u8(nvsHandle, LED_KEY, ledPin));
            ESP_ERROR_CHECK(nvs_commit(nvsHandle));
            break;
        }
    }while(1);
    printf("Pin-ul LED activat: %"PRId8"\n", ledPin);
    printf("Configuratiile sunt gata!\n");
    nvs_close(nvsHandle);

}
void getWifiChannel(void)
{
     
     printf("Introdu canalul WiFi folosit de ESP-NOW (1 - 13):\n");
    do
    {
        
        while (getchar() != '\n');
        scanf("%"PRId8"", &wifiChannel);
        if(wifiChannel < 1 || wifiChannel > 13)
        {
            printf("Canal WiFi %"PRId8" incorect, mai incearca!\n", wifiChannel);
        }
        else
        {
            ESP_ERROR_CHECK(nvs_set_u8(nvsHandle, WIFI_CHANNEL_KEY, wifiChannel));
            ESP_ERROR_CHECK(nvs_commit(nvsHandle));
            break;
        }
    }while(1);
    printf("Canal WiFi introdus: %"PRId8"\n", wifiChannel);
}
void espConsoleInit(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    esp_vfs_dev_uart_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
}
 static void isrTask(void* pvArg)
{
    printf("start of isrTask\n");
    uint8_t gpioNum = -1;
    while(1)
    {
        static  size_t _pulseCount = 0;

        if(xQueueReceive(xGpioIsrQueue, &gpioNum, portMAX_DELAY))
        {           
            if(gpioNum == userPin)
            {
                printf("Buton Utilizator apasat!\n");
            }
            else if(gpioNum == sensorOnePin)
            {
                //printf("%d\n", _pulseCount++);
                if(sensorOne.windowStartTime == 0 &&
                   sysCurrTime - sensorOne.lastAlarmTime > 1200)
                   {
                       sensorOne.windowStartTime = sysCurrTime;
                       sensorOne.pulseCount = 1;
                   }      
                if(sensorOne.windowStartTime != 0)
                {
                    printf("%d\n",sensorOne.pulseCount++);
                }
                if(sysCurrTime - sensorOne.windowStartTime > 1000)
                {
                    if(sensorOne.pulseCount > 35)
                    {
                        sensorOne.isAlarmTriggered = true;
                        sensorOne.lastAlarmTime = millis();
                    }
                }       
            }
            else if(gpioNum == sensorTwoPin)
            {
                if(sensorOne.windowStartTime == 0 &&
                   sysCurrTime - sensorTwo.windowStartTime > 900)
                   {
                       sensorTwo.windowStartTime = sysCurrTime;
                       sensorTwo.pulseCount = 1;
                   }      
                if(sensorTwo.windowStartTime != 0)
                {
                    sensorTwo.pulseCount++;
                }
                if(sysCurrTime - sensorTwo.windowStartTime > 1000)
                {
                    if(sensorTwo.pulseCount > 35)
                    {
                        sensorTwo.isAlarmTriggered = true;
                        sensorTwo.lastAlarmTime = millis();
                    }
                }       
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}
static void gpioIsrHandler(void* pvArg)
{
   // ESP_DRAM_LOGI(TAG, "Button Pressed!");
   // printf("x");
   uint8_t pinCount = (uint8_t)pvArg;
   xQueueSendFromISR(xGpioIsrQueue, &pinCount, NULL);
}

void gpioIsrInit(gpio_num_t sensor1, gpio_num_t sensor2, gpio_num_t user, gpio_num_t led)
{
    printf("Se configureaza pinii...\n");
    gpio_reset_pin(sensor1);
    gpio_reset_pin(sensor2);
    gpio_reset_pin(user);
    gpio_reset_pin(led);
    gpio_set_direction(sensor1, GPIO_MODE_INPUT);
    gpio_set_direction(sensor2, GPIO_MODE_INPUT);
    gpio_set_direction(user, GPIO_MODE_INPUT);
    gpio_set_direction(led, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(sensor1, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(sensor2, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(user, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(sensor1,GPIO_INTR_POSEDGE );
    gpio_set_intr_type(sensor2,GPIO_INTR_POSEDGE );
    ESP_ERROR_CHECK(gpio_set_intr_type(user,GPIO_INTR_NEGEDGE ));
   // gpio_isr_register(gpioIsrHandler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(user, gpioIsrHandler, (void*)user));
    ESP_ERROR_CHECK(gpio_isr_handler_add(sensor1, gpioIsrHandler,(void*)sensor1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(sensor2, gpioIsrHandler,(void*)sensor2));

    xGpioIsrQueue = xQueueCreate(12, sizeof(uint8_t));
    xTaskCreate(isrTask, "isrTask", 4096, NULL, 1, NULL);
}