/******************************/
//ESP32 to implement the alarm!
#include "HomeSpan.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#define MAIN_MAC "30:AE:A4:4D:16:94"
int cnt ;
gpio_num_t SENSOR_PIN = GPIO_NUM_5;
size_t timeout = 0; // tineout pentru conexiunea deintre alarma si esp-ul conectat la Home
SpanPoint* mainDev = NULL;
#define TAG "AlarmAuto"
#define PULSE_THRESHOLD 40 //cate pulsuri de la senzor sa primesti(in milisecunde) ca sa declansezi alarma
#define COOLDOWN_PERIOD 900 // perioada minima in milisecunde intre 2 declansari contigue a alarmei 
#define WINDOW_SIZE 1000//perioada in milisecunde in care se numara pulsurile de la senzor
#define USER_BUTTON GPIO_NUM_9 // buton utilizator - pentru testare
#define KEEPALIVE_MS 2000 //keep-alive message interval in MS
#define CANAL_WIFI 1
#define ALARM_SIGNAL 0xDEAD // signal for telling the ESP32 that the alarm has been detected
size_t pulseCount =0;
size_t windowStartTime = 0;
size_t windowCurrentTime = 0;
size_t lastAlarmTime = 0;
size_t currentTime = 0 ;
size_t lastTime =0 ;
uint16_t rx_byte = 0, tx_byte = 0;
//
bool alarmTriggered = false;
//
static void AlarmTask(void* pvArg)
{
  Serial.println("Starting the HomeSpan task!");
  while(1)
  {
    //check if the vibration sensor iss active here...
    //trigger buzzer if active
    //Send the alarm warning via ESP-NOW 
  }
  vTaskDelete(NULL);
}
static void triggerAlarm(void)
{
  //Send the alarm to the central hub and wait for feedback!
}
 void IRAM_ATTR vibrationISR()
{
    currentTime = millis();
    //Serial.println("/");

    if((windowStartTime == 0) && ((currentTime - lastAlarmTime)> COOLDOWN_PERIOD)) //start counting the pulses
    {
        windowStartTime = currentTime;
        pulseCount = 1;
        Serial.println("-");

    }
    if(windowStartTime != 0)//counting started, inc pulseCount
    {
      pulseCount++;
      Serial.println("+");
    }
    if(currentTime - windowStartTime >= WINDOW_SIZE)
    {
      if(pulseCount >= PULSE_THRESHOLD) //alarma se declanseaza
      {
          lastAlarmTime = millis();
          pulseCount = 0;
          alarmTriggered = true;
      }
    }
}
static void gpio_isr_init(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(SENSOR_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(SENSOR_PIN,GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(SENSOR_PIN,GPIO_PULLDOWN_ONLY));
    ESP_ERROR_CHECK(gpio_set_intr_type(SENSOR_PIN,GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
   // ESP_ERROR_CHECK(gpio_isr_handler_add(SENSOR_PIN, vibrationISR,NULL));
}
static void userISR(void)
{
    //Emulate here the triggering of an alarm for testing purposes
    Serial.println("@");

}
void setup() 
{
    Serial.begin(115200);
    delay(100);

    SpanPoint::setEncryption(false);// no password
    Serial.println("Initializare Alarma Auto");
    //1 byte for RX and TX sizes
    esp_wifi_set_channel(11,(wifi_second_chan_t)0);
    mainDev = new SpanPoint(MAIN_MAC, sizeof(uint16_t), sizeof(uint16_t));
    esp_wifi_set_channel(11,(wifi_second_chan_t)0);

   // homeSpan.setLogLevel(1);
  //xTaskCreate(AlarmTask, "Task Alarma",4096,NULL,5,NULL);
  //gpio_isr_init();
    pinMode(SENSOR_PIN,INPUT);
    pinMode(USER_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(USER_BUTTON),userISR,FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN),vibrationISR,RISING);
      // int channel =0;
      // channel = WiFi.channel();
      // Serial.println("CHANNEL USED ::");
      // Serial.println(channel);
      // String mac_addr = WiFi.macAddress();
      // Serial.println("This is the MAC ------>" + mac_addr);
}

void loop() 
{
    currentTime = millis();
    if(currentTime - lastTime > KEEPALIVE_MS)
    {
        uint8_t keepalive = 0x4F4B;
        bool st = mainDev->send(&keepalive);

        Serial.println("ESP-NOW TX:" + String(st == 1?"OK":"FAIL'"));
        lastTime = currentTime;
      
        //send data via ESP-NOW for satying connected!
        //also get the current time, if wanted...
    }
    if(currentTime - windowStartTime > WINDOW_SIZE && windowStartTime !=0)
    {
      if(pulseCount < PULSE_THRESHOLD)
      {
          pulseCount = 0; //reset everything as not enough pulses were counted
          windowStartTime = 0;
      }
    }
    if(alarmTriggered == true)
    {
      alarmTriggered = false;
      Serial.println("ALARM TRIGGERED!!!");
      tx_byte = ALARM_SIGNAL;
      mainDev->send(&tx_byte);
      lastAlarmTime = millis();
    }
    
    if(mainDev->get(&rx_byte) == true) //data received via ESP-NOW - could be the current time or the alarm to be armed
    {
          //parseRxByte - you may need to implement it via a queue and a task
          Serial.println("Received byte :" + String(rx_byte));
          //parseRxData(rx_byte);
          rx_byte = 0;
    } 
    
}
