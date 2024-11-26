/******************************/
//ESP32 to implement the alarm!
#include "HomeSpan.h"
#include "esp_task_wdt.h"
#include <rtc_wdt.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
/******************************************************************************************************************/
/*
      AICI POTI SCHIMBA CONFIGURATIILE:
*/
#define MAIN_MAC "30:AE:A4:4D:16:94" //Adresa MAC a celeilalte placi
gpio_num_t SENSOR_PIN = GPIO_NUM_5; // pinul folosit pentru primul senzor de vibratie
gpio_num_t SECOND_SENSOR_PIN = GPIO_NUM_6; // pinul folosit pentru al doilea senzor de vibratie
#define PULSE_THRESHOLD 40 //cate pulsuri de la senzor sa primesti(in milisecunde) ca sa declansezi alarma
#define COOLDOWN_PERIOD 900 // perioada minima in milisecunde intre 2 declansari contigue a alarmei 
#define WINDOW_SIZE 1000//perioada in milisecunde in care se numara pulsurile de la senzor
#define USER_BUTTON GPIO_NUM_9 // buton utilizator - pentru testare
#define KEEPALIVE_MS 2000 //keep-alive message interval in MS
#define CANAL_WIFI 1
#define RELAY_PIN 6 //pin pentru conexiunea la releul ce decclanseaza buzerr-ul
//Decomenteaza aici daca vrei ca ESP-ul sa pirmeasca data prin ESP-NOW, nu doar sa transmita
//#define ESP_NOW_RX_ENABLE 

/******************************************************************************************************/
#define ALARM_SIGNAL 0xDEAD // signal for telling the ESP32 that the alarm has been detected
SpanPoint* mainDev = NULL;
size_t pulseCount =0;
size_t windowStartTime = 0;
size_t windowCurrentTime = 0;
size_t lastAlarmTime = 0;
size_t currentTime = 0 ;
size_t lastTime =0 ;
//////////
//Use this variable only as a base source!
size_t systemCurrentTime = 0;
/////////
uint16_t rx_byte = 0, tx_byte = 0;
struct
{
  size_t pulseCount =0;
  size_t windowStartTime = 0;
  size_t windowCurrentTime = 0;
  size_t lastAlarmTime = 0;
  size_t currentTime = 0 ;
  size_t lastTime =0 ;
}alarmTime_t;
alarmTime_t senzor1 = {0};
alarmTime_t senzor2 = {0};
bool alarmTriggered = false;

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
//Sub dezvoltare, se adauga
void IRAM_ATTR vibration2ISR()
{
    currentTime = millis();
      //Serial.println("/");

    if((windowStartTime == 0) && ((currentTime - lastAlarmTime)> COOLDOWN_PERIOD)) //start counting the pulses
    {
        windowStartTime = currentTime;
        pulseCount = 1;
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

static void userISR(void)
{
    //Emulate here the triggering of an alarm for testing purposes
    Serial.println("@");

}
void setup() 
{
   rtc_wdt_protect_off(); // Disable write protection
    rtc_wdt_disable();     // Disable the RTC watchdog
   
   Serial.begin(115200);
    delay(100);

    SpanPoint::setEncryption(false);// no password
    Serial.println("Initializare Alarma Auto");
    //1 byte for RX and TX sizes
    esp_wifi_set_channel(11,(wifi_second_chan_t)0);
    mainDev = new SpanPoint(MAIN_MAC, sizeof(uint16_t), sizeof(uint16_t));
    esp_wifi_set_channel(11,(wifi_second_chan_t)0);

   // homeSpan.setLogLevel(1);
 
    pinMode(SENSOR_PIN,INPUT);
    pinMode(USER_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(USER_BUTTON),userISR,FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN),vibrationISR,RISING);
    attachInterrupt(digitalPinToInterrupt(SECOND_SENSOR_PIN),vibration2ISR,RISING);

      // int channel =0;
      // channel = WiFi.channel();
      // Serial.println("CHANNEL USED ::");
      // Serial.println(channel);
      // String mac_addr = WiFi.macAddress();
      // Serial.println("This is the MAC ------>" + mac_addr);
}
//Send keep-alive message via ESP-NOW
#ifdef ESP_NOW_RX_ENABLE
  static void checkIncomingData(void)
  {
    if(mainDev->get(&rx_byte) == true) //data received via ESP-NOW - could be the current time or the alarm to be armed
    {
          Serial.println("Received byte :" + String(rx_byte));
          rx_byte = 0;
    } 
  }
#endif
inline void sendPeriodicKA(void)
{
     if(currentTime - lastTime > KEEPALIVE_MS)
    {
        uint8_t keepalive = 0x4F4B;
        bool st = mainDev->send(&keepalive);
        if(st == 0)
        {
          Serial.println("ESP-NOW TX Fail!");
        }
        lastTime = currentTime;
    }
}
void loop() 
{
    currentTime = millis();
    
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
  
#ifdef ESP_NOW_RX_ENABLE
  checkIncomingData();
#endif

    
}
