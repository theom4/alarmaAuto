/******************************/
//ESP32 to implement the alarm!
#include "HomeSpan.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
/******************************************************************************************************************/
/*
      AICI POTI SCHIMBA CONFIGURATIILE:
*/
#define MAIN_MAC "30:AE:A4:4D:16:94" //Adresa MAC a celeilalte placi
gpio_num_t SENSOR_PIN = GPIO_NUM_5; // pinul folosit pentru primul senzor de vibratie (GPIO_NUM_X)
gpio_num_t SECOND_SENSOR_PIN = GPIO_NUM_6; // pinul folosit pentru al doilea senzor de vibratie (GPIO_NUM_X)
#define RELAY_PIN 7 // pinul pentru releu ce declanseaza alarma
#define PULSE_THRESHOLD 40 //cate pulsuri de la senzor sa primesti ca sa declansezi alarma
#define COOLDOWN_PERIOD 900 // perioada minima in milisecunde intre 2 declansari contigue a alarmei 
#define WINDOW_SIZE 1000//perioada in milisecunde in care se numara pulsurile de la senzor
#define USER_BUTTON GPIO_NUM_9 // buton utilizator - pentru testare
#define KEEPALIVE_MS 2000 //keep-alive message interval in MS
#define CANAL_WIFI 11 //canalul folosit de ESP-NOW
#define RELAY_PIN 6 //pin pentru conexiunea la releul ce decclanseaza buzerr-ul

//Decomenteaza aici daca vrei ca ESP-ul sa pirmeasca date prin ESP-NOW, nu doar sa transmita
//#define ESP_NOW_RX_ENABLE 

/******************************************************************************************************/
#define ALARM_SIGNAL 0xDEAD // signal for telling the ESP32 that the alarm has been detected
SpanPoint* mainDev = NULL; //ESP-NOW object
size_t pulseCount =0;
size_t windowStartTime = 0;
size_t windowCurrentTime = 0;
size_t lastAlarmTime = 0;
size_t lastKaTime =0 ; //for ESP-NOW delay 
//////////
//Use this variable only as a base source!
size_t sysCurrTime = 0;
/////////
uint16_t rx_byte = 0, tx_byte = 0;
struct alarmTimeStruct {
    size_t pulseCount;
    size_t windowStartTime;
    size_t windowCurrentTime;
    size_t lastAlarmTime;
    bool isAlarmTriggered;
    alarmTimeStruct() 
        : pulseCount(0), windowStartTime(0), windowCurrentTime(0), 
          lastAlarmTime(0), isAlarmTriggered(false) {}
};
alarmTimeStruct senzor1 ;
alarmTimeStruct senzor2 ;
bool alarmTriggered = false;
static void triggerBuzzer(void)
{
  //digitalWrite(RELAY_PIN, LOW);
}
static void stopBuzzer(void)
{
  //digitalWrite(RELAY_PIN, HIGH);
}
static inline void sendAlarmSignal(alarmTimeStruct& sensorStruct)
{
      sensorStruct.isAlarmTriggered = false;
      alarmTriggered = false;
      sensorStruct.lastAlarmTime = millis();
      lastAlarmTime = millis();
      Serial.println("ALARM TRIGGERED!!!");
      tx_byte = ALARM_SIGNAL;
      mainDev->send(&tx_byte);
}
static void triggerAlarm(void)
{
  //Send the alarm to the central hub and wait for feedback!
}
 void IRAM_ATTR vibrationISR()
{
    sysCurrTime = millis();
    //pentru primul senzor
    // if((senzor1.windowStartTime == 0) && ((sysCurrTime - senzor1.lastAlarmTime)> COOLDOWN_PERIOD)) //start counting the pulses
    // {
    //     senzor1.windowStartTime = sysCurrTime;
    //     pulseCount = 1;
    //     senzor1.pulseCount = 1;
    //     Serial.println("-");
    // }
    // if(senzor1.windowStartTime != 0)//counting started, inc pulseCount
    // {
    //   pulseCount++;
    //   senzor1.pulseCount++;
    //   Serial.println("+");
    // }
    // if(sysCurrTime - senzor1.windowStartTime >= WINDOW_SIZE)
    // {
    //   if(senzor1.pulseCount >= PULSE_THRESHOLD) //alarma se declanseaza
    //   {
    //       senzor1.lastAlarmTime = millis();
    //       senzor1.pulseCount = 0;
    //       alarmTriggered = true;
    //   }
    // }
    if((windowStartTime == 0) && ((sysCurrTime - lastAlarmTime)> COOLDOWN_PERIOD)) //start counting the pulses
    {
        windowStartTime = sysCurrTime;
        pulseCount = 1;
        Serial.println("-");
    }
    if(windowStartTime != 0)//counting started, inc pulseCount
    {
      pulseCount++;
      senzor2.pulseCount++;
      Serial.println("+");
    }
    if(sysCurrTime - windowStartTime >= WINDOW_SIZE)
    {
      if(pulseCount >= PULSE_THRESHOLD) //alarma se declanseaza
      {
          lastAlarmTime = millis();
          pulseCount = 0;
          alarmTriggered = true;
      }
    }
}
//ISR pentru al doilea senzor
void IRAM_ATTR vibration2ISR()
{
    sysCurrTime = millis();
    // if((senzor2.windowStartTime == 0) && ((sysCurrTime - senzor2.lastAlarmTime)> COOLDOWN_PERIOD)) //start counting the pulses
    // {
    //     senzor2.windowStartTime = sysCurrTime;
    //     pulseCount = 1;
    //     senzor2.pulseCount = 1;
    //     Serial.println("-");
    // }
    // if(senzor2.windowStartTime != 0)//counting started, inc pulseCount
    // {
    //   pulseCount++;
    //   senzor2.pulseCount++;
    //   Serial.println("+");
    // }
    // if(sysCurrTime - senzor2.windowStartTime >= WINDOW_SIZE)
    // {
    //   if(senzor2.pulseCount >= PULSE_THRESHOLD) //alarma se declanseaza
    //   {
    //       senzor2.lastAlarmTime = millis();
    //       senzor2.pulseCount = 0;
    //       alarmTriggered = true;
    //   }
    // }
    if((windowStartTime == 0) && ((sysCurrTime - lastAlarmTime)> COOLDOWN_PERIOD)) //start counting the pulses
    {
        windowStartTime = sysCurrTime;
        pulseCount = 1;
        senzor2.pulseCount = 1;

    }
    if(windowStartTime != 0)//counting started, inc pulseCount
    {
      pulseCount++;
      senzor2.pulseCount++;
      Serial.println("+");
    }
    if(sysCurrTime - windowStartTime >= WINDOW_SIZE)
    {
      if(pulseCount >= PULSE_THRESHOLD) //alarma se declanseaza
      {
          lastAlarmTime = millis();
          pulseCount = 0;
          alarmTriggered = true;
      }
    }
}

static void userISR(void) //push-button ISR
{
    //Emulate here the triggering of an alarm for testing purposes
    Serial.println("@");

}
void setup() 
{
  //  rtc_wdt_protect_off(); // Disable write protection
  //   rtc_wdt_disable();     // Disable the RTC watchdog
   
   Serial.begin(115200);
    delay(100);

    SpanPoint::setEncryption(false);// no password
    Serial.println("Initializare Alarma Auto");
    //1 byte for RX and TX sizes
    esp_wifi_set_channel(CANAL_WIFI,(wifi_second_chan_t)0);
    mainDev = new SpanPoint(MAIN_MAC, sizeof(uint16_t), sizeof(uint16_t));
    esp_wifi_set_channel(CANAL_WIFI,(wifi_second_chan_t)0);

   // homeSpan.setLogLevel(1);
 
    pinMode(SENSOR_PIN,INPUT);
    pinMode(SECOND_SENSOR_PIN,INPUT);
    pinMode(USER_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(USER_BUTTON),userISR,FALLING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN),vibrationISR,RISING);
    attachInterrupt(digitalPinToInterrupt(SECOND_SENSOR_PIN),vibration2ISR,RISING);
   

    sysCurrTime = millis();
      //Decomenteaza asta daca vrei sa afli canalul folosit de ESP32
     
      // Serial.println("CHANNEL USED ::");
      // Serial.println(WiFi.channel(););
      
      // Serial.println("This is the MAC ------>" + WiFi.macAddress(););
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
     if(sysCurrTime - lastKaTime > KEEPALIVE_MS)
    {
        uint8_t keepalive = 0x4F4B;
        bool st = mainDev->send(&keepalive);
        if(st == false)
        {
          Serial.println("ESP-NOW TX Fail!");
        }
        lastKaTime = sysCurrTime;
    }
}
void loop() 
{
    sysCurrTime = millis();
    if(sysCurrTime - windowStartTime > WINDOW_SIZE && windowStartTime !=0)
    {
      if(pulseCount < PULSE_THRESHOLD)
      {
          pulseCount = 0; //reset everything as not enough pulses were counted
          windowStartTime = 0;
      }
    }

    if(alarmTriggered == true)
    {
      sendAlarmSignal(senzor1);
    }

    if(senzor1.isAlarmTriggered == true)
    {
      senzor1.isAlarmTriggered = false;      
      Serial.println("ALARM TRIGGERED!!!");
      tx_byte = ALARM_SIGNAL;
      mainDev->send(&tx_byte);
      senzor1.lastAlarmTime = millis();
      sendAlarmSignal(senzor1);

    }
     if(senzor2.isAlarmTriggered == true)
    {
       sendAlarmSignal(senzor2);
    }
  
#ifdef ESP_NOW_RX_ENABLE
  checkIncomingData();
#endif

}
