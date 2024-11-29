
//ESP32 to connect to the Home app!
#include "HomeSpan.h"        
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_mac.h"
/*********************************************************************/
//              Aici poti schimba congifuratiile!
#define MAC_ALARMA "40:4C:CA:54:47:D8"
//#define MAC_ALARMA "A0:A3:B3:97:E9:00" //adresa MAC a alarmei auto 
#define LED_PIN 2 //pin-ul pentru LED
#define USER_BUTTON GPIO_NUM_0 //butonul de utilizator de pe ESP
#define CANAL_WIFI 1//pentru comunicatie ESP-NOW si WiFi
#define CONNECTION_TIMEOUT 5000 //timeout in milisecunde pentru conexiunea dintre ESP-uri,
                                //daca nu primeste niciun semnal de la alarma timp de
                                //CONECTION_TIMEOUT, se va trimite o notificare pe Home


/*********************************************************************/
bool alarmTriggered = false;
uint16_t tx_byte = 0xD1;
uint16_t rx_byte = 0;
size_t currentTime, lastTime;
SpanPoint* mainDevice = NULL;
bool alarmConnected = false;
bool alarmArmed = true; //assume the alarm is armed for now,
                        //it can be disarmed through home
bool isAlarmTaskRunning = false;
struct
{
    int x;
    //bool isAlarmTaskRunning;
    //bool
}autoAlarm_t;
struct vibrationSensor:Service::MotionSensor
{
 private:
     SpanCharacteristic* chMotionDetected;
     SpanCharacteristic* chStatusActive;//pentru detectarea starii alarmei(conectata/deconectata)
     SpanCharacteristic* chStatusFault;
     SpanCharacteristic* chStatusLowBattery; //pentru starea bateriei alarmei
  public:
     vibrationSensor():Service::MotionSensor()
     {
          chMotionDetected = new Characteristic::MotionDetected(0);
          chStatusActive = new Characteristic::StatusActive(1);
          chStatusFault = new Characteristic::StatusFault(0);
          chStatusLowBattery = new Characteristic::StatusLowBattery(0);//prima oara are baterie!
          Serial.println("Senzorul de vibratie creat!");
     }
     bool update()
     {
       Serial.println("Eveniment legat de Alarma!!");
       return true;
     }
     bool setMotionState(bool state)
     {
        chMotionDetected->setVal(state);// state detected presence!!
        Serial.println("Stare senzor miscare schimbata in:" + String(state?"Detectat":"Nedectat"));
        return true;
     }

};
struct controlAlarma:Service::SecuritySystem
{
     SpanCharacteristic* eroareAlarma;
     SpanCharacteristic* stareAlarmaTinta;
     SpanCharacteristic* stareAlarmaCurenta;
     controlAlarma():Service::SecuritySystem()
     {
        eroareAlarma = new Characteristic::StatusFault();
        stareAlarmaTinta = new Characteristic::SecuritySystemTargetState();
        stareAlarmaCurenta  = new Characteristic::SecuritySystemCurrentState();
     }
     bool update()
     {
      uint8_t alarmState = stareAlarmaTinta->getNewVal();
      Serial.print("Eveniment legat de Alarma! stareAlarmaTinta:");
      Serial.println(alarmState);
      if(alarmState == 3)
      {
        alarmArmed = false;
        Serial.println("Alarma dezarmata, senzorii inca merg dar nu vei fi notificat la declansare");
      }
      // Serial.print("Eveniment legat de Alarma! stareAlarmaCurenta:");
      // Serial.println(stareAlarmaCurenta->getNewVal());
      return true;
     }
};
struct butonAlarma:Service::Switch
{
    SpanCharacteristic* stareButon;
    SpanCharacteristic* numeButon;//Pentru denumirea ulterioara a butonului, eg Activare senzor
    butonAlarma():Service::Switch()
    {
      stareButon = new Characteristic::On(0);//prima oara este inactiv
      numeButon = new Characteristic::ConfiguredName("butonAlarma");
      Serial.println("ButonAlarma creat!");
    }
    bool update()
    {
      Serial.print("Stare buton schimbata:");
      Serial.println(stareButon->getNewVal() == 1?"Activ" : "Inactiv");
      return true;
    }
};
struct SistemSecuritate:Service::ContactSensor
{
 private: 
    SpanCharacteristic* sensorState; // pentru detectarea prezentei umane
    SpanCharacteristic* sensorActive; //for detecting the state of the sensor(connected/unconnetcted)
    SpanCharacteristic* batteryState;
    SpanCharacteristic* name;
  public:
    SistemSecuritate():Service::ContactSensor()
    {
      sensorState = new Characteristic::ContactSensorState(0); //not detected
      sensorActive = new Characteristic::StatusActive(1);//functioning
      batteryState = new Characteristic::StatusLowBattery(0); //NOT_LOW_BATTERY
      name = new Characteristic::ConfiguredName("SenzoriVibratie");
    }
    bool update()
    {
      uint8_t currentSensorState = sensorState->getNewVal();
      if(sensorState == 0)
      {
        Serial.println("Vibratie Detectata!");
      }
      return true;
    }
    bool setPresenceDetected(bool detected)
    {
      sensorState->setVal(detected);
      Serial.println("Stare senzor contact schimbata in:" + String(detected?"Nedectat":"Detectat"));
      return true;
    }
};
struct ledESP:Service::LightBulb
{
    private:
      int pinLed;
      SpanCharacteristic* stareLED;
      SpanCharacteristic* numeLED;
      String ledName;
    public:
      ledESP(int pin, String name):Service::LightBulb()
      {
          this->pinLed = pin;
          this->ledName = name;
          pinMode(this->pinLed, OUTPUT);
          stareLED = new Characteristic::On(1);
          numeLED = new Characteristic::ConfiguredName(this->ledName.c_str());

      }
    bool update()
    {
        bool newState = stareLED->getNewVal();
        Serial.println("Stare LED schimbata in:" + String(newState));
        digitalWrite(this->pinLed, newState);
        return true;
    }
};
vibrationSensor* _vibrationSensor; //for the motion sensor detection....
SistemSecuritate* _SistemSecuritate;//for the contact sensor....
ledESP* ledPeESP;
static void alarmHandlingTask(void* pvArg)
{
  //get the alarm state via pvArg! - safety approach
  isAlarmTaskRunnning = true;
  Serial.println("ALARMA DECLANSATA!");
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.println("ALARMA OPRITA!");
  taskENTER_CRITICAL();
  alarmTriggered = false;
  isAlarmTaskRunning = false;
  taskENTER_CRITICAL();
  _SistemSecuritate->setPresenceDetected(0);
  _vibrationSensor->setMotionState(1);
  vTaskDelete(NULL);
}
void esp_now_task(void* pvArg)
{
  size_t lastReceivedTime = 0;
  bool printedConnection = false;
bool printedDisconnection = false;
  size_t _currentTime = millis();
  Serial.println("Starting the ESP-NOW Task...");
  while(1)
  {
      _currentTime = millis();
      if(mainDevice->get(&rx_byte) == true)//data received via ESP-NOW
      {
          Serial.println("ESP-NOW:" + String(rx_byte));       
          if(rx_byte == 0xDEAD) //alarm triggered
          {
              if(alarmArmed == true)
              {
                    
                    alarmTriggered = true;
                    _SistemSecuritate->setPresenceDetected(0);
                    _vibrationSensor->setMotionState(1);
                    if(isAlarmTaskRunning == false && alarmArmed == true)
                    {
                      xTaskCreate(alarmHandlingTask, "alarmHandlingTask", 2048, NULL, 5, NULL);

                    }
              }
              else
              {
                  Serial.println("Vibratie detectata, alarma nu va fi declansata, este dezarmata");
              }
           }    
           if (!alarmConnected) 
           {
                alarmConnected = true;
               
                Serial.println("-Alarma conectata!-----------");
            }
          lastReceivedTime = _currentTime;
      }
      else if (_currentTime - lastReceivedTime > CONNECTION_TIMEOUT && alarmConnected) 
      {
            alarmConnected = false;
            
            Serial.println("Alarma deconectata!----------");
        }
      vTaskDelay(pdMS_TO_TICKS(3));
  }
  vTaskDelete(NULL);
}
static void gpio_isr(void)
{
  //reset the alarm state to no triggered
  _SistemSecuritate->setPresenceDetected(0);
  _vibrationSensor->setMotionState(1);
}
void setup() {

   
  Serial.begin(115200); 
  
  homeSpan.begin(Category::SecuritySystems,"Alarma Auto ESP32");  

  new SpanAccessory();                            
  
    new Service::AccessoryInformation();                
      new Characteristic::Identify();                       
      
    _vibrationSensor = new vibrationSensor();
    new Service::BatteryService();
      new Characteristic::BatteryLevel();
      new Characteristic::StatusLowBattery();

   ledPeESP = new ledESP(LED_PIN, "LED_ESP32");
       new butonAlarma();
    
    new controlAlarma();
   _SistemSecuritate = new SistemSecuritate();    
      pinMode(2, OUTPUT);
      digitalWrite(2 , HIGH);
      SpanPoint::setEncryption(false);
      mainDevice = new SpanPoint(MAC_ALARMA, sizeof(uint16_t),sizeof(uint16_t));
      xTaskCreate(esp_now_task,"esp_now_task", 4095 * 2 ,NULL, 4, NULL);
      //Decomenteaza sectiunea asta daca vrei sa afli canalul de WiFi pentru ESP-NOW
      // int channel =0;
      // channel = WiFi.channel();
      // Serial.println("CHANNEL USED ::");
      // Serial.println(channel);
      // String mac_addr = WiFi.macAddress();
      // Serial.println("This is the MAC ------>" + mac_addr);
      pinMode(USER_BUTTON, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(USER_BUTTON),gpio_isr,FALLING);
       //  xTaskCreate(homeSpanTask, "homeSpanTask",2048*4, NULL,5,NULL);     
      //remember to implement ESP-NOW!!!
      //and the feedback mechanism!
}

void loop()
{
      homeSpan.poll();
}
