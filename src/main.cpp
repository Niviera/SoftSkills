#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SPI.h>
#include <Wire.h>
#include <MQ135.h>
#include "display.h"
#include <esp_now.h>

/* Spezifische Pin Sender*/
// #define DHTPIN 26
// #define MQPIN 34

#define LICHTPIN 35
#define REGENSENSOR 32

/* Spezifische Pin Empfänger */
#define DHTPIN 32
#define MQPIN 33

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Uncomment the type of sensor in use:
#define DHTTYPE DHT11 // DHT 11

int display_content = 0;
DHT_Unified dht(DHTPIN, DHTTYPE);
MQ135 mq(MQPIN);
Display display;
uint8_t broadcastAddress[] = {0x24, 0xD7, 0xEB, 0x18, 0xEB, 0x64};
esp_now_peer_info_t peerInfo;
typedef struct struct_message
{
  float humidity;
  float temperatur;
  bool rain;
  float brightness;
  float co2;
} struct_message;
struct_message myData;

uint32_t delayMS;

void send_Data()
{
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK)
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  /* Debug der Sensoren ohne OLED */
  Serial.print("CO2: ");
  Serial.println(myData.co2);
  Serial.print("Humidity: ");
  Serial.println(myData.humidity);
  Serial.print("Temperature: ");
  Serial.println(myData.temperatur);
  Serial.print("Rain: ");
  Serial.println(myData.rain);
  Serial.print("Brightness: ");
  Serial.println(myData.brightness);
  Serial.println();
}
void sender_loop()
{
  /* Temperatur + Humidity */
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    Serial.println(F("Error reading temperature!"));
  }
  else
  {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    myData.temperatur = event.temperature;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    Serial.println(F("Error reading humidity!"));
  }
  else
  {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    myData.humidity = event.relative_humidity;
  }
  /* MQ135 aka C02 */
  Serial.print("MQ: ");
  Serial.println(mq.getPPM());
  Serial.print("Analog: ");
  Serial.println(analogRead(MQPIN));
  myData.co2 = mq.getPPM();

  /* Lichtsensor */
  Serial.print("Lichtsensor: ");
  Serial.println(analogRead(LICHTPIN));
  myData.brightness = analogRead(LICHTPIN);

  /* Regensensor */
  Serial.print("Regen: ");
  Serial.println(analogRead(REGENSENSOR));
  myData.rain = analogRead(REGENSENSOR);

  /* Sende Daten */
  send_Data();
  delay(2000);
}
void empfaenger_loop()
{
  float temp;
  float humi;
  float innen_co2;
  /* Alles was der Empfänger machen soll */

  /* DHT */
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    Serial.println(F("Error reading temperature!"));
    temp = 25;
  }
  else
  {
    Serial.print(F("Temperatur: "));
    Serial.print(event.temperature);
    Serial.println(F("C"));
    temp = event.temperature;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    Serial.println(F("Error reading humidity!"));
    humi = 60;
  }
  else
  {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    humi = event.relative_humidity;
  }

  Serial.print("MQ: ");
  Serial.println(mq.getPPM());
  Serial.print("Analog: ");
  Serial.println(analogRead(MQPIN));

  switch (display_content)
  {
  case 0:
    display.printTempeture(&temp, "Innen:");
    display_content++;
    break;
  case 1:
    display.printTempeture(&myData.temperatur, "Ausen:");
    display_content++;
    break;
  case 2:
    display.printHumi(&humi, "RaumHumi:");
    display_content++;
    break;
  case 3:
    display.printHumi(&myData.humidity, "AusHumi:");
    display_content++;
    break;
  case 4:
    innen_co2 = mq.getPPM();

    if (innen_co2 <= 1000)
    {
      display.printCo2(&innen_co2, "--Luften");
    }
    else if (innen_co2 <= 2000)
    {
      display.printCo2(&innen_co2, "-+Luften");
    }
    else
    {
      display.printCo2(&innen_co2, "++Luften");
    }
    display_content++;
    break;
  case 5:
    if (myData.rain >= 4000)
    {
      display.writeStatusMSG("Kein Regen");
    }
    else if (myData.rain >= 2000)
    {
      display.writeStatusMSG("Leichter Regen");
    }
    else if (myData.rain >= 1000)
    {
      display.writeStatusMSG("Regen");
    }
    else
    {
      display.writeStatusMSG("Starker Regen");
    }
    display_content++;
    break;
  case 6:

    if (myData.brightness >= 3000)
    {
      display.writeStatusMSG("Es ist Dunkel");
    }
    else if (myData.brightness >= 2000)
    {
      display.writeStatusMSG("Es ist Daemmerung");
    }
    else if (myData.brightness >= 1000)
    {
      display.writeStatusMSG("Es ist hell");
    }
    else
    {
      display.writeStatusMSG("Es ist  sehr hell");
    }
    display_content = 0;
    break;
  }

  delay(5000);
}
void sender_setup()
{
  pinMode(LICHTPIN, INPUT);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}
void empfaenger_setup()
{

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  /* Setup Reciecver */
  esp_now_register_recv_cb(OnDataRecv);
  /* Verbinde mit potenzielles OLED pannel */
  if (!display.connectDisplay())
  {
    Serial.println("Kein Display gefunden");
  }
  else
  {
    display.writeStatusMSG("Wilkommen");
  }
}
void setup()
{
  WiFi.mode(WIFI_STA);

  Serial.begin(9600);
  // Initialize device.

  dht.begin();
  pinMode(MQPIN, INPUT);
  empfaenger_setup();
  // sender_setup();
}

void loop()
{
  Serial.println(WiFi.macAddress());
  empfaenger_loop();
  // sender_loop();
}