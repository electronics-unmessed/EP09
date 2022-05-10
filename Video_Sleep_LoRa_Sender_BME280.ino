
// ******** LoRa und BME280 *********************************************
// **********************************************************************
//
// 02/17/2021   Extensions: Serial Print, as there is no display
//              Height adjustment -> pressure based on sea level
//              is already going pretty well
// 02/19/2021   Sensor ID and battery status are transferred
// 02/20/2021   OLED ad is commented out because it does not exist
//              Test with 10 sec deep sleep -> battery would then have to be approx. 18 days instead of 30 hours
//              keep.
// 02/22/2021   Deep Sleep power consumption reduced to 15 mA. Normal operation
//              takes 60 mA. A LoRa pulse at SF7 lasts 60 ms (actually 41).
//              Duty cycle is around 0.6% at 1x in 10 seconds. So legally.
//              That would be about 500 sec usage. Later with 1x in 3 min, then 28.8 sec.
//              TTN fair use policy: max 30 sec per day.
//              Sending increased from 10 sec to 180 sec -> DC = 0.033%. That's right, too
//              match with measurement.
// 04/03/2021   With LoRa.sleep () power consumption reduced from 15mA to 5.9mA. So my sensor should
//              run approx. 3000h / 5.9 = 500h = 21 days. That would be okay.
//              LiIon charge controller TP4056 installed for solar version.
//              Solar operation: 100 cm2 solar module would have to be sufficient for 20% of the maximum yield over the winter.
//              After LOS calculation and measurement with AD8 ... power meter: Board TX Power <0dBm, is
//              very little, but enough for my purposes.
//              Tests with LoRa.setTxPower (1 .. 17) does not change anything in terms of the transmission power. Right library?
//              Correct output parameters?
//
// Findings:    With Deep Sleep you have to pay close attention to the order of the commands,
//              because it is practically always restarted !!!!!
//              The power consumption in deep sleep mode is way too high !!
//              I measured 15 mA / 6 mA with all options in deep sleep mode.
//              If it starts with a battery, then 12.8 mA are again in deep sleep mode ??? !!!
//              Apparently you should get into the microAmpere range, I have to have it ..
//              It is probably just because of the ESP32 board that these sleep modes are not working properly
//              supports. -> Board suggestions are: Firebeetle, DFrobot
//
//
// 17.02.2021   Erweiterungen: Serial Print, da kein Display vorhanden
//                             Höhenanpassung -> Druck bezogen auf Meereshöhe
//                             läuft schon ganz gut
// 19.02.2021   Sensor ID und Akku Stand wird übertragen
// 20.02.2021   OLED Anzeige ist auskommentiert, da nicht vorhanden
//              Test mit 10 sec Deep Sleep -> Akku müsste dann anstatt 30h ca. 18 Tage 
//              halten.
// 22.02.2021   Deep Sleep Stromverbrauch auf 15 mA gesenkt. Normalbetrieb 
//              nimmt 60 mA. Ein LoRa Puls bei SF7 dauert 60 ms (eigentlich 41).
//              Duty Cycle ist bei 1x in 10 Sekunden etwa 0.6 %. Also legal.
//              Damit wären das etwa 500 sec usage. Später mit 1x in 3 min, dann 28,8 sec.
//              TTN fair use policy: max 30 sec am Tag. 
//              Senden von 10 Sec auf 180 sec erhöht -> DC = 0,033%. Stimmt auch ganz gut
//              mit Messung überein.
// 03.04.2021   Mit LoRa.sleep() Stromverbrauch von 15mA auf 5,9mA gesenkt. Damit müsste mein Sensor
//              ca. 3000h/5,9 = 500h = 21 Tage laufen. Das wäre ganz OK.
//              LiIon Laderegler TP4056 eingebaut für Solarversion.
//              Solarbetrieb: 100 cm2 Solarmodul müsste bei 20% des Maximalen Ertrags über den Winter reichen.
//              Nach LOS Berechnung und Messung mit AD8... Leistungsmesser: Board TX Power < 0dBm, ist 
//              recht wenig, reicht aber für meine Zwecke.
//              Tests mit LoRa.setTxPower(1 .. 17) ändert nix an der Sendeleistung. Richtige Library?
//              Richtig Output Parameter?
//              
// Erkenntisse: Bei Deep Sleep muss man sehr auf die Reihenfolge der Befehle achten,
//              da praktisch immer neu gestartet wird !!!!!
//              Der Stromverbrauch im Deep Sleep Modus ist viel zu hoch!!
//              Ich habe 15 mA / 6 mA mit allen Optionen im Deep Sleep Mode gemessen.
//              Wenn er mit Batterie startet, dann stehen wieder 12.8 mA im Deep Sleep Mode ??? !!!
//              Angeblich sollte man aber bis in den microAmpere Bereich kommen, den muss ich haben..
//              Wahrscheinlich liegt es einfach am ESP32 Board, das diese Sleep Modi nicht richtig
//              unterstützt. -> Board Vorschläge sind: Firebeetle, DFrobot 
//

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  180     /* Time ESP32 will go to sleep (in seconds) */

// RTC_DATA_ATTR int bootCount = 0;

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

/*
//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
*/

//Libraries for BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6

/*
//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
*/

//BME280 definition
#define SDA 21
#define SCL 13

TwoWire I2Cone = TwoWire(1);
Adafruit_BME280 bme;

// **** Sensor ID !!!!!!
int Sensor_ID = 2 ;   // möglich ist 2 oder 3

//packet counter
int readingID = 0;

int counter = 0;
String LoRaMessage = "";

float temperature = 0;
float humidity = 0;
float pressure = 0;

float height = 421 ;   // Standorthöhe über dem Meer, für die Höhenanpassung des Drucks

const int voltpin = 39 ;   // Analog PIN
int voltValue = 0;
float voltage = 0; 
int AkkuIntValue = 0;

/*
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//Initialize OLED display
void startOLED(){
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA SENDER");
}
*/

//Initialize LoRa module
void startLoRA(){
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

 // LoRa.setTxPower(14) ;   //  1 -> bestes RSSI, -> 17 schlechtestes RSSI bei Messung am 3.4.2021


  while (!LoRa.begin(BAND) && counter < 10) {
    Serial.print(".");
    counter++;
   // delay(500);
  // LoRa.setTxPower(17) ; 
    delay(10) ;
  }
  if (counter == 10) {
    // Increment readingID on every new reading
    readingID++;
    Serial.println("Starting LoRa failed!"); 
  }

  LoRa.setSyncWord(0xF3) ;
  Serial.println("LoRa Initialization OK!");
  
 /*
  display.setCursor(0,10);
  display.clearDisplay();
  display.print("LoRa Initializing OK!");
  display.display();
*/
  // delay(2000);
  delay(10) ;
}

void startBME(){
  I2Cone.begin(SDA, SCL, 100000); 
  bool status1 = bme.begin(0x76, &I2Cone);  
  if (!status1) {
    Serial.println("Could not find a valid BME280_1 sensor, check wiring!");
    while (1);
  }
}

void getReadings(){
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  
// **** Höhenanpassung ***********************************

  pressure = pressure + ( height/8.39 );
  
}

void sendReadings() {

  readingID = Sensor_ID * 1000 + AkkuIntValue + 1;   // Überttragung SensorID und Akkustand
  
  LoRaMessage = String(readingID) + "/" + String(temperature) + "&" + String(humidity) + "#" + String(pressure);
  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print(LoRaMessage);
  LoRa.endPacket();
/*
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.print("LoRa packet sent!");
  display.setCursor(0,20);
  display.print("Temperature:");
  display.setCursor(72,20);
  display.print(temperature);
  display.setCursor(0,30);
  display.print("Humidity:");
  display.setCursor(54,30);
  display.print(humidity);
  display.setCursor(0,40);
  display.print("Pressure:");
  display.setCursor(54,40);
  display.print(pressure);
  display.setCursor(0,50);
  display.print("Reading ID:");
  display.setCursor(66,50);
  display.print(readingID);
  display.display();
*/
 // Serial.print("Sending packet: ");
 // Serial.println(readingID);  
 
 // readingID++;
 // readingID = 2345 ;
 //readingID = Sensor_ID * 1000 + AkkuIntValue + 5;   // Überttragung SensorID und Akkustand

// ************** Serielle Ausgabe ***************************
  Serial.println("LoRa packet sent!");

  Serial.print("Temperatur:   ");
  Serial.println(temperature);
  
  Serial.print("Luftfeuchte:  ");
  Serial.println(humidity);
  
  Serial.print("Luftdruck:    ");
  Serial.println(pressure);

  Serial.print("Paket Nr:     ");
  Serial.println(readingID);
  
  Serial.print("Akku Spannung ");
  Serial.println(voltage);
  
  Serial.print("Akku Integer Value ");
  Serial.println(AkkuIntValue);
 
  Serial.println();
}

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  
 // startOLED();
  startBME();
  startLoRA();
  delay(100) ;
 // LoRa.setTxPower(17) ; 
// ***** muss ins Set-up da Deep Sleep Modus
  getReadings();
  voltValue = analogRead(voltpin) ;   // Analog PIN
 
  voltage = 0.7 * 0.00066534 * 3.92 * voltValue  ;  
  AkkuIntValue = 0.7 * 0.066534 * 3.92 * voltValue;
  
  sendReadings();
 // delay(100) ;
  delay(50) ;

// LoRa.dumpRegisters(Serial); // hilft auch nicht viel weiter

// ***** Sleep Modus einschalten *********************************
esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

// ***** Configuration des Sleep Modus ***************************

    LoRa.sleep() ;   // reduziert Stromverbrauch um ca. 7mA
 
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);

esp_deep_sleep_start();

}
void loop() {
/*  getReadings();
  sendReadings();
  voltValue = analogRead(voltpin) ;   // Analog PIN
  voltage = voltValue * 1.385 * 2* 3.3/4094 ; 
  AkkuIntValue = 0.22343 * voltValue ;
 */ 
 // delay(10000);
}
