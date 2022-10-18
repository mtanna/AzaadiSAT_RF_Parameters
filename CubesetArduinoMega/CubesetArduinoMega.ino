#include <Wire.h>
// code tcs34725 start - color sensor
#include "DEV_Config.h"
#include "TCS34725.h"
RGB rgb, RGB888;
UWORD   RGB565 = 0;
// code TCS34725 end
//code BMP280 start - pressure - temprature - altitude sensor
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp;
//code BMP280 end
// Code for DHT22 start - temprature & humidity sensor
#include "DHT.h"
#define DHTPIN 7
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
// Code for DHT22 end
// Code for ML8511 start
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board

int UV_INDEX_PIN = A2;
// Code for ML8511 end
//code for GPS GY-NEO6MV2 start
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int RXPin = 11, TXPin = 10;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpsSoftwareSerial(TXPin, RXPin );
//code for GPS GY-NEO6MV2 end

//code for ESP32-CAM start

//VOUT - 8
//VNR - 9
#include "string.h"
SoftwareSerial esp32Serial(8, 9);
const int esp32CamCaptureInterval = 4600;
unsigned long esp32LastCamCapturedTime = 0;
const int esp32DataSendInterval = 10400;
unsigned long esp32LastDataSentTime = 0;
unsigned int esp32MessageId = 1;
unsigned int esp32PictureId = 1;
//code for ESP32-CAM end

//code for LoRa start
#include <SPI.h>              // include libraries
#include <LoRa.h>

const int loRaCSPin = 53;          // LoRa radio chip select
const int loRaResetPin = 4;       // LoRa radio reset
const int loRaIrqPin = 3;         // change for your board; must be a hardware interrupt pin

//String outgoing;              // outgoing message

unsigned long loRaLastSendMessageId = 1;           // count of outgoing messages
unsigned long loRalastSendTime = 0;        // last send time
unsigned int loRaSendInterval = 10000;          // interval between sends
//code for LoRa end

//code for SIM800L start
#include "SIM800L.h"

//SIM800l RX -> 18
//SIM800l TX -> 19
#define SIM800_RST_PIN 12

SIM800L* sim800l;
String sim800lSMSPhoneNo = "9601763605";
String sim800l_SEND_SMS = "SIM800L_SEND_SMS";
//code for sIM800L end
//for battery percent code start

int batteryVoltagePin = A4;
//for battery percent end

//for MPU9250 code start
#define MPU6500_ADDR 0x68


int16_t AcX, AcY, AcZ, mpu9250_Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false; //stores if a fall has occurred
boolean trigger1 = false; //stores if first trigger (lower threshold) has occurred
boolean trigger2 = false; //stores if second trigger (upper threshold) has occurred
boolean trigger3 = false; //stores if third trigger (orientation change) has occurred
byte trigger1count = 0; //stores the counts past since trigger 1 was set true
byte trigger2count = 0; //stores the counts past since trigger 2 was set true
byte trigger3count = 0; //stores the counts past since trigger 3 was set true
int angleChange = 0;
#define FALL_LED_PIN 13
//for MPU9250 code end




unsigned long mainLastLoopTime = 0;        // last send time
unsigned int mainLoopInterval = 4000;

String ml8511UvRadiation = "",
       dht22Temperature = "",
       dht22Humidity = "",
       dht22HeatIndex = "",
       tcs34725Color = "",
       bmp280Temperature = "",
       bmp280Pressure = "",
       bmp280Altitude = "",
       gyneo6MV2Latitude = "",
       gyneo6MV2Longitude = "",
       gyneo6MV2Altitude = "",
       hw837UvIndex = "",
       batteryLevel = "",
       mpu9250Ax = "",
       mpu9250Ay = "",
       mpu9250Az = "",
       mpu9250Gx = "",
       mpu9250Gy = "",
       mpu9250Gz = "",
       mpu9250Temp = "";
void setup() {
  Serial.begin(115200);
  //pinMode(14, OUTPUT); //define a digital pin as output
  //digitalWrite(14, HIGH);// set the above pin as HIGH so it acts as 5V
  //pinMode(15, OUTPUT); //define a digital pin as output
  //digitalWrite(15, HIGH);// set the above pin as HIGH so it acts as 5V
  //pinMode(19, OUTPUT); //define a digital pin as output
  //digitalWrite(19, HIGH);// set the above pin as HIGH so it acts as 5V
  Serial.println("Main setup start ----");
  Wire.begin();
  setupML8511();
  setupLoRa();
  setupDHT22();
  setupTCS34725();
  setupBMP280();
  setupHW837();
  setupGYNEO6MV2();
  setupMPU9250();
  setupSIM800L();
  setupESP32();
  setupBatteryVoltageReading();
  delay(2500);
  Serial.println("Main setup complete -----");
}
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readString();
    Serial.print("Cmd received : ");
    Serial.println(cmd);
    if (cmd.startsWith(sim800l_SEND_SMS)) {
      prepareSim800LSMS();
    } else {
      Serial.println("Sending command to SIM800L");
      Serial1.println(cmd);
    }
  }
  if (Serial1.available())
  {
    Serial.println(Serial1.readString());
  }

  int mainCMillis = millis();
  int mainDiff = mainCMillis - mainLastLoopTime ;
  loopESP32CAM();
  loopLoRa();
  loopGYNEO6MV2();
  if (mainDiff > mainLoopInterval) {
    Serial.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
    loopML8511();
    loopDHT22();
    loopBMP280();
    loopTCS34725();
    loopHW837();
    loopMPU9250();
    calculateBatteryLevel();
    mainLastLoopTime = mainCMillis;
    Serial.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
  }

}
void setupBatteryVoltageReading() {
  pinMode(batteryVoltagePin, INPUT);
  Serial.println("Battery voltage reading setup complete.");
}
void setupMPU9250() {
  pinMode(FALL_LED_PIN, OUTPUT);
  digitalWrite(FALL_LED_PIN, LOW);
  Serial.println("MPU9250 setup complete.");
}
void calculateBatteryLevel() {

  int a0Value = averageAnalogRead(batteryVoltagePin, 64);
  float voltage = (a0Value * 5.0 / 1023) / 0.2;
  Serial.print("Battery voltage : ");
  Serial.println(voltage);
  batteryLevel = String(voltage);//+"-voltage-"+String(voltage);
  /*float perc = mapfloat(voltage, 3.1, 3.7, 0, 100);
    if (perc >= 100.0)
    { perc = 100.0;
    }
    if (perc <= 0.0)
    {
    perc = 0.0;
    }
    batteryLevel = String(perc);//+"-voltage-"+String(voltage);
    Serial.print("Battery = ");
    Serial.println(batteryLevel);*/

  /* sensorValue = analogRead(analogInPin);
    voltage = (((sensorValue * 3.3) / 1024) * 2 + calibration); //multiply by two as voltage divider network is 100K & 100K Resistor

    bat_percentage = mapfloat(voltage, 2.8, 4.2, 0, 100); //2.8V as Battery Cut off Voltage & 4.2V as Maximum Voltage

    if (bat_percentage >= 100)
    {
     bat_percentage = 100;
    }
    if (bat_percentage <= 0)
    {
     bat_percentage = 1;
    }
    Serial.print("Analog Value = ");
    Serial.print(sensorValue);
    Serial.print("\t Output Voltage = ");
    Serial.print(voltage);
    Serial.print("\t Battery Percentage = ");
    Serial.println(bat_percentage);
    batteryLevel = String(bat_percentage);*/
}
void setupLoRa() {
  //Serial.println("LoRa setup start.");
  LoRa.setPins(loRaCSPin, loRaResetPin, loRaIrqPin);// set CS, reset, IRQ pin
  if (!LoRa.begin(433E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  LoRa.setSyncWord(0xF1);
  LoRa.setTxPower(10);
  Serial.println("LoRa setup complete.");
}
void setupBMP280() {
  //Serial.println("BMP280 setup start.");
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56 - 0x58 represents a BMP 280, \n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    //while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  Serial.println("BMP280 setup complete.");
}
void setupTCS34725() {
  //Serial.println("TCS34725 setup start.");
  Config_Init();
  if (TCS34725_Init() != 0) {
    Serial.print("TCS34725 initialization error!!\r\n");
    return;
  }
  //digitalWrite(PWM_PIN, HIGH);
  Serial.println("TCS34725 setup complete.");
}
void setupDHT22() {
  //Serial.println("DHT22 setup start.");
  dht.begin();
  Serial.println("DHT22 setup complete.");
}
void setupHW837() {
  pinMode(UV_INDEX_PIN, INPUT);
  Serial.println("HW837 setup complete.");
}
void setupGYNEO6MV2() {
  //Serial.println("GPS setup start.");
  gpsSoftwareSerial.begin(GPSBaud);
  Serial.println("GPS setup complete.");
}
void setupESP32() {
  //Serial.println("ESP32 setup start.");
  esp32Serial.begin(9600);
  Serial.println("ESP32 setup complete.");
}
void loopDHT22() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  //float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) ) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  //float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F(" %  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(F("  Heat index: "));
  Serial.print(hic);
  Serial.println(F("°C "));
  dht22Temperature = String(t);// + " °C";
  dht22Humidity = String(h);// + "%";
  dht22HeatIndex = String(hic);// + " °C";
  //Serial.print(hif);
  //Serial.println(F("°F"));
}

void setupML8511() {
  //Serial.println("ML8511 setup start.");
  pinMode(UVOUT, INPUT);
  //pinMode(REF_3V3, INPUT);
  Serial.println("ML8511 setup complete.");
}
void setupSIM800L() {

  //serialSIM800L = new SoftwareSerial(SIM800_RX_PIN, SIM800_TX_PIN);
  Serial.println("SIM800L setup start......");
  Serial1.begin(9600);
  sim800l = new SIM800L((Stream *)&Serial1, SIM800_RST_PIN, 200, 512);
  while (!sim800l->isReady()) {
    Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
    delay(1000);
  }
  Serial.println(F("AT command successful."));

  // Wait for the GSM signal
  uint8_t signal = sim800l->getSignal();
  while (signal <= 0) {
    delay(1000);
    signal = sim800l->getSignal();
  }

  Serial.print(F("Signal OK (strenght: "));
  Serial.print(signal);
  Serial.println(F(")"));
  delay(1000);
  /*
    Serial.print("Registering to network..");
    // Wait for operator network registration (national or roaming network)
    NetworkRegistration network = sim800l->getRegistrationStatus();
    while (network != REGISTERED_HOME && network != REGISTERED_ROAMING) {
      delay(1000);
      Serial.print(".");
      network = sim800l->getRegistrationStatus();
    }
    Serial.println(F("Network registration OK"));
    delay(1000);
  */
  sim800lSleepMode();
  Serial.println("SIM800L setup complete.");

}
void prepareSim800LSMS() {
  String msg = "LAT=" + gyneo6MV2Latitude + "&";
  msg += "LONG=" + gyneo6MV2Longitude + "&";
  msg += "ALT=" + gyneo6MV2Altitude + "&";
  msg += "BTR=" + batteryLevel + "V";
  sendSIM800LSMS(sim800lSMSPhoneNo, msg);
}
void loopMPU9250() {

  mpu9250_read();
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;

  // calculating Amplitute vactor for 3 axis
  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int Amp = Raw_Amp * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
  Serial.println(Amp);
  if (Amp <= 2 && trigger2 == false) { //if AM breaks lower threshold (0.4g)
    trigger1 = true;
    Serial.println("TRIGGER 1 ACTIVATED");
  }
  if (trigger1 == true) {
    trigger1count++;
    if (Amp >= 12) {  //12//if AM breaks upper threshold (3g)
      trigger2 = true;
      Serial.println("TRIGGER 2 ACTIVATED");

      trigger1 = false; trigger1count = 0;
    }
  }
  if (trigger2 == true) {
    trigger2count++;
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5); Serial.println(angleChange);
    if (angleChange >= 30 && angleChange <= 400) { //if orientation changes by between 80-100 degrees
      trigger3 = true; trigger2 = false; trigger2count = 0;
      Serial.println(angleChange);
      Serial.println("TRIGGER 3 ACTIVATED");
    }
  }
  if (trigger3 == true) {
    trigger3count++;
    if (trigger3count >= 10) {
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      //delay(10);
      Serial.println(angleChange);
      if ((angleChange >= 0) && (angleChange <= 10)) { //if orientation changes remains between 0-10 degrees

        fall = true; trigger3 = false; trigger3count = 0;
        Serial.println(angleChange);
      }
      else { //user regained normal orientation
        trigger3 = false; trigger3count = 0;
        Serial.println("TRIGGER 3 DEACTIVATED");
        digitalWrite(FALL_LED_PIN, LOW);
      }
    }
  }
  if (fall == true) { //in event of a fall detection
    Serial.println(">>>>>>>>>>>> FALL DETECTED");
    digitalWrite(FALL_LED_PIN, HIGH);
    fall = false;

  }
  if (trigger2count >= 6) { //allow 0.5s for orientation change
    trigger2 = false; trigger2count = 0;
    Serial.println("TRIGGER 2 DECACTIVATED");
    digitalWrite(FALL_LED_PIN, LOW);
  }
  if (trigger1count >= 6) { //allow 0.5s for AM to break upper threshold
    trigger1 = false; trigger1count = 0;
    Serial.println("TRIGGER 1 DECACTIVATED");
    digitalWrite(FALL_LED_PIN, LOW);

  }

  /*
    mpu9250AccelerationX= "",
       mpu9250AccelerationY= "",
       mpu9250AccelerationZ= "",
       mpu9250GyroX= "",
       mpu9250GyroY= "",
       mpu9250GyroZ= "",
       mpu9250Temp="";
  */


  mpu9250Ax = String(ax);
  mpu9250Ay = String(ay);
  mpu9250Az = String(az);

  mpu9250Gx = String(gx);
  mpu9250Gy = String(gy);
  mpu9250Gz = String(gz);

  mpu9250Temp = String(mpu9250_Tmp);
  Serial.print("AccX : ");
  Serial.print(mpu9250Ax);
  Serial.print("\tAccY : ");
  Serial.print(mpu9250Ay);
  Serial.print("\tAccZ : ");
  Serial.print(mpu9250Az);
  Serial.print("\tGyroX : ");
  Serial.print(mpu9250Gx);
  Serial.print("\tGyroY : ");
  Serial.print(mpu9250Gy);
  Serial.print("\tGyroZ : ");
  Serial.print(mpu9250Gz);
  Serial.print("\tTemp : ");
  Serial.println(mpu9250Temp);


}
void loopML8511() {
  //analogReference(3.3);
  int uvLevel = averageAnalogRead(UVOUT, 8);
  int refLevel = averageAnalogRead(REF_3V3, 8);


  //float refVoltage = refLevel * (5.0 /1023);

  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  //float outputVoltage = uvLevel * (3.10 /1023);
  float uvIntensity = mapfloat(outputVoltage  , 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level

  /*analogReference(3.3);
    int refLevel = analogRead(REF_3V3); //averageAnalogRead(REF_3V3);
    float refVoltage = float(refLevel) * (3.3 / 1023.0);
    Serial.print("ref level : ");
    Serial.println(refLevel);

    Serial.print("ref volt : ");
    Serial.println(refVoltage);

    analogReference(3.3);
    int uvLevel = averageAnalogRead(UVOUT);
    Serial.print("uv level : ");
    Serial.println(uvLevel);
    float op = float(uvLevel) * (3.3/ 1023.0);
    Serial.print("uv voltage : ");
    Serial.println(op);
    float uvIntensity = mapfloat(op ,0.99, 2.8,  0.0, 15.0); //Convert the voltage to a UV intensity level

  */
  /*

    float refLevel =float(averageAnalogRead(REF_3V3)) * (5.0 /1023);
    float uvLevel =float(averageAnalogRead(UVOUT)) / 1023.0 * refLevel;
    // float(averageAnalogRead(UVOUT)) * (5.0 / 1024.0);
      Serial.print("ML8511 ref voltage : ");
      Serial.println(refLevel);

    Serial.print("ML8511 output voltage : ");
    Serial.println(uvLevel );
    float uvIntensity = mapfloat(uvLevel ,0.99, 2.8,  0.0, 15.0); //Convert the voltage to a UV intensity level
  */



  /*
    Serial.print("refLevel: ");
    Serial.print(refLevel);

    Serial.print("  uvLevel: ");
    Serial.print(uvLevel);

    Serial.print(" / ML8511 voltage: ");
    Serial.print(outputVoltage);
  */
  /* ml8511UvRadiation = "";
    ml8511UvRadiation += "refLevel-";
    ml8511UvRadiation += String(refLevel);
    ml8511UvRadiation +=  "-uvLevel-";
    ml8511UvRadiation +=  String(uvLevel);
    ml8511UvRadiation += "-opVolat-";
    ml8511UvRadiation += String(outputVoltage);
    ml8511UvRadiation += "-uvI-";

    ml8511UvRadiation += String(uvIntensity);
  */
  Serial.print("UV output voltate : ");
  Serial.println(outputVoltage);
  ml8511UvRadiation = String(uvIntensity);// + "UV-VOLT=" + String(outputVoltage); // + " (mW / cm ^ 2)";
  Serial.print("UV Intensity (mW / cm ^ 2): ");
  Serial.println(ml8511UvRadiation);

}
void loopBMP280() {
  bmp280Temperature = String(bmp.readTemperature());// + " °C";
  bmp280Pressure = String(bmp.readPressure());// + "Pa";
  bmp280Altitude = String(bmp.readAltitude(1009));// + " m";
  Serial.print(F("Temperature = "));
  Serial.print(bmp280Temperature);


  Serial.print(F("  Pressure = "));
  Serial.print(bmp280Pressure);
  Serial.print(" Pa");

  Serial.print(F("  Approx altitude = "));
  Serial.print(bmp280Altitude); /* Adjusted to local forecast! */
  //The "1019.66" is the pressure(hPa) at sea level in day in your region
  //If you don't know it, modify it until you get your current altitude
  Serial.println(" m");

}
void loopGYNEO6MV2() {

  Serial.print("Latitude : ");
  gyneo6MV2Latitude = printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  Serial.print(gyneo6MV2Latitude);
  Serial.print("  Longitude : ");
  gyneo6MV2Longitude = printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  Serial.print(gyneo6MV2Longitude);
  //printInt(gps.location.age(), gps.location.isValid(), 5);
  Serial.print("  Date Time : ");
  printDateTime(gps.date, gps.time);
  Serial.print("  Altitude : ");
  gyneo6MV2Altitude = printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  Serial.println(gyneo6MV2Altitude);

  //printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  //printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
  }
}
void loopTCS34725()
{
  rgb = TCS34725_Get_RGBData();
  RGB888 = TCS34725_GetRGB888(rgb);
  RGB565 = TCS34725_GetRGB565(rgb);
  Serial.print("RGB888 : R = ");
  Serial.print(RGB888.R);
  Serial.print(" G = ");
  Serial.print(RGB888.G);
  Serial.print(" B = ");
  Serial.println(RGB888.B);
  tcs34725Color = String(RGB888.R) + "_" + String(RGB888.G) + "_" + String(RGB888.B);
  //Serial.print("\r\n");
  //Serial.print("RGB565 = 0x");
  //Serial.println((RGB565), HEX);
  /*if (TCS34725_GetLux_Interrupt(0xff00, 0x00ff) == 1) {
    Serial.println("Lux_Interrupt = 1\r\n");
    } else {
    Serial.println("Lux_Interrupt = 0\r\n");
    }*/
}
void loopHW837() {
  float sensorVoltage;
  float sensorValue;
  int UV_index;
  String quality = "";
  //analogReference(5.0);
  sensorValue = averageAnalogRead(UV_INDEX_PIN, 8);

  sensorVoltage = sensorValue * (5.0 / 1023);
  Serial.print("uv sensor voltage: ");
  Serial.println(sensorVoltage);
  UV_index = sensorVoltage / 0.1;
  hw837UvIndex = String(UV_index);

  //condition for UV state
  if (UV_index <= 2) {
    quality = "   LOW ";
  }

  else if (UV_index > 2 && UV_index <= 5) {
    quality = "   MOD ";
  }

  else if (UV_index > 5 && UV_index <= 7) {
    quality = "   HIGH ";
  }

  else if (UV_index > 7 && UV_index <= 10) {
    quality = "VERY HIGH";

  }

  else {
    quality = " EXTREME ";
  }
  Serial.print("UV value : ");
  Serial.print(UV_index);
  Serial.print("  UV quality : ");
  Serial.println(quality);
}
void loopESP32CAM() {
  int cMillis = millis();

  int diff = cMillis - esp32LastDataSentTime;
  if (diff > esp32DataSendInterval) {
    Serial.println("[Arduino] : Send SEND_DATA_TO_ESP32 to ESP32 - CAM.");
    String msg = "T"+String(cMillis)+"&";
    msg +="UR=" + ml8511UvRadiation + "&";
    msg += "UI=" + hw837UvIndex + "&";
    msg += "TEMP=" + dht22Temperature + "&";
    msg += "HU=" + dht22Humidity + "&";
    msg += "HI=" + dht22HeatIndex + "&";
    msg += "COLOR=" + tcs34725Color + "&";
    msg += "BMPTEMP=" + bmp280Temperature + "&";
    msg += "BMPPRE=" + bmp280Pressure + "&";
    msg += "BMPALT=" + bmp280Altitude + "&";
    msg += "LAT=" + gyneo6MV2Latitude + "&";
    msg += "LONG=" + gyneo6MV2Longitude + "&";
    msg += "ALT=" + gyneo6MV2Altitude + "&";
    msg += "AX=" + mpu9250Ax + "&";
    msg += "AY=" + mpu9250Ay + "&";
    msg += "AZ=" + mpu9250Az + "&";
    msg += "GX=" + mpu9250Gx + "&";
    msg += "GY=" + mpu9250Gy + "&";
    msg += "GZ=" + mpu9250Gz + "&";
    msg += "MPUTEMP=" + mpu9250Temp + "&";
    msg += "BT=" + batteryLevel + "V";
    String esp32Data = "CMD:DATA:" + String(esp32MessageId) + ":" + msg;
    esp32Serial.write(esp32Data.c_str());
    //String esp32Data = String(esp32MessageId) + ":" + msg;
    esp32LastDataSentTime = cMillis;
    esp32MessageId++;
  } else {
    diff = cMillis - esp32LastCamCapturedTime ;
    if (diff > esp32CamCaptureInterval) {
      Serial.println("[Arduino] : Send CMD: CAPTURE_CAMERA to ESP32");
      esp32Serial.write("CMD:CAPTURE_CAMERA");
      esp32LastCamCapturedTime = cMillis;
    }
  }

  String esp32Response = "";
  esp32Response = esp32Serial.readString();
  if (esp32Response.length() > 0) {
    Serial.print("[Arduino] : ESP32 message received : ");
    Serial.println(esp32Response);
  }
}
int averageAnalogRead(int pinToRead, byte numberOfReadings)
{

  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++)
  {
    runningValue += analogRead(pinToRead);
  }
  runningValue /= numberOfReadings;

  return (runningValue);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static String printFloat(float val, bool valid, int len, int prec)
{
  String result = "";
  if (!valid)
  {
    while (len-- > 1) {
      Serial.print('*');
      result += '*';
    }
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    result = String(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');

  }

  smartDelay(0);
  return result;
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = " * * * * * * * * * * * * * * * * * ";
  if (valid)
    sprintf(sz, " % ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F(" * * * * * * * * * * "));
  }
  else
  {
    char sz[32];
    sprintf(sz, " % 02d / % 02d / % 02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F(" * * * * * * * * "));
  }
  else
  {
    char sz[32];
    sprintf(sz, " % 02d: % 02d: % 02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gpsSoftwareSerial.available())
      gps.encode(gpsSoftwareSerial.read());
  } while (millis() - start < ms);
}
void loopLoRa() {
  int cMillis = millis();
  int diff = cMillis - loRalastSendTime ;
  if (diff > loRaSendInterval) {
    Serial.println("Send start -------");
    loRaSendMessage();
    loRalastSendTime = cMillis;            // timestamp the message
    Serial.println("Send complete-------");

  }
}
void loRaSendMessage() {
  String preMsg =  "[MEGA]:";
  String msg =  ":" + String(loRaLastSendMessageId) + ":";
  /*
    ml8511UvRadiation = "",
       dht22Temperature = "",
       dht22Humidity = "",
       dht22HeatIndex = "",
       tcs34725Color="",
       bmp280Temperature = "",
       bmp280Pressure = "",
       bmp280Altitude = "",
       gyneo6MV2Latitude = "",
       gyneo6MV2Longitude = "",
       gyneo6MV2Altitude = "",
       hw837UvIndex ="";
  */
  msg += "UR=" + ml8511UvRadiation + "&";
  msg += "UI=" + hw837UvIndex + "&";
  msg += "TEMP=" + dht22Temperature + "&";
  msg += "HU=" + dht22Humidity + "&";
  msg += "HI=" + dht22HeatIndex + "&";
  msg += "COLOR=" + tcs34725Color + "&";
  msg += "BMPTEMP=" + bmp280Temperature + "&";
  msg += "BMPPRE=" + bmp280Pressure + "&";
  msg += "BMPALT=" + bmp280Altitude + "&";
  msg += "LAT=" + gyneo6MV2Latitude + "&";
  msg += "LONG=" + gyneo6MV2Longitude + "&";
  msg += "ALT=" + gyneo6MV2Altitude + "&";
  //msg += "AX=" + mpu9250AccelerationX + "&";
  //msg += "AY=" + mpu9250AccelerationY + "&";
  //msg += "AZ=" + mpu9250AccelerationZ + "&";
  msg += "AX=" + mpu9250Ax + "&";
  msg += "AY=" + mpu9250Ay + "&";
  msg += "AZ=" + mpu9250Az + "&";
  msg += "GX=" + mpu9250Gx + "&";
  msg += "GY=" + mpu9250Gy + "&";
  msg += "GZ=" + mpu9250Gz + "&";
  msg += "MPUTEMP=" + mpu9250Temp + "&";
  msg += "BT=" + batteryLevel + "V";
  int mLength = preMsg.length() + msg.length() ;
  mLength += String(mLength).length() ;
  msg = preMsg + String(mLength) + msg;
  char msgBytes[msg.length() + 1];
  msg.toCharArray(msgBytes, msg.length() + 1);
  Serial.println("Sending : " + msg);
  LoRa.beginPacket();                   // start packet
  //LoRa.write(loRaDestination);              // add destination address
  //LoRa.write(loRaLocalAddress);             // add sender address
  //LoRa.write(loRaLastSendMessageId);                 // add message ID
  //LoRa.write(msg.length() + 1);      // add payload length
  LoRa.write(msgBytes, msg.length() + 1);            // add payload
  LoRa.endPacket();                     // finish packet and send it
  loRaLastSendMessageId++;                           // increment message ID
  msg = "";
}

void sim800lSleepMode() {
  Serial.println("entering sleep mode");
  Serial1.print("AT+CSCLK=2\r");
}
void sim800lWakeup() {
  Serial.println("sim800l wakeup start");
  Serial1.print("AT\r");
  delay(4900);
  Serial1.print("AT+CSCLK=0\r");


  uint8_t signal = sim800l->getSignal();
  while (signal <= 0) {
    delay(1000);
    signal = sim800l->getSignal();
  }

  Serial.print(F("Signal OK (strenght: "));
  Serial.print(signal);
  Serial.println(F(")"));
  delay(200);
  Serial.println("sim800l wakeup complete");
}
void sendSIM800LSMS(String &mobileNoString, String &message)
{
  sim800lWakeup();
  Serial.println("Sending SMS...");               //Show this message on serial monitor
  Serial1.print("AT+CMGF=1\r");                   //Set the module to SMS mode
  delay(1000);
  String smsCmd = "AT+CMGS=\"+91" + mobileNoString + "\"\r";
  Serial1.print(smsCmd.c_str());  //Your phone number don't forget to include your country code, example +212123456789"
  delay(1000);
  Serial1.print(message.c_str());       //This is the text to send to the phone number, don't make it too long or you have to modify the SoftwareSerial buffer
  delay(1000);
  Serial1.print((char)26);// (required according to the datasheet)
  delay(1000);
  Serial1.println();
  Serial.println("SMS Sent.");
  delay(1000);
  sim800lSleepMode();
}
int getSIM800LNetwork()
{
  String buff;
  //buff = "+CSQ: 16,0";
  unsigned int result, index1, index2, timeout = 0;

  Serial1.println("AT+CSQ");
  for (unsigned long start = millis(); millis() - start < 2000;) {
    while (Serial1.available())
    {
      buff = Serial1.readString();
      timeout = 1;
      break;
    }
  }

  if (timeout == 0)
  {
    return 0;
  }

  Serial.println(buff);

  //String network_status;
  //_____________________________________________________
  //Remove sent "AT Command" from the response string.
  index1 = buff.indexOf("\r");
  buff.remove(0, index1 + 2);
  buff.trim();
  //_____________________________________________________

  //_____________________________________________________

  index1 = buff.indexOf(":");
  index2 = buff.indexOf(",");
  buff = buff.substring(index1 + 1, index2);
  buff.trim();
  result = buff.toInt();

  if (result == 99)
  {
    //not known or not detectable
    return 0;
  }
  else if (result >= 2 && result <= 9)
  {
    //Signal Quality = Marginal
    return 20;
  }
  else if (result >= 10 && result <= 14)
  {
    //Signal Quality = OK
    return 40;
  }
  else if (result >= 15 && result <= 19)
  {
    //Signal Quality = Good
    return 60;
  }
  else if (result >= 20 && result <= 31)
  {
    //Signal Quality = Excellent
    return 99;
  }

  return 0;
}
void mpu9250_read() {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  mpu9250_Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
