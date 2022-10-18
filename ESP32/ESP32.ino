
#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// define the number of bytes you want to access
#define EEPROM_SIZE 4
unsigned int pictureNumber = 1;
String CMD_CAPTURE_CAMERA = "CMD:CAPTURE_CAMERA";
String CMD_DATA = "CMD:DATA:";
//#define FLASH_GPIO_NUM 4

void setup() {
  Serial.begin(9600);
  Serial.write("[ESP32] Setup start -----");
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.write("[ESP32] Camera init failed with error");
    Serial.write(err);

    return;
  }

  Serial.println("Starting SD Card");
  if (!SD_MMC.begin()) {
    Serial.write("[ESP32] SD Card Mount Failed");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.write("[ESP32] No SD Card attached");
    return;
  }
  Serial.write("[ESP32] Setup complete -----");
  //pinMode(FLASH_GPIO_NUM, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  String cmd ;
  cmd = Serial.readString();

  if (cmd.length() > 0) {
    Serial.write("[ESP32] Command received : ");
    Serial.write(cmd.c_str());

    if (cmd.startsWith(CMD_CAPTURE_CAMERA)) {
      //Serial.write("[ESP32] inside CMD:CAPTURE_CAMERA ");

      capturePhoto();
    }
    if (cmd.startsWith(CMD_DATA)) {
      Serial.write("[ESP32] inside CMD:DATA ");
      String mId = getValue(cmd, ':', 2);
      String mData = getValue(cmd, ':', 3);
      writeDataToSDCard(mId + ":" + mData);
    }

    // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
    /*pinMode(4, OUTPUT);
      digitalWrite(4, LOW);
      rtc_gpio_hold_en(GPIO_NUM_4); */
    /*String path = "/message-" + String(pictureNumber) + ".txt";

      fs::FS &fs = SD_MMC;

      File file = fs.open(path.c_str(), FILE_WRITE);
      if (!file) {
      //Serial.println("Failed to open file in writing mode");
      }
      else {
      file.println(cmd);
      file.println(String(cmd.length()));
      EEPROM.write(0, pictureNumber);
      EEPROM.commit();
      }
      file.close();*/


  }
}

void writeDataToSDCard (String message) {
  Serial.write("[ESP32] writeDataToSDCard : ");

  Serial.write(message.c_str());
  String path = "/cubeset-data.txt";

  fs::FS &fs = SD_MMC;

  File file = fs.open(path.c_str(), FILE_APPEND);
  if (!file) {
    Serial.write("[ESP32] Failed to open file in writing mode");
    return;
  }
  else {
    file.println(message.c_str());
    Serial.write("[ESP32] Data written to file successfully");
  }
  file.close();
}


void capturePhoto() {
  
  camera_fb_t * fb = NULL;

  // Take Picture with Camera
  fb = esp_camera_fb_get();
  if (!fb) {
    //Serial.println("Camera capture failed");
    return;
  }
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;

  String path = "/p-" + String(pictureNumber) + ".jpg";
  fs::FS &fs = SD_MMC;
  Serial.printf("Picture file name: %s\n", path.c_str());

  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    //Serial.println("Failed to open file in writing mode");
  }
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    //Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();
  esp_camera_fb_return(fb);
  Serial.write("Camera capture successful");
}
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
