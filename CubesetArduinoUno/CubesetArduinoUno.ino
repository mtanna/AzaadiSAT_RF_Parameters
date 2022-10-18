/*
  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/
// FOR UNO
//    MOSI  -   11
//    MISO  -   12
//    SCK   -   13

#include <SD.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
const unsigned int csPin = 10;                                                                                                   // LoRa radio chip select
const unsigned int resetPin = 9;       // LoRa radio reset
const unsigned int irqPin = 8;         // change for your board; must be a hardware interrupt pin

//String outgoing;              // outgoing message

unsigned long lastReceiveTime = 0;
unsigned int receiveInterval = 200;
unsigned int lastReceivedMessageId = 0;

const unsigned int SDCardPin = 7;

//for battery percent code start

//for battery percent end
void setup() {
  Serial.begin(115200);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Uno : ");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(433E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  LoRa.setSyncWord(0xF1);
  LoRa.setTxPower(10);
  //LoRa.setSpreadingFactor(33);
  //LoRa.setSignalBandwidth(62.5E3);

  Serial.println("LoRa init succeeded.");
  if (!SD.begin(SDCardPin)) {
    Serial.println("SD Card initialization failed!");
    while (1);
  }
  Serial.println("SD Card ok!");

}

void loop() {

  int cMillis = millis();
  int diff = cMillis - lastReceiveTime ;

  if (diff > receiveInterval) {
    onReceive(LoRa.parsePacket());
    lastReceiveTime = cMillis;



    /*String inputmsg = "[MEGA]:243:21:UR=-0.14&TEMP=29.60&HU=76.90&HI=35.68&COLOR=253_249_236&BMPTEMP=30.08&BMPPRE=100867.21&BMPALT=2.74&LAT=**********&LONG=***********&ALT=******&UI=28&AX=0.01&AY=0.00&AZ=1.01&G=1.01&GX=0.44&GY=0.15&GZ=90.00&MPUTEMP=32.65&BT=0.00%";

      int found = 0;
      int lastFoundIndex = 0;
      int maxIndex = inputmsg.length() - 1;
      String outputData[4];

      for (int i = 0; i <= maxIndex; i++) {
      if (inputmsg.charAt(i) == ':' || i == maxIndex) {

        outputData[found] = inputmsg.substring(lastFoundIndex, (i == maxIndex) ? i + 1 : i);
        found++;
        lastFoundIndex = i;
        //strIndex[0] = strIndex[1] + 1;
        //strIndex[1] = (i == maxIndex) ? i + 1 : i;
      }
      }

      for (int i = 0; i < found; i++) {
      Serial.print(i);
      Serial.print (" : ");
      Serial.println(outputData[i]);
      }*/
  }


}

void onReceive(int packetSize) {
  if (packetSize == 0) return; // if there's no packet, return
  String incomingMessage = "";
  while (LoRa.available()) {
    incomingMessage += (char)LoRa.read();
  }

  Serial.print("Received Message : ");
  Serial.println(incomingMessage);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  

  
  int found = 0;
  int lastFoundIndex = 0;
  int maxIndex = incomingMessage.length() - 1;
  String outputData[3];

  for (int i = 0; i < maxIndex || found < 3; i++) {
    if (incomingMessage.charAt(i) == ':' || i == maxIndex) {

      outputData[found] = incomingMessage.substring(lastFoundIndex, (i == maxIndex) ? i + 1 : i);
      found++;
      lastFoundIndex = i + 1;
      //strIndex[0] = strIndex[1] + 1;
      //strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
 
  for (int i = 0; i < found; i++) {
    Serial.print(i);
    Serial.print (" : ");
    Serial.println(outputData[i]);
  }
  if (found >= 3) {

    //String tempString = getValue(incomingMessage, ':', 0);
    Serial.print("Sender : "  );
    Serial.println(outputData[0]);
    //if (!incomingMessage.startsWith("[MEGA]")) {
    if (outputData[0] != "[MEGA]") {
      Serial.print("[LoRa Uno OnReceive] [Error] : Not from [MEGA] : ");
      incomingMessage = "";
      return;
    }
    unsigned int messageLeangth = outputData[1].toInt();
    // Serial.print("Message length in Data : ");
    // Serial.println(messageLeangth);
    // Serial.print("incoming message length : ");
    // Serial.println(incomingMessage.length());
    if (messageLeangth != incomingMessage.length() - 1) {
      Serial.println("[LoRa Uno OnReceive] [Error] : Message length mismatch");
      incomingMessage = "";
      return;
    }
    //incomingMessage = incomingMessage.substring(0, messageLeangth);
    //String receivedMId = getValue(incomingMessage, ':', 2);

    //Serial.println("getValue Message Id" + receivedMId);

    unsigned long receivedMessageId =  atol(outputData[2].c_str());
    //Serial.println("Received Message Id : " + String(receivedMessageId));
    if (receivedMessageId <= lastReceivedMessageId) {

      Serial.print("[LoRa Uno OnReceive] [Error] : Old message.");
      Serial.println(incomingMessage);
      incomingMessage = "";
      return;
    }

    Serial.println(">>>>>>>>> Incoming message");
    

    writeMessageToSDCard(incomingMessage);
    lastReceivedMessageId = receivedMessageId;

  } else {
    Serial.println("incomplete message received");
  }

  incomingMessage = "";
}
void writeMessageToSDCard( String &message) {

  Serial.print("writeMessageToSDCard : ");
  Serial.println(message);

  File myFile = SD.open("data.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to SD card...");
    myFile.println(message);
    //myFile.print("\r\n");
    // close the file:
    myFile.close();
    Serial.println("File write done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening UnoDatat.txt");
  }
}
