

#include <ArduinoBLE.h>
// #include <MqttSerial.h>
#include <LSM6DS3.h>
#include <Wire.h>
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A
float aX, aY, aZ, gX, gY, gZ;
const float accelerationThreshold = 2.5;  // threshold of significant in G's
const int numSamples = 10;
int samplesRead = numSamples;

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214");  // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = LED_BUILTIN;  // pin to use for the LED

void setup() {
  Serial.begin(9600);
  

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1)
      ;
  }
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("LED");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");
  // mqttSerial.begin("quickstart.messaging.internetofthings.ibmcloud.com", "1883", "", "");  
  // mqttSerial.connect();
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
        Serial.println(switchCharacteristic.value());
        if (switchCharacteristic.value()==48) {  // any value other than 0
          Serial.println("start !!");
           
          while (samplesRead == numSamples) {
            // read the acceleration data
            aX = myIMU.readFloatAccelX();
            aY = myIMU.readFloatAccelY();
            aZ = myIMU.readFloatAccelZ();

            // sum up the absolutes
            float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

            // check if it's above the threshold
            if (aSum >= accelerationThreshold) {
              // reset the sample read count
              samplesRead = 0;
              break;
            }
          }

          // check if the all the required samples have been read since
          // the last time the significant motion was detected
          while (samplesRead < numSamples) {
            // check if both new acceleration and gyroscope data is
            // available
            // read the acceleration and gyroscope data

            samplesRead++;

            // print the data in CSV format
            Serial.print(myIMU.readFloatAccelX(), 3);
            Serial.print(',');
            Serial.print(myIMU.readFloatAccelY(), 3);
            Serial.print(',');
            Serial.print(myIMU.readFloatAccelZ(), 3);
            Serial.print(',');
            Serial.print(myIMU.readFloatGyroX(), 3);
            Serial.print(',');
            Serial.print(myIMU.readFloatGyroY(), 3);
            Serial.print(',');
            Serial.print(myIMU.readFloatGyroZ(), 3);
            Serial.println();
            // String pub = "{\"d\": {\"Temp\": \""+ 3 +"\"}}";
            // mqttSerial.publish("iot-2/evt/status/fmt/json", pub);

            if (samplesRead == numSamples) {
              // add an empty line if it's the last sample
              Serial.println();
            }
          }       // will turn the LED on
        } else {  // a 0 value
          Serial.println("end !!");
          //   Serial.println(F("LED off"));
          // digitalWrite(ledPin, LOW);  // will turn the LED off
        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
