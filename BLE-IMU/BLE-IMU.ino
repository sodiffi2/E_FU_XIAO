

#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A
float aX, aY, aZ, gX, gY, gZ;
const float accelerationThreshold = 3;  // threshold of significant in G's
const int numSamples = 10000;
int count=0;

float _checkAddNum=0;
int samplesRead = numSamples;

double d, e, f;


BLEService bleService("19B20000-E8F2-537E-4F6C-D104768A1214");  // Bluetooth® Low Energy Service

// Bluetooth® Low Energy Characteristic
BLEBoolCharacteristic switchChar("ff00", BLERead | BLEWrite);
BLEStringCharacteristic numberChar("ff01", BLERead | BLENotify, 20);
BLEStringCharacteristic recordChar("ff02", BLERead | BLENotify, 72);



void setup() {
  Serial.begin(9600);
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }
  // set advertised local name and service UUID:

  // add the characteristic to the service
  bleService.addCharacteristic(switchChar);
  bleService.addCharacteristic(numberChar);
  bleService.addCharacteristic(recordChar);

  // add service
  BLE.addService(bleService);

  // set the initial value for the characeristic:
  switchChar.writeValue(0);
  switchChar.broadcast();
  numberChar.writeValue("0");
  recordChar.writeValue("");
  BLE.setAdvertisedService(bleService);
  BLE.setLocalName("e-fu .112");
  // start advertising
  BLE.advertise();
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
      if (switchChar.written()) {
        Serial.println("get value : ");
        Serial.println(switchChar.value());
        if (switchChar.value()) {
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


          while (samplesRead < numSamples) {
            samplesRead++;
            float a = myIMU.readFloatAccelX();
            float b = myIMU.readFloatAccelY();
            float c = myIMU.readFloatAccelZ();

            float axis_X = myIMU.readFloatGyroX();
            float axis_Y = myIMU.readFloatGyroY();
            float axis_Z = myIMU.readFloatGyroZ();
           
           
           
            float pitch=(180*atan2(a,sqrt(b*b+c*c))/PI);
            float roll=(180*atan2(b,sqrt(a*a+c*c))/PI);
             bool isMinAngle = false, isMaxAngle = false;
            // _displayAngle = pitch;
            isMinAngle = pitch < -80;
            isMaxAngle = pitch > 80;
            if (_checkAddNum == 0 && isMinAngle) _checkAddNum += .5;

            if (_checkAddNum == 0.5 && isMaxAngle) _checkAddNum += .5;

            if (_checkAddNum == 1) {
              count += 1;
              _checkAddNum = 0.0;
            }
            Serial.print(pitch,3);
            Serial.print(',');
            Serial.print(roll,3);
            Serial.print(',');
            Serial.print(count);
            Serial.println();
            
            Serial.println();

            updateIMU(a, b, c, axis_X, axis_Y, axis_Z);

            if (samplesRead == numSamples) {
              // add an empty line if it's the last sample
              Serial.println();
            }
          }
        } else {  // a 0 value
          Serial.println("end !!");
        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}

void updateIMU(float ax, float ay, float az, float gx, float gy, float gz) {

  recordChar.writeValue(String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz));
}
