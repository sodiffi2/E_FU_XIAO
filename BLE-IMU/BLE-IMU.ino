
#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <time.h>
#include <string>
#include <ctime>
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A
float aX, aY, aZ, gX, gY, gZ;
const float accelerationThreshold = 3;  // threshold of significant in G's
// const int numSamples = 100;
int count = 0;
String s;
float _checkAddNum = 0;
// int samplesRead = numSamples;
int times = 30;
double d, e, f;
double start, end, diff;


BLEService bleService("19B20000-E8F2-537E-4F6C-");  // Bluetooth® Low Energy Service

// Bluetooth® Low Energy Characteristic
BLEStringCharacteristic switchChar("ff00", BLERead | BLEWrite,20);
BLEStringCharacteristic numberChar("ff01", BLERead | BLENotify, 20);
BLEStringCharacteristic recordChar("ff02", BLERead | BLENotify, 72);
BLEStringCharacteristic datrChar("ff03", BLERead | BLENotify, 65535000);




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
  bleService.addCharacteristic(datrChar);

  // add service
  BLE.addService(bleService);

  // set the initial value for the characeristic:
  switchChar.writeValue("0");
  // switchChar.broadcast();
  numberChar.writeValue("0");
  recordChar.writeValue("");
  BLE.setAdvertisedService(bleService);
  BLE.setLocalName("e-fu .112");
  // start advertising
  BLE.advertise();
}

void loop() {
 
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
          start = clock();
          diff = 0;
          // while (samplesRead == numSamples) {
          //   // read the acceleration data
          //   aX = myIMU.readFloatAccelX();
          //   aY = myIMU.readFloatAccelY();
          //   aZ = myIMU.readFloatAccelZ();

          //   // sum up the absolutes
          //   float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

          //   // check if it's above the threshold
          //   if (aSum >= accelerationThreshold) {
          //     // reset the sample read count
          //     samplesRead = 0;
          //     break;
          //   }
          // }
          string parameter = "手";
          while (diff <= times) {
            float a = myIMU.readFloatAccelX();
            float b = myIMU.readFloatAccelY();
            float c = myIMU.readFloatAccelZ();
            float axis_X = myIMU.readFloatGyroX();
            float axis_Y = myIMU.readFloatGyroY();
            float axis_Z = myIMU.readFloatGyroZ();

            float pitch = (180 * atan2(a, sqrt(b * b + c * c)) / PI);
            float roll = (180 * atan2(b, sqrt(a * a + c * c)) / PI);
            bool isMinAngle = false, isMaxAngle = false;
            // _displayAngle = pitch;
            //腳部
            //pitch +=90;
            //isMinAngle = pitch < 30;
            //isMaxAngle = pitch > 77;
            //手部
            //isMinAngle = pitch < -5;
            //isMaxAngle = pitch > 60;

            if (parameter == "手") {
                isMinAngle = pitch < -5;
                isMaxAngle = pitch > 60;
            } else {
                pitch += 90;
                isMinAngle = pitch < 30;
                isMaxAngle = pitch > 77;
              }
              
            if (_checkAddNum == 0 && isMinAngle) _checkAddNum += .5;

            if (_checkAddNum == 0.5 && isMaxAngle) _checkAddNum += .5;

            if (_checkAddNum == 1) {
              count += 1;
              _checkAddNum = 0.0;
            }
            Serial.print(pitch, 3);
            Serial.print(',');
            Serial.print(roll, 3);
            Serial.print(',');
            Serial.print(count);
            Serial.println();

            Serial.println();

            end = clock();
            diff = (end - start) / CLOCKS_PER_SEC;
            Serial.println(diff);
            updateIMU(a, b, c, axis_X, axis_Y, axis_Z, pitch,count);

           
          }
          if (diff >= times) {
            s=(String(millis())+ "','"+String(count));            
            Serial.println(String(millis())+ ","+String(count));  
            datrChar.writeValue(s);
            count=0;
            // diff=0;
            
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

void updateIMU(float ax, float ay, float az, float gx, float gy, float gz, float pitch,int c) {
  // s += "[" + String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," + String(pitch) + "],";
  recordChar.writeValue(String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz)+ "," + String(pitch)+ "," + String(c));
}
