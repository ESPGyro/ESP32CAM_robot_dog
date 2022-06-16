/*
  Quadruped robot arduino sketch.
  3/10/2020 by Alexandros Petkos
  Updates available at https://github.com/maestrakos/warp

  This kinematics sketch is placed under CC-BY.

  This file is part of warp_kinematics.

  [source] This is the main file that manages [kinematics] & [hardware]
  all the important parameters are set in this file.

  Comment Description:

  /// comment

  //> used to explain the function of a line
  //: used to summurize the function of multiple lines

  === used for headers
  ::: used for sketch parts

  // ## used to explain the measurement unit of a variable
  // !! used for warnings
*/
/*
   Modify the file and support V7RC by Mason (2021/05/22)   
 - Hardware :  ESP32CAM + Motor shield + PCA9685 
 
 */
#include "datatypes.h"

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define I2C_SDA 15  // 1
#define I2C_SCL 13  // 3

// MG90 Servo PWM Pulse Traveling
const int PWMRES_Min = 0;       // PWM Resolution 0
const int PWMRES_Max = 180;     // PWM Resolution 180
const int SERVOMIN = 500;       // 500
const int SERVOMAX = 2400;      // 2400

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include <EEPROM.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/rtc_io.h"

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
bool run_status = true; 

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

String receive_data = ""; 

// servo offset, pleaee update the offset values
//                      P2, P5,P13,P10, P4, P7,P12,P11,P17,P16,P3,P15
// int offset_12x[] = { 5, 6, -4, 12, 11, 2, -3, 5, 7, 1, -1, 4}; // default
// int offset_map[] = { 8, 1, 6, -6, 3, -5, 2, 10, 2, -6, 5, 0}; // default
///int offset_map[] = { 8, 0, 6, 5, 3, -5, 2, 10, 2, -6, 5, 0}; // default
/// aa int offset_map[] = { 8, -5, 6, 2, 3, -5, 2, 10, 6, -6, 5, 0}; // default
//// int offset_map[] = { -12, -4, 4, -13, 2, -4, 2, -6, -6, -12, -7, 0}; // default
int offset_map[] = { 3, -7, -3, -14, -2, -8, -13, 5, -1, -11, -8, 0}; // default
/// int offset_map[] = { 8, -5, 6, 2, 3, -5, 2, 10, 5, -6, 5, -5}; // default


/*
  ==============================
  IMPORTANT PARAMETERS
  ==============================
*/
//> stores the frequency of the loop function
float frequency = 500.0; // ## Hz  small -> fast 
//const float frequency = 440.0; // ## Hz

/// Kinematics Parameters

//: stores the location, rotation and scale of the main [body]
const datatypes::Transform body_transform = {
  {0, 0, 0},  // ## {mm, mm, mm}
  {0, 0, 0},   // ## {deg, deg, deg}
  {130, 40, 90} // ## {mm, mm, mm}
};
const datatypes::Transform body_transformorg = {
  {0, 0, 0},  // ## {mm, mm, mm}
  {0, 0, 0},   // ## {deg, deg, deg}
  {300, 40, 180} // ## {mm, mm, mm}
};

//: stores the parent joint location relative to the [body]
const datatypes::Vector p_joint_originaaa[] = {
  { -50, 0, 0}, // ## {mm, mm, mm}
  { +50, 0, 0}, // ## {mm, mm, mm}
  { +50, 0, 0}, // ## {mm, mm, mm}
  { -50, 0, 0}  // ## {mm, mm, mm}
};
const datatypes::Vector p_joint_origin[] = {
  { -0, 0, 0}, // ## {mm, mm, mm}
  { +0, 0, 0}, // ## {mm, mm, mm}
  { +0, 0, 0}, // ## {mm, mm, mm}
  { -0, 0, 0}  // ## {mm, mm, mm}
};
// const float bone_length = 105; // ## mm
const float bone_length = 37; // ## mm
float step_x =0.0; 

//: high level parameters for the step function
// const datatypes::Vector step_extent = {40, 40, 26}; // ## {mm, mm}
const datatypes::Vector step_extent = {step_x, 15, 10}; // ## {mm, mm}
// float vrt_offset = - 16.50; // ## mm
// float hrz_offset = - 6.00; // ## mm

float vrt_offset = 0.0; // ## mm  forward 
float hrz_offset = 0.00; // ## mm

float base_offset[] = { 0, -1, 0, -2};
const float precision = 0.001; // ## mm

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
         receive_data =""; 
        for (int i = 0; i < rxValue.length(); i++) {
           receive_data = receive_data + rxValue[i]; }
    //     Serial.println(receive_data);
   }
};

unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }  
  return decValue;
}

void setup() {
  
  Wire.begin(I2C_SDA,I2C_SCL);
  Wire.setClock(400000);

  pwm.begin();
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  
  Serial.begin(115200);

   // Create the BLE Device
  BLEDevice::init("");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  // https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/BLEAdvertising.cpp
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
 
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  BLEUUID uuid("6E400001");
  //Device Name
  String device_name = "ESP_dog";
  oAdvertisementData.setName(device_name.c_str());
  pAdvertising->setAdvertisementData(oAdvertisementData);
 
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); 
  pAdvertising->setMinPreferred(0x12);

   pAdvertising->start();
  Serial.println("Characteristic defined! Now you can read it in your phone!");

  init_input();
}

//: those local variables control the step direction and period
datatypes::Vector2D _direction = {0, 0};
float turn = 0; //> indicates the direction of rotation
float height = 0; //> indicates the leg extension

// int state = 0; //> indicates the type of gait, (0) idle, (1) trot, (2) yaw, (3) pitch-roll, (4) object-detection
int state = 1; //> indicates the type of gait, (0) idle, (1) trot, (2) yaw, (3) pitch-roll, (4) object-detection
float _period = 10.0; //> indicates the number of steps every second

datatypes::Rotator _sRotation; //> this variable stores the relative rotation of the body

unsigned long duration;
int sample_sum, sample_num = 10,
                sample_index;
float freq;


void loop() {
  duration = millis();

  handle_kinematics(_direction, turn, height, _period);

  handle_input_v7rc_ss8();
//  handle_input_v7rc_srt();
}

float vo, ho;
void init_input() {
  vo = vrt_offset;
  ho = hrz_offset;
}

bool _tb = false;
float stick_min = 6.f;
float lx, ly, rx, ry;
int ch5_data;

void handle_input_v7rc_ss8() {
     // decode V7RC protocol // hex : 64 - 96 - c8    dec : 100 - 150  - 200
       if (receive_data.startsWith("SS8")&&(receive_data.length()==20)){
          run_status = false; 
          int ch1_data=(hexToDec(receive_data.substring(3,5))*10); 
          int ch2_data=(hexToDec(receive_data.substring(5,7))*10); 
          int ch3_data=(hexToDec(receive_data.substring(7,9))*10); 
          int ch4_data=(hexToDec(receive_data.substring(9,11))*10); 
           ch5_data=(hexToDec(receive_data.substring(11,13))*10); 
          int ch6_data=(hexToDec(receive_data.substring(13,15))*10); 
          int ch7_data=(hexToDec(receive_data.substring(15,17))*10); 
          int ch8_data=(hexToDec(receive_data.substring(17,19))*10); 

          float in_lx = map(ch3_data, 1000, 2000, -128, 127);
          float in_ly = map(ch2_data, 1000, 2000, -128, 127);  // f b 
          float in_rx = map(ch1_data, 1000, 2000, -128, 127);  // l r 
          float in_ry = map(ch4_data, 1000, 2000, -128, 127);

          // fine-tune hrz_offset / vrt_offset  
          vrt_offset = map(ch7_data, 1000, 2000, -15, 15);  // -6
          hrz_offset  = map(ch8_data, 1000, 2000, -15, 15);
  //        frequency  = map(ch8_data, 1000, 2000, 400, 800);
          step_x = map(ch6_data, 1000, 2000, 5, 20);  // 15

         lx = inter(lx, in_lx/ 4.0f, 0.5f); //> gets the interpolated x-position of the left  analog stick
         ly = inter(ly, in_ly/ 4.0f, 0.5f); //> gets the interpolated y-position of the left  analog stick
         rx = inter(rx, in_rx/ 4.0f, 0.5f); //> gets the interpolated x-position of the right analog stick
         ry = inter(ry, in_ry/ 4.0f, 0.5f); //> gets the interpolated y-position of the right analog stick

    if (abs(lx) > stick_min) { //> checks whether the stick position is out of the deadzone
      float x0 = lx - stick_min * sign(lx); //> subtracts the deadzone
      if (state == 1) {
        _direction.y = 0;//x0 / 10.f;
      } else if (state != 4) {
        _direction.y = x0 / 2;
      }
    } else _direction.y = 0;

    if (abs(ly) > stick_min) { //> checks whether the stick position is out of the deadzone
      float y0 = ly - stick_min * sign(ly); //> subtracts the deadzone
      if (state == 1) {
        _direction.x = y0 / 10.f;
        if (y0 > 0)
          vrt_offset = vrt_offset - 10.f;  // -20
        else
          vrt_offset = vrt_offset - 5.f;   // 0

      } else if (state != 4) {
        _direction.x = y0 / 2;
        vrt_offset = vo;
      }
    } else {
      _direction.x = 0;
      vrt_offset = vo;
    };

    if (abs(rx) > stick_min) { //> checks whether the stick position is out of the deadzone
      float x1 = rx - stick_min * sign(rx); //> subtracts the deadzone
      if (state == 1)
        turn = x1 / 16.f;
      else if (state != 4)
        turn = x1;
    } else turn = 0;

    if (abs(ry) > stick_min) { //> checks whether the stick position is out of the deadzone
      float y1 = ry - stick_min * sign(ry); //> subtracts the deadzone
      height = y1;
    } else height = 0;
   }

    if (ch5_data > 1700) {
    if (_tb == true) {
      _tb = false; state++;
      if (state > 4) state = 0;      
      } 
    }else _tb = true;
}

void handle_input_v7rc_srt() {
     // decode V7RC protocol // 
       if (receive_data.startsWith("SRT")&&(receive_data.length()==20)){
          run_status = false; 
          int ch1_data=(receive_data.substring(3,7)).toInt(); 
          int ch2_data=(receive_data.substring(7,11)).toInt();
          int ch3_data=(receive_data.substring(11,15)).toInt(); 
          int ch4_data=(receive_data.substring(15,19)).toInt();

          float in_lx = map(ch3_data, 1000, 2000, -128, 127);
          float in_ly = map(ch2_data, 1000, 2000, -128, 127);  // f b 
          float in_rx = map(ch1_data, 1000, 2000, -128, 127);  // l r 
          float in_ry = map(ch4_data, 1000, 2000, -128, 127);
         Serial.println(in_lx);

        lx = inter(lx, in_lx/ 4.0f, 0.5f); //> gets the interpolated x-position of the left  analog stick
        ly = inter(ly, in_ly/ 4.0f, 0.5f); //> gets the interpolated y-position of the left  analog stick
        rx = inter(rx, in_rx/ 4.0f, 0.5f); //> gets the interpolated x-position of the right analog stick
        ry = inter(ry, in_ry/ 4.0f, 0.5f); //> gets the interpolated y-position of the right analog stick

    if (abs(lx) > stick_min) { //> checks whether the stick position is out of the deadzone
      float x0 = lx - stick_min * sign(lx); //> subtracts the deadzone
      if (state == 1) {
        _direction.y = 0;//x0 / 10.f;
      } else if (state != 4) {
        _direction.y = x0 / 2;
      }
    } else _direction.y = 0;

    if (abs(ly) > stick_min) { //> checks whether the stick position is out of the deadzone
      float y0 = ly - stick_min * sign(ly); //> subtracts the deadzone
      if (state == 1) {
        _direction.x = y0 / 10.f;
////        if (y0 > 0)
////          vrt_offset = inter(vrt_offset, vo - 6.f, 2.f);
////        else
 /////         vrt_offset = inter(vrt_offset, vo + 3.f, 2.f);
      } else if (state != 4) {
        _direction.x = y0 / 2;
        vrt_offset = vo;
      }
    } else {
      _direction.x = 0;
      vrt_offset = vo;
    };

    if (abs(rx) > stick_min) { //> checks whether the stick position is out of the deadzone
      float x1 = rx - stick_min * sign(rx); //> subtracts the deadzone
      if (state == 1)
        turn = x1 / 16.f;
      else if (state != 4)
        turn = x1;
    } else turn = 0;

    if (abs(ry) > stick_min) { //> checks whether the stick position is out of the deadzone
      float y1 = ry - stick_min * sign(ry); //> subtracts the deadzone
      height = y1;
    } else height = 0;
   }
}

// !! make sure you have enabled Newline or Carriage return
#define _mode 1 // (0) used for calibration and testing, (1) uses serial as input

//: this is an interpolation function used to smooth
float inter(float in, float en, float pl) {
  if (in < en - pl) {
    return ((in * 1000.f) + (pl * 1000.f)) / 1000.0;
  } else if (in > en + pl) {
    return ((in * 1000.f) - (pl * 1000.f)) / 1000.0;
  } else return en;
}
