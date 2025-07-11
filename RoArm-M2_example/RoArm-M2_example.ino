#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<256> jsonInfoSend;
StaticJsonDocument<256> jsonInfoHttp;

#include <SCServo.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <nvs_flash.h>

//functions for IMU
#include "IMU.h"

// functions for oled.
#include "oled_ctrl.h"

// functions for RoArm-M2 ctrl.
#include "RoArm-M2_module.h"

// functions for pneumatic modules and lights ctrl. 
#include "switch_module.h"

// define json cmd.
#include "json_cmd.h"

// functions for editing the files in flash.
#include "files_ctrl.h"

// advance functions for RoArm-M2 ctrl.
#include "RoArm-M2_advance.h"

// functions for wifi ctrl.
#include "wifi_ctrl.h"

// functions for esp-now.
#include "esp_now_ctrl.h"

// functions for uart json ctrl.
#include "uart_ctrl.h"

// functions for http & web server.
#include "http_server.h"

EulerAngles stAngles;
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;

unsigned long prevTime = 0;
float gyroYawRate = 0.0;
float newYaw = 0.0;

void getIMU(){
  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  gyroYawRate = stGyro RawData.Z;

  newYaw += gyroYawRate * dt;

  if (abs(newYaw) > 1.0) {
    RoArmM2_singleJointAngleCtrl(BASE_JOINT, newYaw, 60, 60);
  }
}

void moveIMU() {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  gyroYawRate = stGyroRawData.Z;
  newYaw += gyroYawRate * dt;

  if (abs(newYaw) > 1.0) {
    RoArmM2_singleJointAngleCtrl(BASE_JOINT, newYaw, 60, 60);
  }
}

void printIMU(){
   // Serial.println("gyroYawRate : " + String(stGyroRawData.Z) + "    newYaw : " + String(newYaw));
    // Serial.println();
    // Serial.println("/-------------------------------------------------------------/");
    // Serial.print("Roll : "); Serial.print(stAngles.roll);
    // Serial.print("    Pitch : "); Serial.print(stAngles.pitch);
    // Serial.print("    Yaw : "); Serial.print(stAngles.yaw);
    // Serial.println();
    // Serial.print("Acceleration: X : "); Serial.print(stAccelRawData.X);
    // Serial.print("    Acceleration: Y : "); Serial.print(stAccelRawData.Y);
    // Serial.print("    Acceleration: Z : "); Serial.print(stAccelRawData.Z);
    // Serial.println();
    // Serial.print("Gyroscope: X : "); Serial.print(stGyroRawData.X);
    // Serial.print("       Gyroscope: Y : "); Serial.print(stGyroRawData.Y);
    // Serial.print("       Gyroscope: Z : "); Serial.print(stGyroRawData.Z);
    // Serial.println();
    // Serial.print("Magnetic: X : "); Serial.print(stMagnRawData.s16X);
    // Serial.print("      Magnetic: Y : "); Serial.print(stMagnRawData.s16Y);
    // Serial.print("      Magnetic: Z : "); Serial.print(stMagnRawData.s16Z);
    // Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(S_SDA, S_SCL);
  while(!Serial) {}

  delay(1200);

  initOLED();
  screenLine_0 = "RoArm-M2";
  screenLine_1 = "version: 0.84";
  screenLine_2 = "starting...";
  screenLine_3 = "";
  oled_update();

  //init the IMU
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize IMU";
  oled_update();
  if (InfoPrint == 1) {Serial.println("Initialize IMU sensor");}
  imuInit();

  // init the littleFS funcs in files_ctrl.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize LittleFS";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize LittleFS for Flash files ctrl.");}
  initFS();

  // init the funcs in switch_module.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize 12V-switch ctrl";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize the pins used for 12V-switch ctrl.");}
  switchPinInit();

  // servos power up
  screenLine_2 = screenLine_3;
  screenLine_3 = "Power up the servos";
  oled_update();
  if(InfoPrint == 1){Serial.println("Power up the servos.");}
  delay(500);
  
  // init servo ctrl functions.
  screenLine_2 = screenLine_3;
  screenLine_3 = "ServoCtrl init UART2TTL...";
  oled_update();
  if(InfoPrint == 1){Serial.println("ServoCtrl init UART2TTL...");}
  RoArmM2_servoInit();

  // check the status of the servos.
  screenLine_2 = screenLine_3;
  screenLine_3 = "Bus servos status check...";
  oled_update();
  if(InfoPrint == 1){Serial.println("Bus servos status check...");}
  RoArmM2_initCheck(false);

  if(InfoPrint == 1 && RoArmM2_initCheckSucceed){
    Serial.println("All bus servos status checked.");
  }
  if(RoArmM2_initCheckSucceed) {
    screenLine_2 = "Bus servos: succeed";
  } else {
    screenLine_2 = "Bus servos: " + 
    servoFeedback[BASE_SERVO_ID - 11].status +
    servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].status +
    servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].status +
    servoFeedback[ELBOW_SERVO_ID - 11].status +
    servoFeedback[GRIPPER_SERVO_ID - 11].status;
  }
  screenLine_3 = ">>> Moving to init pos...";
  oled_update();
  RoArmM2_resetPID();
  RoArmM2_moveInit();

  screenLine_3 = "Reset joint torque to ST_TORQUE_MAX";
  oled_update();
  if(InfoPrint == 1){Serial.println("Reset joint torque to ST_TORQUE_MAX.");}
  RoArmM2_dynamicAdaptation(0, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX);

  screenLine_3 = "WiFi init";
  oled_update();
  if(InfoPrint == 1){Serial.println("WiFi init.");}
  initWifi();

  screenLine_3 = "http & web init";
  oled_update();
  if(InfoPrint == 1){Serial.println("http & web init.");}
  initHttpWebServer();

  screenLine_3 = "ESP-NOW init";
  oled_update();
  if(InfoPrint == 1){Serial.println("ESP-NOW init.");}
  initEspNow();

  screenLine_3 = "RoArm-M2 started";
  oled_update();
  if(InfoPrint == 1){Serial.println("RoArm-M2 started.");}

  getThisDevMacAddress();

  updateOledWifiInfo();

  screenLine_2 = String("MAC:") + macToString(thisDevMac);
  oled_update();

  if(InfoPrint == 1){Serial.println("Application initialization settings.");}
  createMission("boot", "these cmds run automatically at boot.");
  missionPlay("boot", 1);

  RoArmM2_handTorqueCtrl(300);
  
}


void loop() {

  serialCtrl();
  server.handleClient();

  unsigned long curr_time = millis();
  if (curr_time - prev_time >= 10){
    constantHandle();
    prev_time = curr_time;
  }

  RoArmM2_getPosByServoFeedback();

  if (InfoPrint == 2) {
    RoArmM2_infoFeedback();
  }

  if(runNewJsonCmd) {
    jsonCmdReceiveHandler();
    jsonCmdReceive.clear();
    runNewJsonCmd = false;
  }

  getIMU();


} 