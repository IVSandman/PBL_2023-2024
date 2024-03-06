//========== BNO055 ==========
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)

//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

//========== Serial Proxy ==========
#include "esp_now_proxy.h"

void setup() {
  Serial.begin(115200);
  //         sda, scl
  Wire.begin(19,22);
  
  esp_now_proxy_init();

  delay(100);
  Serial.printf("Orientation Sensor Test!! \n");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
}

void loop() {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  read_mpu_msg.roll = event.orientation.z;
  read_mpu_msg.pitch = event.orientation.y;
  read_mpu_msg.yaw = event.orientation.x;

  Serial.printf("roll : %f , pitch : %f , yaw : %f \n" , event.orientation.z, event.orientation.y, event.orientation.x);
  esp_now_proxy_handler(0);

//  delay(10); 
}
