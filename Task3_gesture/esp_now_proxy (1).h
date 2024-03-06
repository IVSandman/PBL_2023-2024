#include <esp_now.h>
#include <WiFi.h>

typedef struct mpuMessage { // สร้างตัวแปรแพ็จเกจแบบ struct
  float yaw = 0.00f;
  float pitch = 0.00f;
  float roll = 0.00f;
} mpuMessage;

mpuMessage read_mpu_msg; // ตัวแปรแพ็คเกจที่ต้องการส่ง

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&read_mpu_msg, incomingData, sizeof(read_mpu_msg));
}

void esp_now_proxy_init() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
