//ตัวส่ง ESP32
#include <esp_now.h>
#include <WiFi.h>

//A0:B7:65:F6:19:9C
//A0:B7:65:F6:19:9C

uint8_t broadcastAddress[] = {0xA0, 0xB7, 0x65, 0xF6, 0x19, 0x9C};//ส่งไปหาเฉพาะ mac address
//uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//ส่งไปหาทุกตัว

// Structure example to send data
// Must match the receiver structure

typedef struct mpuMessage { // สร้างตัวแปรแพ็จเกจแบบ struct
  float yaw = 0.00f;
  float pitch = 0.00f;
  float roll = 0.00f;
} mpuMessage;

mpuMessage read_mpu_msg; // ตัวแปรแพ็คเกจที่ต้องการส่ง

esp_now_peer_info_t peerInfo;

//เมื่อส่งข้อมูลมาทำฟังก์ชั่นนี้
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //  Serial.print("\r\nLast Packet Send Status:\t");
  //  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void esp_now_proxy_init() {
  //ตั้งเป็นโหมด Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void esp_now_proxy_handler(int16_t latency = 0 , bool debug = false) {
  // Send message via ESP-NOW
  static unsigned long premillis = 0;
  if (millis() - premillis >= latency) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &read_mpu_msg, sizeof(read_mpu_msg));
    if (debug) {
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
    }
    premillis = millis();
  }
}
