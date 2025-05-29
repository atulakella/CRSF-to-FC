#include <esp_now.h>
#include <WiFi.h>

typedef struct ControlPacket {
  int throttle;
  int roll;
  int pitch;
  int yaw;
  bool armed;
} ControlPacket;

ControlPacket control = {172, 992, 992, 992, false};

// REPLACE with your actual receiver MAC address
uint8_t receiverMAC[] = {0xA0, 0x85, 0xE3, 0x0E, 0x31, 0x1C};

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Delivery: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success ✅" : "Failed ❌");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(receiverMAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  Serial.println("Sender ready with ACK callback.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case 't': control.armed = true; Serial.println("ARM ON"); break;
      case 'g': control.armed = false; Serial.println("ARM OFF"); break;
      case 'w': control.throttle = min(control.throttle + 50, 1811); Serial.println(control.throttle); break;
      case 's': control.throttle = max(control.throttle - 50, 172); Serial.println(control.throttle); break;
      case 'j': control.roll = max(control.roll - 50, 172); Serial.println(control.roll); break;
      case 'l': control.roll = min(control.roll + 50, 1811); Serial.println(control.roll); break;
      case 'i': control.pitch = min(control.pitch + 50, 1811); Serial.println(control.pitch); break;
      case 'k': control.pitch = max(control.pitch - 50, 172); Serial.println(control.pitch); break;
      case 'a': control.yaw = max(control.yaw - 50, 172); Serial.println(control.yaw); break;
      case 'd': control.yaw = min(control.yaw + 50, 1811); Serial.println(control.yaw); break;
      case 'r': control.roll = 992; control.pitch = 992; control.yaw = 992; Serial.println("Reset RPY"); break;
      default: Serial.println("Unknown command."); break;
    }

    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&control, sizeof(control));
    if (result != ESP_OK) {
      Serial.println("❌ Error sending message");
    }
  }

  delay(20);
}
