#include <esp_now.h>
#include <WiFi.h>

typedef struct ControlPacket {
  int throttle;
  int roll;
  int pitch;
  int yaw;
  bool armed;
} ControlPacket;

typedef struct HeartbeatPacket {
  bool isHeartbeat;
} HeartbeatPacket;

typedef struct AckPacket {
  bool heartbeatAck;
} AckPacket;

ControlPacket control = {172, 992, 992, 992, false};
HeartbeatPacket heartbeat = {true};
AckPacket ack;

uint8_t receiverMac[] = {0xa0, 0x85, 0xe3, 0x0e, 0x31, 0x1c};  

unsigned long lastAckTime = 0;
unsigned long lastHeartbeatSent = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000;
const unsigned long HEARTBEAT_TIMEOUT = 1500;
bool connectionAlive = false;

void onAckRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(AckPacket)) {
    memcpy(&ack, incomingData, len);
    if (ack.heartbeatAck) {
      lastAckTime = millis();
      if (!connectionAlive) {
        connectionAlive = true;
        Serial.println("✅ Connection re-established via heartbeat ACK.");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Sender Booting...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onAckRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(receiverMac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  Serial.println("ESP-NOW Sender Ready");
}

void handleSerialInput() {
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

    // Send updated control packet
    esp_now_send(receiverMac, (uint8_t *)&control, sizeof(control));
  }
}

void loop() {
  unsigned long now = millis();

  // Process incoming Serial input (if any)
  handleSerialInput();

  // Send heartbeat at regular interval
  if (now - lastHeartbeatSent >= HEARTBEAT_INTERVAL) {
    esp_now_send(receiverMac, (uint8_t *)&heartbeat, sizeof(heartbeat));
    lastHeartbeatSent = now;
    Serial.println("📡 Heartbeat sent");
  }

  // Check for ACK timeout
  if (connectionAlive && (now - lastAckTime > HEARTBEAT_TIMEOUT)) {
    connectionAlive = false;
    Serial.println("❌ No heartbeat ACK received - connection lost.");
  }

  delay(20);  // Small delay to ease CPU load
}
