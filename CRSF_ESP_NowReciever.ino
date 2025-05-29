#include <esp_now.h>
#include <WiFi.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

typedef struct ControlPacket {
  int throttle;
  int roll;
  int pitch;
  int yaw;
  bool armed;
} ControlPacket;

ControlPacket latestControl = {172, 992, 992, 992, false};

HardwareSerial crsfSerialOut(1);
AlfredoCRSF crsfOut;

#define PIN_TX_OUT 10
#define PIN_RX_UNUSED -1

// To track sender MAC for replying ACKs if needed
uint8_t lastSenderMac[6];

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlPacket)) {
    memcpy(&latestControl, incomingData, len);
    memcpy(lastSenderMac, info->src_addr, 6);
    
    Serial.println("✅ Control packet received:");
    Serial.print("Throttle: "); Serial.print(latestControl.throttle);
    Serial.print(", Roll: "); Serial.print(latestControl.roll);
    Serial.print(", Pitch: "); Serial.print(latestControl.pitch);
    Serial.print(", Yaw: "); Serial.print(latestControl.yaw);
    Serial.print(", Armed: "); Serial.println(latestControl.armed ? "YES" : "NO");
  }
}

void sendManualChannels() {
  crsf_channels_t ch = {0};

  ch.ch0 = latestControl.roll;
  ch.ch1 = latestControl.pitch;
  ch.ch2 = latestControl.throttle;
  ch.ch3 = latestControl.yaw;
  ch.ch4 = latestControl.armed ? 1805 : 172;

  ch.ch5 = ch.ch6 = ch.ch7 = ch.ch8 = ch.ch9 = ch.ch10 = ch.ch11 = ch.ch12 = ch.ch13 = ch.ch14 = ch.ch15 = 172;

  crsfOut.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &ch, sizeof(ch));
}

void setup() {
  Serial.begin(115200);
  Serial.println("Receiver Booting...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("ESP-NOW Receiver Ready");

  crsfSerialOut.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX_UNUSED, PIN_TX_OUT);
  crsfOut.begin(crsfSerialOut);
  Serial.println("CRSF Started");
}

void loop() {
  sendManualChannels();
  delay(20);
}
