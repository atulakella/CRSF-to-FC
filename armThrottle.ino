#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#define PIN_TX_OUT 10    // TX to Crossfire receiver
#define PIN_RX_UNUSED -1 // Not used

HardwareSerial crsfSerialOut(1); // Use Serial1 for output
AlfredoCRSF crsfOut;

bool armed_state = false;
int throttle_value = 172; // Start with minimum throttle

void setup() {
  Serial.begin(115200); // For ESP32 debug output if needed
  Serial.println("CRSF Sender Starting...");

  crsfSerialOut.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX_UNUSED, PIN_TX_OUT);
  delay(500);
  crsfOut.begin(crsfSerialOut);
  Serial.println("CRSF Initialized.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'a':
        armed_state = true;
        Serial.println("Received 'a': ARM ON");
        break;
      case 'd':
        armed_state = false;
        Serial.println("Received 'd': ARM OFF");
        break;
      case 'w':
        throttle_value += 50; // Increase throttle
        if (throttle_value > 1811) throttle_value = 1811;
        Serial.print("Received 'w': Throttle Up -> ");
        Serial.println(throttle_value);
        break;
      case 's':
        throttle_value -= 50; // Decrease throttle
        if (throttle_value < 172) throttle_value = 172;
        Serial.print("Received 's': Throttle Down -> ");
        Serial.println(throttle_value);
        break;
      default:
        Serial.println("Unknown command.");
        break;
    }
  }

  sendManualChannels(armed_state, throttle_value);
  delay(20); // 50Hz update rate
}

// Sends fixed or manual RC channel values
void sendManualChannels(bool arm, int throttle) {
  crsf_channels_t crsfChannels = {0};

  crsfChannels.ch0 = 992;          // Roll - Center
  crsfChannels.ch1 = 992;          // Pitch - Center
  crsfChannels.ch2 = throttle;     // Throttle
  crsfChannels.ch3 = 992;          // Yaw - Center
  crsfChannels.ch4 = arm ? 1805 : 172; // Arm switch

  // Set remaining aux channels to min
  crsfChannels.ch5 = 172;
  crsfChannels.ch6 = 172;
  crsfChannels.ch7 = 172;
  crsfChannels.ch8 = 172;
  crsfChannels.ch9 = 172;
  crsfChannels.ch10 = 172;
  crsfChannels.ch11 = 172;
  crsfChannels.ch12 = 172;
  crsfChannels.ch13 = 172;
  crsfChannels.ch14 = 172;
  crsfChannels.ch15 = 172;

  crsfOut.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfChannels, sizeof(crsfChannels));
}

