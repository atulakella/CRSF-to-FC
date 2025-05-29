#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#define PIN_TX_OUT 10    // TX to Crossfire receiver
#define PIN_RX_UNUSED -1 // Not used

HardwareSerial crsfSerialOut(1); // Use Serial1 for output
AlfredoCRSF crsfOut;

bool armed_state = false;
unsigned long last_toggle_time = 0;

void setup() {
  Serial.begin(115200); // For ESP32 debug output if needed
  Serial.println("CRSF Sender Starting...");

  crsfSerialOut.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX_UNUSED, PIN_TX_OUT);
  delay(500);
  crsfOut.begin(crsfSerialOut);
  Serial.println("CRSF Initialized.");
}

void loop() {
  // Toggle arm state every 5 seconds for testing
  if (millis() - last_toggle_time > 5000) {
    armed_state = !armed_state;
    last_toggle_time = millis();
    if (armed_state) {
      Serial.println("Attempting to send ARM ON signal");
    } else {
      Serial.println("Attempting to send ARM OFF signal");
    }
  }

  sendManualChannels(armed_state);
  delay(20); // 50Hz update rate (20ms interval)
}

// Sends fixed or manual RC channel values
void sendManualChannels(bool arm) {
  crsf_channels_t crsfChannels = {0};

  // Manually set channel values (range: 172 to 1811)
  crsfChannels.ch0 = 992; // Roll - Center
  crsfChannels.ch1 = 992; // Pitch - Center
  crsfChannels.ch2 = 172; // Throttle - Min
  crsfChannels.ch3 = 992; // Yaw - Center

  if (arm) {
    crsfChannels.ch4 = 1805; // Arm switch - ON
  } else {
    crsfChannels.ch4 = 172;  // Arm switch - OFF
  }

  crsfChannels.ch5 = 172;  // Aux2
  crsfChannels.ch6 = 172;  // Aux3
  crsfChannels.ch7 = 172;  // Aux4
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