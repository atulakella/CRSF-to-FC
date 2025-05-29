#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#define PIN_TX_OUT 10    // TX to Crossfire receiver
#define PIN_RX_UNUSED -1 // Not used

HardwareSerial crsfSerialOut(1); // Use Serial1 for output
AlfredoCRSF crsfOut;

bool armed_state = false;
int throttle_value = 172; // Min throttle
int roll_value = 992;     // Center
int pitch_value = 992;    // Center
int yaw_value = 992;      // Center

void setup() {
  Serial.begin(115200);
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
      case 't':
        armed_state = true;
        Serial.println("ARM ON");
        break;
      case 'g':
        armed_state = false;
        Serial.println("ARM OFF");
        break;
      case 'w':
        throttle_value += 50;
        if (throttle_value > 1811) throttle_value = 1811;
        Serial.print("Throttle Up -> "); Serial.println(throttle_value);
        break;
      case 's':
        throttle_value -= 50;
        if (throttle_value < 172) throttle_value = 172;
        Serial.print("Throttle Down -> "); Serial.println(throttle_value);
        break;
      case 'j':
        roll_value -= 50;
        if (roll_value < 172) roll_value = 172;
        Serial.print("Roll Left -> "); Serial.println(roll_value);
        break;
      case 'l':
        roll_value += 50;
        if (roll_value > 1811) roll_value = 1811;
        Serial.print("Roll Right -> "); Serial.println(roll_value);
        break;
      case 'i':
        pitch_value += 50;
        if (pitch_value > 1811) pitch_value = 1811;
        Serial.print("Pitch Forward -> "); Serial.println(pitch_value);
        break;
      case 'k':
        pitch_value -= 50;
        if (pitch_value < 172) pitch_value = 172;
        Serial.print("Pitch Backward -> "); Serial.println(pitch_value);
        break;
      case 'a':
        yaw_value -= 50;
        if (yaw_value < 172) yaw_value = 172;
        Serial.print("Yaw Left -> "); Serial.println(yaw_value);
        break;
      case 'd':
        yaw_value += 50;
        if (yaw_value > 1811) yaw_value = 1811;
        Serial.print("Yaw Right -> "); Serial.println(yaw_value);
        break;
      case 'r':
        roll_value = 992;
        pitch_value = 992;
        yaw_value = 992;
        Serial.println("Reset Roll, Pitch, Yaw to center");
        break;
      default:
        Serial.println("Unknown command.");
        break;
    }
  }

  sendManualChannels();
  delay(20); // 50Hz
}

void sendManualChannels() {
  crsf_channels_t crsfChannels = {0};

  crsfChannels.ch0 = roll_value;
  crsfChannels.ch1 = pitch_value;
  crsfChannels.ch2 = throttle_value;
  crsfChannels.ch3 = yaw_value;
  crsfChannels.ch4 = armed_state ? 1805 : 172;

  // Manually assign remaining channels to 172 since bit-fields can't use pointer arithmetic
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

  crsfOut.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
                      &crsfChannels, sizeof(crsfChannels));
}
