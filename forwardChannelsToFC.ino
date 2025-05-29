#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#define PIN_TX_OUT 10  // TX to Crossfire receiver
#define PIN_RX_UNUSED -1  // Not used

HardwareSerial crsfSerialOut(1); // Use Serial1 for output
AlfredoCRSF crsfOut;

void setup() {
  crsfSerialOut.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX_UNUSED, PIN_TX_OUT);
  delay(500);
  crsfOut.begin(crsfSerialOut);
}

void loop() {
  sendManualChannels();

  delay(20); // 50Hz update rate (20ms interval)
}

// Sends fixed or manual RC channel values
void sendManualChannels() {
  crsf_channels_t crsfChannels = { 0 };

  // Manually set channel values (range: 172 to 1811)
  crsfChannels.ch0 = 172;  // Roll
  crsfChannels.ch1 = 172;  // Pitch
  crsfChannels.ch2 = 172;  // Throttle
  crsfChannels.ch3 = 172;  // Yaw
  crsfChannels.ch4 = 1805;  // Arm switch
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
