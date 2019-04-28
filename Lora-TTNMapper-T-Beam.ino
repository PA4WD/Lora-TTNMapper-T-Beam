/* To decode the binary payload, you can use the following
   javascript decoder function. It should work with the TTN console.

  function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};
  // if (port === 1) decoded.led = bytes[0];
  decoded.lat = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
  decoded.lat = (decoded.lat / 16777215.0 * 180) - 90;
  decoded.lon = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
  decoded.lon = (decoded.lon / 16777215.0 * 360) - 180;
  var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
  var sign = bytes[6] & (1 << 7);
  if(sign)
  {
    decoded.alt = 0xFFFF0000 | altValue;
  }
  else
  {
    decoded.alt = altValue;
  }
  decoded.hdop = bytes[8] / 10.0;
  return decoded;
  }

*/
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>

#define PRINTDEBUG

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "config.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60 /* Time ESP32 will go to sleep (in seconds) */

// T-Beam specific hardware
#define BUILTIN_LED 21
#define GPS_TX 12
#define GPS_RX 15

// The TinyGPS++ object
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

#ifdef OTAA
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}
#endif
#ifdef ABP
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#endif

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN, // was "14,"
  .dio = {26, 33, 32},
};

#define PMTK_SET_NMEA_UPDATE_05HZ  "$PMTK220,2000*1C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

//fake u-blox  XM37-1612
//unsigned char StandbyMode[] = {"$PMTK161,0*28\x0D\x0A"};
//#define PMTK_SET_NMEA_STANDBY "$PMTK161,0*28\x0D\x0A"
//unsigned char PeriodicMode[] = {"$PMTK225,1,5000,12000*1C\x0D\x0A"}; // sec Navigation and 12 sec sleep in Backup state.
//unsigned char PeriodicModeStop[] = {"$PMTK225,0*2B\x0D\x0A"};

uint8_t txBuffer[9];

void build_packet()
{
tryagain:
  while (GPSSerial.available())
  {
    gps.encode(GPSSerial.read());
  }
  if (gps.location.lat() == 0 || gps.location.lng() == 0) {
    //esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR); //sleep 1 second
    //esp_light_sleep_start();
    goto tryagain;
  }

#ifdef PRINTDEBUG
  Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
  Serial.print("ALT=");  Serial.println(gps.altitude.meters());
  Serial.print("HDOP=");  Serial.println(gps.hdop.value());
#endif

  uint32_t LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
  uint32_t LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;

  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  uint16_t altitudeGps = gps.altitude.meters();
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  //uint8_t hdopGps = gps.hdop.value() / 10;
  //txBuffer[8] = hdopGps & 0xFF;
  txBuffer[8] = (gps.hdop.value() / 10) & 0xFF;

#ifdef PRINTDEBUG
  String toLog = "";
  for (size_t i = 0; i < sizeof(txBuffer); i++)
  {
    char buffer[3];
    sprintf(buffer, "%02x", txBuffer[i]);
    toLog = toLog + String(buffer);
  }
  Serial.println(toLog);
#endif
}

char s[32]; // used to sprintf for Serial output

void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received Ack"));
      }
      if (LMIC.dataLen) {
        sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
      }
      // Schedule next transmission
      //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

      //Serial.println("light_sleep_enter");
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      //Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
      //delay(100);
      esp_light_sleep_start();
      //Serial.println("Wake up");

      do_send(&sendjob);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    build_packet();
    LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);

    Serial.println(F("Packet queued"));
    digitalWrite(BUILTIN_LED, HIGH);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Lora Tracker reboot"));

  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();

  GPSSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  GPSSerial.setTimeout(2);

  GPSSerial.println(F(PMTK_SET_NMEA_OUTPUT_RMCGGA));
  GPSSerial.println(F(PMTK_SET_NMEA_UPDATE_1HZ));   // 1 Hz update rate
  //GPSSerial.println(F(PMTK_SET_NMEA_STANDBY));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

#ifdef ABP
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#endif

  LMIC_setLinkCheckMode(1);
  LMIC_setAdrMode(0);
  // Let LMIC compensate for +/- 1% clock error
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF11, 14);

  // Start job
  do_send(&sendjob);

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
}

void loop() {
  os_runloop_once();
}
