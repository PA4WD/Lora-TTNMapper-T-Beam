#include <Arduino.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <TinyGPS++.h>
#include <WiFi.h>

//#define PRINTDEBUG

// UPDATE the config.h file in the same folder WITH YOUR TTN KEYS AND ADDR.
#include "config.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60 /* Time ESP32 will go to sleep (in seconds) */

// T-Beam specific hardware
#define BUILTIN_LED 14
#define GPS_TX 12
#define GPS_RX 15

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

#define GPS_OFF "0xB5\x62\x02\x41\x08\x00\x00\x00\x00\x00\x02\x00\x00\x00\x4D\x3B"
#define GPS_ON "\xB5\x62\x02\x41\x08\x00\x00\x00\x00\x00\x01\x00\x00\x00\x4C\x37"
#define GPS_SETPSM "\xB5\x62\x06\x11\x02\x00\x08\x01\x22\x92" // Setup for Power Save Mode (Default Cyclic 1s)

uint8_t txBuffer[9];

void build_packet()
{
  TinyGPSPlus gps;
#ifdef PRINTDEBUG
  Serial.println("gps wakeup");
#endif
  Serial1.print(F(GPS_ON));
  
  Serial1.print(F(GPS_SETPSM));

  while (!gps.location.isValid() || !gps.altitude.isValid() || !gps.hdop.isValid()) {
    while (Serial1.available())
    {
#ifdef PRINTDEBUG
      char chr = Serial1.read();
      Serial.print(chr);
      gps.encode(chr);
#else
      gps.encode(Serial1.read());
#endif
    }
  }

#ifdef PRINTDEBUG
  Serial.print("LOC valid  = ");  Serial.println(gps.location.isValid());
  Serial.print("ALT valid  = ");  Serial.println(gps.altitude.isValid());
  Serial.print("HDOP valid = ");  Serial.println(gps.hdop.isValid());

  Serial.print("LAT = ");  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG= "); Serial.println(gps.location.lng(), 6);
  Serial.print("ALT = ");  Serial.println(gps.altitude.meters());
  Serial.print("HDOP= ");  Serial.println(gps.hdop.value() / 100.0);
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

  txBuffer[8] = (gps.hdop.value() / 10) & 0xFF;

#ifdef PRINTDEBUG
  String toLog = "";
  for (size_t i = 0; i < sizeof(txBuffer); i++)
  {
    char buffer[3];
    sprintf(buffer, "%02x", txBuffer[i]);
    toLog = toLog + String(buffer);
  }
  Serial.print("TTN Message = ");
  Serial.println(toLog);

  Serial.println("gps sleep");
#endif  
  Serial1.print(F(GPS_OFF));
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
      // {
      //   u4_t netid = 0;
      //   devaddr_t devaddr = 0;
      //   u1_t nwkKey[16];
      //   u1_t artKey[16];
      //   LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      //   Serial.print("netid: ");
      //   Serial.println(netid, DEC);
      //   Serial.print("devaddr: ");
      //   Serial.println(devaddr, HEX);
      //   Serial.print("artKey: ");
      //   for (int i = 0; i < sizeof(artKey); ++i) {
      //     Serial.print(artKey[i], HEX);
      //   }
      //   Serial.println("");
      //   Serial.print("nwkKey: ");
      //   for (int i = 0; i < sizeof(nwkKey); ++i) {
      //     Serial.print(nwkKey[i], HEX);
      //   }
      //   Serial.println("");
      // }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      //LMIC_setLinkCheckMode(0);
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
      digitalWrite(BUILTIN_LED, HIGH);
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

      Serial.println("Enter light sleep");
      Serial.flush();
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      //Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
      esp_light_sleep_start();
      Serial.println("Wake up");

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
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  digitalWrite(BUILTIN_LED, LOW);
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    build_packet();
    LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);

    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
  digitalWrite(BUILTIN_LED, HIGH);
}



void setup() {
  Serial.begin(115200);
  Serial.println(F("Lora Tracker reboot"));

  //Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);

  Serial1.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  Serial1.print(F(GPS_SETPSM));
 
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

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.


  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  //LMIC_setAdrMode(0);
  // Let LMIC compensate for +/- 1% clock error
  //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
