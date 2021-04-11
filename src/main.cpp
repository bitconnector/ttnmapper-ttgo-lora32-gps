#include <Arduino.h>

#define LED 4
#define ButtonPin 38

//Power
#include <axp20x.h>
AXP20X_Class axp;
#define I2C_SDA 21
#define I2C_SCL 22
#define PMU_IRQ 35

//GPS
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define GPS_RX 12
#define GPS_TX 34
#define GPS_Baud 9600

TinyGPSPlus gps;
HardwareSerial ss(1);

// LMIC
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRa Pins
#define LoRa_RST 23
#define LoRa_CS 18
#define LoRa_DIO0 26
#define LoRa_DIO1 33
#define LoRa_DIO2 32

#include "credentials.h"
#include "locations.h"

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

char mydata[6 + 1]; // +1 for sprintf
static uint16_t packetNumber = 0;
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned TX_INTERVAL = 20;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = LoRa_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LoRa_RST,
    .dio = {LoRa_DIO0, LoRa_DIO1, LoRa_DIO2},
};

void blinkLED(unsigned long duration);

void gps_setup()
{

    //https://github.com/LilyGO/TTGO-T-Beam/blob/master/GPS-T22_v1.0-20190612/GPS-T22_v1.0-20190612.ino

    Wire.begin(21, 22);
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
    {
        Serial.println("AXP192 Begin PASS");
    }
    else
    {
        Serial.println("AXP192 Begin FAIL");
    }

    //axp.setLDO3Voltage(3300); //GPS  VDD

    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); //GPS
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    ss.begin(9600, SERIAL_8N1, 34, 12); //17-TX 18-RX
    //ss.begin(9600, SERIAL_8N1, 34, 12); //17-TX 18-RX
    //ss.begin(9600, SERIAL_8N1, 12, 34); //false

    //ss.begin(GPS_Baud, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.print(F("TinyGPS++ library v. "));
    Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println();

    // byte message[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
    //                   0x80, 0x25, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79};
    // for (int i = 0; i < sizeof(message); i++)
    //     ss.write(message[i]);

    while (false)
    {
        while (Serial.available())
        {
            ss.print(Serial.read());
        }
        while (ss.available())
        {
            Serial.write(ss.read());
        }
    }
}

void gps_loop()
{
    unsigned long lock = millis();
    while (ss.available() > 0 && millis() < lock + 2)
    {
        gps.encode(ss.read());
    }
}
uint8_t txBuffer[9];
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
boolean confirmed = false;
double last_lat;
double last_lng;

void PayloadNow()
{

    //if (button_count() == 1) confirmed = true;

    if (gps.location.age() < 10000)
    {
        Serial.println("GPS valid");

        LatitudeBinary = ((gps.location.lat() + 90) / 180) * 16777215;
        LongitudeBinary = ((gps.location.lng() + 180) / 360) * 16777215;

        txBuffer[0] = (LatitudeBinary >> 16) & 0xFF;
        txBuffer[1] = (LatitudeBinary >> 8) & 0xFF;
        txBuffer[2] = LatitudeBinary & 0xFF;

        txBuffer[3] = (LongitudeBinary >> 16) & 0xFF;
        txBuffer[4] = (LongitudeBinary >> 8) & 0xFF;
        txBuffer[5] = LongitudeBinary & 0xFF;

        altitudeGps = gps.altitude.meters();
        txBuffer[6] = (altitudeGps >> 8) & 0xFF;
        txBuffer[7] = altitudeGps & 0xFF;

        if (gps.hdop.isValid())
            hdopGps = gps.hdop.hdop() * 10;
        else
            hdopGps = 144;
        txBuffer[8] = hdopGps & 0xFF;

        u1_t port = 21;

        for (int i = 0; i < (sizeof(geofence) / sizeof(Geofence)); i++)
        {
            if (gps.location.lat() > geofence[i].lat_min && gps.location.lat() < geofence[i].lat_max && gps.location.lng() > geofence[i].lng_min && gps.location.lng() < geofence[i].lng_max)
            {
                port += (i + 1);
                break;
            }
        }

        if (gps.distanceBetween(gps.location.lat(), gps.location.lng(), last_lat, last_lng) < 25) //test, if location has changed significantly
            port += 100;
        else
        {
            last_lat = gps.location.lat();
            last_lng = gps.location.lng();
        }

        LMIC_setTxData2(port, txBuffer, sizeof(txBuffer), confirmed);
    }
    else
    {
        Serial.println("GPS not valid");
        LMIC_setTxData2(1, txBuffer, 0, confirmed);
    }
    confirmed = false;
}

void blinkShort(int times)
{
    digitalWrite(LED, HIGH);
    delay(300);
    for (int i = 0; i < times; i++)
    {
        digitalWrite(LED, LOW);
        delay(50);
        digitalWrite(LED, HIGH);
        delay(150);
    }
}

void switchDR()
{
    u1_t actual = LMIC.datarate;
    switch (actual)
    {
    case DR_SF10:
        actual = DR_SF7;
        TX_INTERVAL = 20; //8-200
        Serial.println(F("Switching Datarate to DR_SF7"));
        blinkShort(1);
        break;
    case DR_SF9:
        actual = DR_SF10;
        TX_INTERVAL = 80; //60-1200
        Serial.println(F("Switching Datarate to DR_SF10"));
        blinkShort(4);
        break;
    case DR_SF8:
        actual = DR_SF9;
        TX_INTERVAL = 40; //30-800
        Serial.println(F("Switching Datarate to DR_SF9"));
        blinkShort(3);
        break;
    default: //DR_SF7
        actual = DR_SF8;
        TX_INTERVAL = 20; //15-400
        Serial.println(F("Switching Datarate to DR_SF8"));
        blinkShort(2);
        break;
    }
    LMIC_setDrTxpow(actual, 14);
}

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        //sprintf(mydata, "l%5u", packetNumber);
        //LMIC_setTxData2(1, (xref2u1_t)mydata, sizeof(mydata) - 1, 0);
        blinkLED(1000);
        PayloadNow();
        Serial.println(F("Packet queued"));
        //packetNumber++;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
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
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
        {
            Serial.println(F("Received ack"));
            blinkLED(3000);
        }
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
        //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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

void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(ButtonPin, INPUT);
    // init packet counter
    sprintf(mydata, "l%5u", packetNumber);

    Serial.begin(115200);
    Serial.println(F("Starting"));

    gps_setup();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

    // disable channels (only use channel 0 - 868.1 MHz - for my single channel gateway!!!)
    // for (int channel = 1; channel <= 8; channel++) {
    //   LMIC_disableChannel(channel);
    // }

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);
}

bool Button = 0;
unsigned long ButtonTime;

void parseButton()
{
    if (!digitalRead(ButtonPin))
    {
        if (!Button)
        {
            ButtonTime = millis();
            Button = 1;
        }
        else
        {
            if (millis() - ButtonTime < 300)
            {
                digitalWrite(LED, HIGH);
            }
            else if (millis() - ButtonTime < 2000)
            {
                digitalWrite(LED, LOW);
            }
            else if (millis() - ButtonTime < 5000)
            {
                digitalWrite(LED, HIGH);
            }
            else
            {
                digitalWrite(LED, LOW);
            }
        }
    }
    else if (Button)
    {
        Button = 0;
        if (millis() - ButtonTime < 300)
        {
            Serial.println("got 1");
            confirmed = true;
            //param++;
        }
        else if (millis() - ButtonTime < 2000)
        {
            Serial.println("got 2");
            switchDR();
            //screen++;
        }
        else if (millis() - ButtonTime < 5000)
        {
            Serial.println("got 3");
            //role++;
        }
        else
        {
            Serial.println("got 4");
        }
        //updateRoleScreenParam();
    }
}

unsigned long Time1, Timed;

unsigned long LEDIntervall;

void blinkLED(unsigned long duration)
{
    digitalWrite(LED, LOW);
    LEDIntervall = millis() + duration;
}

void updateLED()
{
    if (LEDIntervall < millis())
        digitalWrite(LED, HIGH);
}

void loop()
{
    parseButton();

    if (millis() - Time1 > 1000 * TX_INTERVAL)
    {
        Time1 = millis();
        do_send(&sendjob);
    }

    gps_loop();
    os_runloop_once();
    updateLED();
}
