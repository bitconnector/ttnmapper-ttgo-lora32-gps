#include <Arduino.h>

#define LED 2

// OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//GPS
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define GPS_RX 35
#define GPS_TX 34
#define GPS_Baud 9600

TinyGPSPlus gps;
HardwareSerial ss(1);

// LMIC
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// OLED Pins
#define OLED_SCL 15 // GPIO 15
#define OLED_SDA 4  // GPIO  4
#define OLED_RST 16 // GPIO 16

// define the display type that we use
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);

// LoRa Pins
#define LORA_RST 14  // GPIO 14
#define LORA_CS 18   // GPIO 18
#define LORA_DIO0 26 // GPIO 26
#define LORA_DIO1 33 // GPIO 33
#define LORA_DIO2 32 // GPIO 32

#include "credentials.h"

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
    .nss = LORA_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST,
    .dio = {LORA_DIO0, LORA_DIO1, LORA_DIO2},
};

void gps_setup()
{
    ss.begin(GPS_Baud, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.print(F("TinyGPS++ library v. "));
    Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println();
    while (false)
        while (ss.available() > 0)
        {
            Serial.print(ss.read());
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

void PayloadNow()
{
    boolean confirmed = false;

    //if (button_count() == 1) confirmed = true;

    if (gps.location.isValid())
    {

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
            hdopGps = 40.0 * 10;
        txBuffer[8] = hdopGps & 0xFF;

        LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), confirmed);
    }
    else
    {
        LMIC_setTxData2(1, txBuffer, 0, confirmed);
    }
}

void showDatarate()
{
    display.print("Datarate: ");
    switch (LMIC.datarate)
    {
    case DR_SF7:
        display.print("DR_SF7");
        break;

    case DR_SF8:
        display.print("DR_SF8");
        break;

    case DR_SF9:
        display.print("DR_SF9");
        break;

    case DR_SF10:
        display.print("DR_SF10");
        break;

    default:
        display.print("DR_Error");
        break;
    }
}

void switchDR()
{
    u1_t actual = LMIC.datarate;
    switch (actual)
    {
    case DR_SF10:
        actual = DR_SF7;
        TX_INTERVAL = 10; //8-200
        break;
    case DR_SF9:
        actual = DR_SF10;
        TX_INTERVAL = 80; //60-1200
        break;
    case DR_SF8:
        actual = DR_SF9;
        TX_INTERVAL = 40; //30-800
        break;
    default: //DR_SF7
        actual = DR_SF8;
        TX_INTERVAL = 20; //15-400
        break;
    }
    LMIC_setDrTxpow(actual, 14);
}

void showFrequency()
{
    display.print("Channel: ");
    display.print(LMIC.txChnl);
    char frequency[20];
    float fre = LMIC.freq;
    fre /= 1000000;
    sprintf(frequency, " Freq %6.2f MHz", fre);
    //display.print(frequency);
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
            Serial.println(F("Received ack"));
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
    // init packet counter
    sprintf(mydata, "l%5u", packetNumber);

    Serial.begin(115200);
    Serial.println(F("Starting"));

    gps_setup();

    // set up the display
    Wire.begin(OLED_SDA, OLED_SCL);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.display();

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

bool Button = 1;
unsigned long ButtonTime;

void parseButton()
{
    if (!digitalRead(0))
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

void displayTimestamp()
{
    uint32_t seconds = millis() / 1000;
    uint32_t minutes = seconds / 60;
    seconds %= 60;
    uint32_t hours = minutes / 60;
    minutes %= 60;
    if (hours < 10)
        display.print("0");
    display.print(hours);
    display.print(":");
    if (minutes < 10)
        display.print("0");
    display.print(minutes);
    display.print(":");
    if (seconds < 10)
        display.print("0");
    display.print(seconds);
}

unsigned long Time1, Timed;

void updateDisplay()
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("APB:  ");

    //display.print("send");
    unsigned long remaining = 1000 * TX_INTERVAL + Time1 - millis();
    float delay = float(remaining) / 1000;
    if (delay < 10)
        display.print(" ");
    display.print(delay, 1);
    display.print("  ");
    displayTimestamp();
    display.println();

    float percent = 80 * (float(remaining) / (1000 * TX_INTERVAL));
    display.drawRect(20, 9, 80, 5, SSD1306_WHITE);
    display.fillRect(20, 9, int(percent), 5, SSD1306_WHITE);

    display.println();
    showDatarate();
    display.println();
    showFrequency();
    display.println();
    display.println();

    display.print("send: ");
    //display.print(mydata);
    for (int i = 0; i < sizeof(txBuffer); i++)
        display.print(txBuffer[i], HEX);
    display.println();

    // display.print("Data: ");
    // display.print(LoRa.packetRssi());
    // display.print(" ->");
    // display.println(pow(10, (65 - LoRa.packetRssi()) / (10 * 3)) / 1000, 0);
    // display.print(RecivedData);
    // display.drawRect(0, 60, int(signalStrength()), 4, SSD1306_WHITE);

    //display.drawLine(5, 5, 115, 55, SSD1306_WHITE);
    //display.fillRect(2, 40, 3, 20, SSD1306_WHITE);
    //display.drawRect(5, 40, 3, 10, SSD1306_WHITE);
    //display.drawRect(9, 40-15, 3, 15, SSD1306_WHITE);

    display.display();
}

unsigned long LEDIntervall;

void blinkLED(unsigned long duration)
{
    digitalWrite(LED, HIGH);
    LEDIntervall = millis() + duration;
}

void updateLED()
{
    if (LEDIntervall < millis())
        digitalWrite(LED, LOW);
}

void loop()
{
    parseButton();
    if (millis() - Timed > 80)
    {
        Timed = millis();
        updateDisplay();
    }

    if (millis() - Time1 > 1000 * TX_INTERVAL)
    {
        Time1 = millis();
        do_send(&sendjob);
        blinkLED(300);
    }

    gps_loop();
    os_runloop_once();
    updateLED();
}
