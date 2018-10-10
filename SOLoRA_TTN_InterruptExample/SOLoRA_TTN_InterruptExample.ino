UNDER DEVELOPMENT
/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 * Modified 2018 Joe Miller for SOLoRa.
 *              Also added sleep after 'hello world' sent 
 * 
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network. It's pre-configured for the
 * SOLoRa Board (M0, Arduino Zero and Adafruit M0 feather compatible)
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//#include "LowPower.h"
#include <RTCZero.h>

// system operation type. Choose either RTC_TIMER wakeups for reporting (this example),
//    or interrupt based (HW_ITNERRUPT), in which case your sensor must invoke a hardware
//    pin setup in Arduino as an interrupt input ...
//    attachInterrupt(digitalPinToInterrupt(pin), ISR, mode);
//    where: pin  = 5,6,9,10,12,14,15,16,17,18,19 (available SOLoRa pins)
//    where: ISR  = &yourISRFunction();
//    where: mode = LOW to trigger the interrupt whenever the pin is low,
//                  CHANGE to trigger the interrupt whenever the pin changes value
//                  RISING to trigger when the pin goes from low to high,
//                  FALLING for when the pin goes from high to low.
//                  HIGH to trigger the interrupt whenever the pin is high.
#define RTC_TIMER (0)
#define HW_INTERRUPT (1)
// Set wakeup type below.
#define WAKEUP_TYPE RTC_TIMER //<<<<<<<<<<< Set this

#if (WAKEUP_TYPE == RTC_TIMER)
/* Create an rtc object */
RTCZero rtc;
#endif

// Set DEBUG_MESSAGES to 1 to enable debug messages to USB terminal
//                Set to 0 for final firmware compile when ready for field use with no USB terminal
#define DEBUG_MESSAGES (0)
//
#if DEBUG_MESSAGES
#define D(x) Serial.print(x);
#define DL(x) Serial.println(x);
#define D2(x, y) Serial.print(x, y);
#define DL2(x, y) Serial.println(x, y);
#else
#define D(x)
#define DL(x)
#define D2(x, y)
#define DL2(x, y)
#endif

#define LED (13) // SOLoRa Red LED port pin
// LED consumes ~1.1mA. 100ms pulse = 110uA*s = 0.0303uA*hrs

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte (LSB x x ... x x MSB)
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,0x70.
static const u1_t PROGMEM APPEUI[8] = {0xA9, 0x09, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above. (LSB x x ... x x MSB)
static const u1_t PROGMEM DEVEUI[8] = {0x6F, 0xDA, 0x2A, 0x00, 0x68, 0xB4, 0x84, 0x00};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in BIG endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
// (MSB x x x x x x x x x x x x x x LSB)
static const u1_t PROGMEM APPKEY[16] = {0x26, 0x12, 0x02, 0x4C, 0x0C, 0x80, 0x02, 0xAE, 0xB3, 0xD9, 0xDE, 0x19, 0x91, 0x5E, 0x79, 0x6C};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t mydata[] = "Hello, world!";
static osjob_t send_packet_job;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 15; // set fast in this example for debugging

// Pin mapping
//#if defined(ARDUINO_SAMD_FEATHER_M0)
// Pin mapping for Adafruit Feather M0 LoRa, SOLoRa, etc.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 11, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8, // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};
//#else
//# error "Unknown target"
//#endif

void alarmMatch()
{
    // Schedule next packet with the os_ to send packet.
    os_setTimedCallback(&send_packet_job, 50, send_packet);
}

void onEvent(ev_t ev)
{
    D(os_getTime());
    D(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        DL(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        DL(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        DL(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        DL(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        DL(F("EV_JOINING"));
        break;
    case EV_JOINED:
        DL(F("EV_JOINED"));
        {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            //D("netid: ");
            //DL2(netid, DEC);
            D("devaddr: ");
            DL2(devaddr, HEX);
            D("artKey: ");
            for (int i = 0; i < sizeof(artKey); ++i)
            {
                if (i != 0)
                    D("-");
                D2(artKey[i], HEX);
            }
            DL("");
            D("nwkKey: ");
            for (int i = 0; i < sizeof(nwkKey); ++i)
            {
                if (i != 0)
                    D("-");
                D2(nwkKey[i], HEX);
            }
            DL("");
            // Blink twice to show join success
            digitalWrite(LED, HIGH);
            delay(50);
            digitalWrite(LED, LOW);
            delay(100);
            digitalWrite(LED, HIGH);
            delay(50);
            digitalWrite(LED, LOW);
        }
        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
        LMIC_setLinkCheckMode(0);
        break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     DL(F("EV_RFU1"));
        ||     break;
        */
    case EV_JOIN_FAILED:
        DL(F("EV_JOIN_FAILED"));
        // Blink three times to denote join fail
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        break;
    case EV_REJOIN_FAILED:
        DL(F("EV_REJOIN_FAILED"));
        // Blink three times to denote join fail
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        break;
    case EV_TXCOMPLETE:
        DL(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            DL(F("Received ack"));
        if (LMIC.dataLen)
        {
            DL(F("Received "));
            DL(LMIC.dataLen);
            DL(F(" bytes of payload"));
        }
        // Blink one once to denote txComplete. comment next statement to save power (~50uA*s/transmission)
        digitalWrite(LED, HIGH);
        delay(50);
        digitalWrite(LED, LOW);

        // Prepare for Sleep/Idle
#if (WAKEUP_TYPE == RTC_TIMER)
        // Schedule next transmission
        // Sleep for a period of TX_INTERVAL using single shot alarm
        rtc.setAlarmEpoch(rtc.getEpoch() + TX_INTERVAL);
        rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
        rtc.standbyMode();
        // try RTCZERO with LP_Sleep() in the future
#else
        // WAKEUP_TYPE == HW_INTERRUPT
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        __DSB();
        __WFI();
        // will only wakeup with pin interrupt
#endif
        break;
    case EV_LOST_TSYNC:
        DL(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        DL(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        DL(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        DL(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        DL(F("EV_LINK_ALIVE"));
        break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    DL(F("EV_SCAN_FOUND"));
        ||    break;
        */
    case EV_TXSTART:
        DL(F("EV_TXSTART"));
        break;
    default:
        D(F("Unknown event: "));
        DL((unsigned)ev);
        break;
    }
}

void send_packet(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        DL(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
        DL(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup()
{
    // initialize unused pins as inputs_pullup
    // other init functions will change pinMode to suit specific needs
    const uint8_t SOLoRaPins[] = {5, 6, 10, 12, 14, 15, 16, 17, 18, 19}; //unused SOLoRa pins
    for (int i = 0; i < sizeof(SOLoRaPins); i++)
    {
        pinMode(SOLoRaPins[i], INPUT_PULLUP);
    }

    // digital pin LED as an output.
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    delay(1000); // 2 x blinks to confirm startup
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
    digitalWrite(LED, LOW); // turn off the LED

    delay(3000);
#if DEBUG_MESSAGES // Debug mode, use serial port
    while (!Serial)
        ;
    Serial.begin(9600);
    DL(F("Starting"));
#else
    // USB port consumes extra current
    USBDevice.detach();
#endif

    //
    // init calls for SOLoRa on-board features
    //
    // uncomment next two lines if enabling on-board accelerometer OR termperature sensor
    //Wire.begin();                // join i2c bus (address optional for master)
    //Serial.begin(9600);          // start serial communication at 9600bps
    //
    // initalize temperature sensor
    //initTemp();
    // initial accelerometer
    //initAccel();
    //
    // initialize ADC for Battery
    //init_readBatteryVoltage();
    //

#if (WAKEUP_TYPE == RTC_TIMER)
    // Initialize RTC
    rtc.begin();
    // Use RTC as a second timer instead of calendar
    rtc.setEpoch(0);
    rtc.attachInterrupt(alarmMatch);
#endif

    //-ignor this stuff. I used it for testing
    // rtc.standbyMode();
    // SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    // __DSB();
    // __WFI();

    //
    // Put your sensor initialization calls here
    //

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7, 2); //
    LMIC_selectSubBand(1);

    // Start job (sending automatically starts OTAA too)
    send_packet(&send_packet_job);
}

void loop()
{
    os_runloop_once();
}
