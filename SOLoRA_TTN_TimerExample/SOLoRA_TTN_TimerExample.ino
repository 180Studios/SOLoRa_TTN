/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
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
 /* Modified 2018,2019 Joe Miller for SOLoRa.
 *      + Sensors added optional LM75 temperature and LIS3DH accelerometer
 *      + added modifiable code to operate as either periodic timer wakeup or hardware interrupt
 *      + Payloads are formatted for Cayenne services for data repository and visualization
 *          - https://mydevices.com/cayenne/docs/lora/#lora-cayenne-low-power-payload
 *      + Improvements made to decrease sleep currrent
 ******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//#include "LowPower.h"
#include <RTCZero.h>
#include "CayenneLPP.h"


#define USE_TEMP_SENSOR         (1)  
#if USE_TEMP_SENSOR 
    #include <Wire.h>
    #define LM75_I2CADDR             0x48
    #define LM75_TEMP                0x00    /**< RAM reg - Temperature */
#endif
float temperature;



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

        // enumerated list of operating modes
#define RTC_TIMER       (0)     
#define HW_INTERRUPT    (1)    
// Set wakeup type below.
#define WAKEUP_TYPE RTC_TIMER //<<<<<<<<<<< Set this to RTC_TIMER or HW_INTERRUPT

#if (WAKEUP_TYPE == RTC_TIMER)
/* Create an rtc object */
RTCZero rtc;
// Schedule TX every this many seconds 
const unsigned TX_INTERVAL = (60*60*12); // set to 1/2 day in seconds
//const unsigned TX_INTERVAL = (30); // for testing
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

#define MOISTURE_SIGNAL A0
#define RESERVOIR_SIGNAL A2
#define CHARGE_SIGNAL A7
#define MOISTURE_POWER (15)
#define CAYENNE (1)         // send data in Cayenne Low Power Protocol format (LPP)
#define VCC_READ_POWER (17)
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
static const u1_t PROGMEM APPEUI[8] = {0xD8, 0x29, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above. (LSB x x ... x x MSB)
static const u1_t PROGMEM DEVEUI[8] = {0xED, 0x77, 0x91, 0x67, 0x92, 0xDD, 0x5F, 0x00};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in BIG endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
// (MSB x x x x x x x x x x x x x x LSB)
static const u1_t PROGMEM APPKEY[16] = {0x1C, 0x68, 0x44, 0xDB, 0xC3, 0x67, 0x71, 0xA7, 0x24, 0x8B, 0x07, 0x58, 0x16, 0x8F, 0x0E, 0x37};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

//static uint8_t mydata[] = "Hello, world!";
struct myPacket_t
{
    uint16_t moisture;
    uint16_t reservoir;
} mydata;

#pragma pack(push,1)  // align this structure on byte boundries
struct cayenne_packet_t
{
    const   uint8_t     channelH = 1;                // Moisture sensor
    const   uint8_t     idH = LPP_RELATIVE_HUMIDITY;// 104 = Humitity (used here for moisture)
            uint8_t     dataH;                     // 0.5%/LSB unsigned
    const   uint8_t     channelt = 2;             // Temperature Sensor
    const   uint8_t     idt = LPP_TEMPERATURE;   // 103, TEMPERATURE
            int16_t     dataT;                  // 0.1C/LSB signed
    const   uint8_t     channelR = 3;          // Reservoir Voltage
    const   uint8_t     idR = 2;              // 2 = Analog input 
            int16_t     dataR;               // scale: 0.01/LSB signed
} cayenneData;
#pragma pack(pop)

static osjob_t send_packet_job;

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


float getTemp(void){
    float temperature;
    uint16_t val;

    // send the device address then the register pointer byte
    Wire.beginTransmission(LM75_I2CADDR);
    Wire.write(LM75_TEMP);
    Wire.endTransmission(false);

    // resend device address then get the 2 returned bytes
    Wire.requestFrom(LM75_I2CADDR, (uint8_t)2);

    // data is returned as 2 bytes big endian
    val = Wire.read() << 8;
    val |= Wire.read();

    temperature = (float)val * 0.125/32.0;
    return temperature;
}

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
        digitalWrite(MOISTURE_POWER, HIGH);
#ifdef USE_TEMP_SENSOR
        temperature = getTemp();
#endif
        delay(100); // settling time for moisture reading
        mydata.moisture = analogRead(MOISTURE_SIGNAL); 
        digitalWrite(MOISTURE_POWER, LOW);
        digitalWrite(VCC_READ_POWER, HIGH);
        delay(10);
        mydata.reservoir = analogRead(RESERVOIR_SIGNAL);
        digitalWrite(VCC_READ_POWER, LOW);
            

#if CAYENNE     // send data in Cayenne low-power-protocol (LPP) format
        // adjust raw 12bits value to Cayenne 0.5%/LSB 8-bit range
        mydata.moisture = (uint8_t)((float)mydata.moisture * -0.458F +390.3F);
        cayenneData.dataH = (uint8_t)mydata.moisture;
    
        //swap bytes on the next two fields to match endianess of Cayenne inputs
        cayenneData.dataT = (((int16_t)(temperature * 10.0F)>>8)&0x00ff) | 
            (((int16_t)(temperature * 10.0F)<<8)&0xff00); // reverse endianess       
     
        // adjust raw value to voltage * 100 for Cayenne analog input format
#define VOLTAGEDIVIDERFACTOR (0.581F) // includes ADC and Cayenne 0.01 resolution factors
        mydata.reservoir = (int16_t)((float)mydata.reservoir * VOLTAGEDIVIDERFACTOR);
        cayenneData.dataR = (((uint16_t)mydata.reservoir>>8)&0x00ff) | 
            (((uint16_t)mydata.reservoir<<8)&0xff00); // reverse endianess       
       LMIC_setTxData2(1, (xref2u1_t)&cayenneData, sizeof(cayenneData), 0);
#else
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, (xref2u1_t)&mydata, sizeof(mydata), 0);
#endif
        D("temp: " );
        DL2(cayenneData.dataT, DEC);
        DL(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}



void setup()
{
    // SOLoRa hardware specific setup
    // initialize unused pins as inputs_pullup. This will reduce sleep current
    // other init functions will change pinMode to suit specific needs
#if USE_TEMP_SENSOR
    const uint8_t SOLoRaPins[] = {5, 6, 7, 9, 10, 12, 18, 19}; //unused SOLoRa pins
#else
    const uint8_t SOLoRaPins[] = {5, 6, 7, 9, 10, 12, 18, 19, 20, 21}; //unused SOLoRa pins
#endif
    //             (A)nalog #                          4   5  
    //pin#=RFM95 signal: 3=IRQ, 4=RST, 8=CSn, 11= , 22=MISO, 23=MOSI, 24=SCK
    //SOLoRa signals 0=RX, 1=Tx, 13=LED, 20=sda, 21=scl, 7=nc, 9(A7)=chrV (charge Voltage Ain)
    //specific to moisture app: 14(A0)=moisture Ain, 15=moisture power, 
    //                          16(A2)=ResV Ain,     17=resV read power
    for (int i = 0; i < sizeof(SOLoRaPins); i++)
    {
        pinMode(SOLoRaPins[i], INPUT_PULLUP);
    }
    pinMode (MOISTURE_POWER,OUTPUT);
    digitalWrite(MOISTURE_POWER, LOW);
    analogReference(AR_INTERNAL1V0);

#if USE_TEMP_SENSOR   
    Wire.begin();                // join i2c bus (address optional for master)
#endif

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
    //init_readchargeVoltage();
    //

    // end of SOLoRa specific hardware setup

#if (WAKEUP_TYPE == RTC_TIMER)
    // Initialize RTC
    rtc.begin();
    // Use RTC as a second timer instead of calendar
    rtc.setEpoch(0);
    rtc.attachInterrupt(alarmMatch);
    DL(F("RTC initalized"));
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
    LMIC_setDrTxpow(DR_SF7, 10); //
    LMIC_selectSubBand(1);

    DL(F("LMIC initialized"));
    // Start job (sending automatically starts OTAA too)
    send_packet(&send_packet_job);
}

void loop()
{
    os_runloop_once();
}
