#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <U8g2lib.h>
#include <Button2.h>
#include <Wire.h>
#include <time.h>


/*!
 * Board function definitions
 */
#define UART_DEBUG

#ifdef UART_DEBUG
#define DEBUG(x)            Serial.print(x);
#define DEBUGLN(x)          Serial.println(x);
#else
#define DEBUG(x)
#define DEBUGLN(x)
#endif

/*!
 * Board MUC pins definitions
 */
#define RADIO_RESET                                 PB10
#define RADIO_MOSI                                  PB15
#define RADIO_MISO                                  PB14
#define RADIO_SCLK                                  PB13
#define RADIO_NSS                                   PB12

#define RADIO_DIO_0                                 PB11
#define RADIO_DIO_1                                 PC13
#define RADIO_DIO_2                                 PB9
#define RADIO_DIO_3                                 PB4
#define RADIO_DIO_4                                 PB3
#define RADIO_DIO_5                                 PA15

#define RADIO_ANT_SWITCH_RXTX                       PA1 //1:Rx, 0:Tx

#define UART_TX                                     PA9
#define UART_RX                                     PA10

#define GPS_RST                                     PB2
#define GPS_RX                                      PC11
#define GPS_TX                                      PC10
#define GPS_LEVEL_SHIFTER_EN                        PC6
#define GPS_BAUD_RATE                               115200
#define GPS_1PPS_INTPUT                             PB5
#define I2C_SCL                                     PB6
#define I2C_SDA                                     PB7

#define USER_BUTTON                                 PA4
#define SSD1306_ADDRRESS                            0x3C

#ifndef USBCON
HardwareSerial DebugSerial(UART_RX, UART_TX);
#define Serial DebugSerial
#endif

#define DATABUFFER_SIZE 128
#define LORA_BAND       915E6

HardwareSerial gpsPort(GPS_RX, GPS_TX);
TinyGPSPlus gps;
Button2 btn(USER_BUTTON);

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

uint8_t func_select = 2;
bool sx1278_ok = false;
bool isSave = false;
uint32_t timeStamp = 0;
int counter = 0;

char dataBuffer[5][DATABUFFER_SIZE] = {
    {"00.00"},
    {"00.00"},
    {"00.00"},
    {"0"},
    {"00/00/00-00:00:00"}
};

/**
 * @brief  setRadioDirection
 * @note   Lora ANT Switch
 * @param  rx: true:Rx, false:Tx
 * @retval None
 */
void setRadioDirection(bool rx)
{
    digitalWrite(RADIO_ANT_SWITCH_RXTX, rx ? HIGH : LOW);
}

bool loarSetup()
{
    SPI.setMISO(RADIO_MISO);
    SPI.setMOSI(RADIO_MOSI);
    SPI.setSCLK(RADIO_SCLK);
    SPI.begin();
    LoRa.setSPI(SPI);
    LoRa.setPins(RADIO_NSS, RADIO_RESET, RADIO_DIO_0);// set CS, reset, IRQ pin

    if (!LoRa.begin(LORA_BAND)) {             // initialize ratio at 915 MHz
        return false;
    }
    //! Initialize Radio ant switch pin
    pinMode(RADIO_ANT_SWITCH_RXTX, OUTPUT);
    //! Lora ANT Switch 1:Rx, 0:Tx
    setRadioDirection(true);
    return true;
}


void gpsSetup()
{
    gpsPort.begin(GPS_BAUD_RATE);

    //! Added 1pps intput
    pinMode(GPS_1PPS_INTPUT, INPUT);

    pinMode(GPS_LEVEL_SHIFTER_EN, OUTPUT);
    digitalWrite(GPS_LEVEL_SHIFTER_EN, HIGH);

    pinMode(GPS_RST, GPIO_PULLUP);
    //Set  Reset Pin as 0
    digitalWrite(GPS_RST, LOW);
    //Scope shows 1.12s (Low Period)
    delay(200);
    //Set  Reset Pin as 1
    digitalWrite(GPS_RST, HIGH);
    delay(100);

    //! Start GPS connamd
    gpsPort.println("@GSR");
}

void gpsDetect()
{
    // Dispatch incoming characters
    while (gpsPort.available() > 0)
        gps.encode(gpsPort.read());

    if (gps.charsProcessed() < 10) {
        DEBUGLN(F("WARNING: No GPS data.  Check wiring."));
    }

    if (gps.location.isUpdated()) {

        if (gps.location.isValid() && !isSave) {
            isSave = true;
            gpsPort.println("@BUP");
        }

        DEBUG(F("LOCATION   Fix Age="));
        DEBUG(gps.location.age());
        DEBUG(F("ms Raw Lat="));
        DEBUG(gps.location.rawLat().negative ? "-" : "+");
        DEBUG(gps.location.rawLat().deg);
        DEBUG("[+");
        DEBUG(gps.location.rawLat().billionths);
        DEBUG(F(" billionths],  Raw Long="));
        DEBUG(gps.location.rawLng().negative ? "-" : "+");
        DEBUG(gps.location.rawLng().deg);
        DEBUG("[+");
        DEBUG(gps.location.rawLng().billionths);
        DEBUG(F(" billionths],  Lat="));
        DEBUG(gps.location.lat());
        DEBUG(F(" Long="));
        DEBUGLN(gps.location.lng());

        snprintf(dataBuffer[0], DATABUFFER_SIZE, "%.2f", gps.location.lat());
        snprintf(dataBuffer[1], DATABUFFER_SIZE, "%.2f", gps.location.lng());
    }
    if (gps.date.isUpdated()) {

        DEBUG(F("DATE       Fix Age="));
        DEBUG(gps.date.age());
        DEBUG(F("ms Raw="));
        DEBUG(gps.date.value());
        DEBUG(F(" Year="));
        DEBUG(gps.date.year());
        DEBUG(F(" Month="));
        DEBUG(gps.date.month());
        DEBUG(F(" Day="));
        DEBUGLN(gps.date.day());

    }
    if (gps.time.isUpdated()) {

        DEBUG(F("TIME       Fix Age="));
        DEBUG(gps.time.age());
        DEBUG(F("ms Raw="));
        DEBUG(gps.time.value());
        DEBUG(F(" Hour="));
        DEBUG(gps.time.hour());
        DEBUG(F(" Minute="));
        DEBUG(gps.time.minute());
        DEBUG(F(" Second="));
        DEBUG(gps.time.second());
        DEBUG(F(" Hundredths="));
        DEBUGLN(gps.time.centisecond());

        snprintf(dataBuffer[4], DATABUFFER_SIZE, "%u/%u/%u-%u:%u:%u",
                 gps.date.year() % 100,
                 gps.date.month(),
                 gps.date.day(),
                 gps.time.hour(),
                 gps.time.minute(),
                 gps.time.second()
                );
    }
    if (gps.speed.isUpdated()) {
        //! U8g2 Speed
        double speed = gps.speed.kmph();
        snprintf(dataBuffer[2], DATABUFFER_SIZE, "%.2f", speed);

        DEBUG(F("SPEED      Fix Age="));
        DEBUG(gps.speed.age());
        DEBUG(F("ms Raw="));
        DEBUG(gps.speed.value());
        DEBUG(F(" Knots="));
        DEBUG(gps.speed.knots());
        DEBUG(F(" MPH="));
        DEBUG(gps.speed.mph());
        DEBUG(F(" m/s="));
        DEBUG(gps.speed.mps());
        DEBUG(F(" km/h="));
        DEBUGLN(gps.speed.kmph());

    }
    if (gps.course.isUpdated()) {
        DEBUG(F("COURSE     Fix Age="));
        DEBUG(gps.course.age());
        DEBUG(F("ms Raw="));
        DEBUG(gps.course.value());
        DEBUG(F(" Deg="));
        DEBUGLN(gps.course.deg());
    }
    if (gps.altitude.isUpdated()) {
        DEBUG(F("ALTITUDE   Fix Age="));
        DEBUG(gps.altitude.age());
        DEBUG(F("ms Raw="));
        DEBUG(gps.altitude.value());
        DEBUG(F(" Meters="));
        DEBUG(gps.altitude.meters());
        DEBUG(F(" Miles="));
        DEBUG(gps.altitude.miles());
        DEBUG(F(" KM="));
        DEBUG(gps.altitude.kilometers());
        DEBUG(F(" Feet="));
        DEBUGLN(gps.altitude.feet());
    }
    if (gps.satellites.isUpdated()) {

        DEBUG(F("SATELLITES Fix Age="));
        DEBUG(gps.satellites.age());
        DEBUG(F("ms Value="));
        DEBUGLN(gps.satellites.value());

        snprintf(dataBuffer[3], DATABUFFER_SIZE, "%lu", gps.satellites.value());

    } else if (gps.hdop.isUpdated()) {
        DEBUG(F("HDOP       Fix Age="));
        DEBUG(gps.hdop.age());
        DEBUG(F("ms Value="));
        DEBUGLN(gps.hdop.value());
    }

    static uint32_t stampTime = 0;

    if (millis() - stampTime > 1000 ) {
        u8g2.clearBuffer();

        //! U8g2 Location
        u8g2.setFont(u8g2_font_helvR10_tr);
        u8g2.drawStr(60, 32, "lat:");
        u8g2.drawStr(85, 32, dataBuffer[0]);
        u8g2.drawStr(60, 46, "lng:");
        u8g2.drawStr(85, 46, dataBuffer[1]);

        //! U8g2 Speed
        u8g2.setFont(u8g2_font_profont22_mf);
        u8g2.drawStr(0, 16, dataBuffer[2]);
        u8g2.drawStr(76, 16, "km/h");
        u8g2.drawStr(39, 42, dataBuffer[3]);

        //! U8g2 satellites
        u8g2.setFont(u8g2_font_5x7_tf  );
        u8g2.drawRFrame(2, 20, 53, 28, 6);
        u8g2.drawStr(5, 30, "SATE:");

        //! U8g2 datetime
        u8g2.setFont(u8g2_font_helvR10_tr);
        u8g2.drawStr(4, 64, dataBuffer[4]);

        u8g2.sendBuffer();
        stampTime = millis();
    }
}

void setup()
{
    Serial.begin(115200);

    delay(2000);

    Wire.setSCL(I2C_SCL);

    Wire.setSDA(I2C_SDA);

    Wire.begin();

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_profont22_mf );
    u8g2.drawStr(32, 24, "SoftRF");
    u8g2.drawStr(48, 42, "and");
    u8g2.drawStr(32, 60, "LilyGO");
    u8g2.sendBuffer();
    delay(8000);

    if (loarSetup()) {
        sx1278_ok = true;
    } else {
        DEBUGLN("LoRa init failed.");
    }

    gpsSetup();

    btn.setPressedHandler([](Button2 & b) {
        DEBUGLN("pressed");
        func_select = func_select + 1 > 2 ? 0 : func_select + 1 ;
        switch (func_select) {
        case 0:
            DEBUGLN("Sender Started");
            setRadioDirection(true);
            break;
        case 1:
            DEBUGLN("Received Started");
            setRadioDirection(false);
            break;
        case 2:
            u8g2.clear();
            u8g2.setFont(u8x8_font_8x13_1x2_f);
        default:
            break;
        }
    });
}

void loraSender()
{
    if (millis() - timeStamp > 3000) {
        if (!sx1278_ok) {
            DEBUGLN("Sx1278 is not initialize ...");
            return;
        }
        timeStamp = millis();
        DEBUG("Sending packet: ");
        DEBUGLN(counter);
        // send packet
        LoRa.beginPacket();
        LoRa.print("hello ");
        LoRa.print(counter);
        LoRa.endPacket();
        counter++;
    }
}

void loarReciver()
{
    if (!sx1278_ok) {
        DEBUGLN("Sx1278 is not initialize ...");
        return;
    }
    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        DEBUG("Received packet '");
        // read packet
        while (LoRa.available()) {
            DEBUG((char)LoRa.read());
        }
        // print RSSI of packet
        DEBUG("' with RSSI ");
        DEBUGLN(LoRa.packetRssi());
    }
}


void loop()
{
    btn.loop();

    switch (func_select) {
    case 0:
        loraSender();
        break;
    case 1:
        loarReciver();
        break;
    case 2:
        gpsDetect();
        break;
    default:
        break;
    }
}