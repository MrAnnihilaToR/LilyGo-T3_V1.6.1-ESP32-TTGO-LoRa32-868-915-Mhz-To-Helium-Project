/*
LilyGo T3_V1.6.1, ESP32, TTGO with LoRa32 - 868-915 Mhz, Sensor Values simulation;
Created by Mario Andreev, 17.12.22
*/

//Arduino header
#include <Arduino.h>

// Libraries for LoRa 
#include <SPI.h>
#include <LoRa.h>

// Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Libraries for MCCI LoRaWAN
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <arduino_lmic_hal_boards.h>

// define the pins used by the LoRa transceiver module
#define SCK 5 
#define MISO 19 
#define MOSI 27 
#define SS 18 //SX1276_CS / SS
#define RST 23 // SX1276_RST
#define DI0 26 // SX1276_DI0
#define SX1276_DI1 33 // SX1276_DI1
#define SX1276_DI2 32 // SX1276_DI2
#define SX1276_SPI_FREQ 8000000 ///SX1276 SPI Frequency

/// This is Manual Configuration of the current PinMap ;////////////////// Check getpinmap_thisboard.cpp
lmic_pinmap obj =  {
  
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23,

  .dio = {
      26, // DIO0 (IRQ) is D25
      33, // DIO1 is D26
      32, // DIO2 is D27
  },
  .rxtx_rx_active = 0,
   .rssi_cal = 10,
    .spi_freq = 8000000, /* 8MHz */
};

const lmic_pinmap *pPinMap = &obj;

// 433E6 for Asia
// 866E6 for Europe
// 915E6 for North America
#define BAND 866E6 //Europe

// OLED pins
#define OLED_SDA 21 
#define OLED_SCL 22 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//C includes
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// packet counter
int counter = 0;

// Let's make 4 integers and simulate sensor values
int num1 = 100;
int num2 = 150;
int num3 = 200;
int num4 = 250;

///Declaration of the display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire); /// no LED_RST here

/////////////////////LoRaWAN OTAA to HELIUM things//////////////////////////////////////

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //TTN SERVER ONLY! -> Last Bytes: 0xD5, 0xB3, 0x70
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

uint8_t mydata[16]; ///The data, which will be sent
static osjob_t sendjob;


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

void printHex2(unsigned v)
{
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
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
    //Change the data, before it is queued ....
    
    ///Converting the integers(num1,num2,num3,num4) to char arrays
  
    char str1[10];
    itoa(num1, str1, 10);
    ////
    char str2[10];
    itoa(num2, str2, 10);
    ////
    char str3[10];
    itoa(num3, str3, 10);
    ////
    char str4[10];
    itoa(num4, str4, 10);

    char buffer[16]; //Making a char array buffer to store all the sensor values
    strcat(buffer , str1);
    strcat(buffer , " ");
    strcat(buffer, str2);
    strcat(buffer, " ");
    strcat(buffer, str3);
    strcat(buffer, " ");
    strcat(buffer, str4);
    
    //assigning the data to "mydata" variable (in this case are numbers only)
    for(int i=0;i < sizeof(mydata);i++) 
    {
      mydata[i] = buffer[i];
      if(!(buffer[i]>='0' && buffer[i] <= '9')){
        mydata[i] = ' ';
        buffer[i] = ' ';
      }
    }

    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(buffer);
    Serial.println(F("Packet queued"));
    counter++;
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
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    {
      u4_t netid = 0;
      devaddr_t devaddr = 0;
      u1_t nwkKey[16];
      u1_t artKey[16];
      LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      Serial.print("netid: ");
      Serial.println(netid, DEC);
      Serial.print("devaddr: ");
      Serial.println(devaddr, HEX);
      Serial.print("AppSKey: ");
      for (size_t i = 0; i < sizeof(artKey); ++i)
      {
        if (i != 0)
          Serial.print("-");
        printHex2(artKey[i]);
      }
      Serial.println("");
      Serial.print("NwkSKey: ");
      for (size_t i = 0; i < sizeof(nwkKey); ++i)
      {
        if (i != 0)
          Serial.print("-");
        printHex2(nwkKey[i]);
      }
      Serial.println();
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
  ||     Serial.println(F("EV_RFU1"));
  ||     break;
  */
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen)
    {
      Serial.println(F("Received "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
    }
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
  /*
  || This event is defined but not used in the code. No
  || point in wasting codespace on it.
  ||
  || case EV_SCAN_FOUND:
  ||    Serial.println(F("EV_SCAN_FOUND"));
  ||    break;
  */
  case EV_TXSTART:
    Serial.println(F("EV_TXSTART"));
    break;
  case EV_TXCANCELED:
    Serial.println(F("EV_TXCANCELED"));
    break;
  case EV_RXSTART:
    /* do not print anything -- it wrecks timing */
    break;
  case EV_JOIN_TXCOMPLETE:
    Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
    break;

  default:
    Serial.print(F("Unknown event: "));
    Serial.println((unsigned)ev);
    break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial){
    yield();
  }
  // initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false))
  { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever -> smart
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("LORA SENDER ");
  display.display();

  Serial.println("LoRa Sender Test");

  // SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);

  // setup LoRa transceiver module
  LoRa.setPins(SS, RST, DI0);

  if (!LoRa.begin(BAND))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0, 10);
  display.print("LoRa Initializing OK!");
  display.display();
  delay(2000);
  Serial.println("C'mon dude, let's start");
  display.println("C'mon dude, let's start");
  //display.println(mydata);
  display.display();

  //////LoRaWAN code part /////////////////////////////////////////
  delay(5000);
  while (!Serial)
   ;
  //Serial.begin(115200);
  Serial.println(F("Starting"));

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif
  
   #if defined(ARDUINO_DISCO_L072CZ_LRWAN1)
    SPI.setMOSI(RADIO_MOSI_PORT);
    SPI.setMISO(RADIO_MISO_PORT);
    SPI.setSCLK(RADIO_SCLK_PORT);
    SPI.setSSEL(RADIO_NSS_PORT);
  #endif
  
  // Pin mapping
  //  We use the built-in mapping -- see src/hal/getpinmap_thisboard.cpp
  //
  // If your board isn't supported, declare an lmic_pinmap object as static
  // or global, and set pPimMap to that pointer.
  //
  ///const lmic_pinmap *pPinMap = Arduino_LMIC::GetPinmap_ThisBoard();
  
  // don't die mysteriously; die noisily.
  if (pPinMap == nullptr)
  {
    pinMode(LED_BUILTIN, OUTPUT);
    for (;;)
    {
      // flash lights, sleep.
      for (int i = 0; i < 5; ++i)
      {
        digitalWrite(LED_BUILTIN, 1);
        delay(100);
        digitalWrite(LED_BUILTIN, 0);
        delay(900);
      }
      Serial.println(F("board not known to library; add pinmap or update getconfig_thisboard.cpp"));
    }
  }

  os_init_ex(pPinMap);

  //LMIC_init(); Do we need it ?
  //os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // allow much more clock error than the X/1000 default. See:
  // https://github.com/mcci-catena/arduino-lorawan/issues/74#issuecomment-462171974
  // https://github.com/mcci-catena/arduino-lmic/commit/42da75b56#diff-16d75524a9920f5d043fe731a27cf85aL633
  // the X/1000 means an error rate of 0.1%; the above issue discusses using values up to 10%.
  // so, values from 10 (10% error, the most lax) to 1000 (0.1% error, the most strict) can be used.
  //LMIC_setClockError(1 * MAX_CLOCK_ERROR / 40); /// ORIGINAL
  //LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
  LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 1000);
  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7, 14);
  //LMIC_selectSubBand(1);

/*
#if CFG_LMIC_EU_like // CFG_LMIC_EU_like_MASK
  // This makes joins faster in the EU because we don't wander all over the
  // spectrum.
  LMIC_selectSubBand(1);
#endif
*/

//Region is defined in platformio.ini

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop()
{
  os_runloop_once();
}


