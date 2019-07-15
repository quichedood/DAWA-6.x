/**************************************************************
  DAWA 6.0 (Arduino M0 - Onboard SAMD21G)
  Bootloader have to be burned on new chip with for example Atmel ICE
  Triumph motorbikes laptimer/datalogger
  Edouard PIGEON - 2019
**************************************************************/

/**************************************************************
  TODO
  Terminer/vérifier log fichiers
  Angle inclinaison
  Ajouter nom circuit au nom du fichier
  Ajouter bluetooth init test
  Ajouter gestion temps partiels
  Gestion menu/boutons
**************************************************************/

/**************************************************************
  Enable debug output to serial
**************************************************************/
//#define DEBUG // /!\ DAWA will wait serial connection to start

/**************************************************************
  Debug to serial
  Also used by NeoGPS library
**************************************************************/
#ifdef DEBUG
#define DEBUG_PORT SERIAL_PORT_USBVIRTUAL
#endif

/**************************************************************
  #################################################
  ############## Includes definition ##############
  #################################################
**************************************************************/

/**************************************************************
  SPI library
  It's included with Arduino
**************************************************************/
#include <SPI.h>

/**************************************************************
  I2C library
  It's included with Arduino
**************************************************************/
#include <Wire.h>

/**************************************************************
  EEPROM library
  It's included with Arduino

    extEEPROM.c >
  //REMOVE l.98 :
  //TWBR = ( (F_CPU / twiFreq) - 16) / 2;

  //REPLACE l.98 :
  //sercom3.disableWIRE();                         // Disable the I2C bus
  //SERCOM3->I2CM.BAUD.bit.BAUD = SystemCoreClock / ( 2 * 400000) - 1 ;   // // Set the I2C SCL frequency to 400kHz
  //sercom3.enableWIRE();                          // Restart the I2C bus
  // https://forum.arduino.cc/index.php?topic=347425.0

  extEEPROM.h >
  ADD l.63 :
  #define BUFFER_LENGTH 64 // Add var here because no more defined in wire.h (SAMD21's buffer = 64)

  EEPROM addresses used :
  0-21 BNO Calibration offsets (11x uint16_t)
  25 Ask a calibration offset
  30 ADC disabled bits (1x uint8_t)
  31-32 Throttle max value (1x int16_t)
  33-46 Gear calibration data (7x int16_t)
  50-55 MLX I2C Address (x uint8_t ... 6 max)
**************************************************************/
#include <extEEPROM.h> // Arduino External EEPROM Library v3.0 (Official Github : https://github.com/JChristensen/extEEPROM)
extEEPROM eep(kbits_2, 1, 8); // I2C Address 0x50

/**************************************************************
  Modification of the EEPROMAnything library that allows you to read and write any data structure or array to and from the external EEPROM
  https://forum.arduino.cc/index.php?topic=358648.0
**************************************************************/
#include <EEPROMAnything2.h> // Enable object read/write on EEPROM

/**************************************************************
  I/O Expander MCP23017
  https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
**************************************************************/
#include <Adafruit_MCP23017.h>

/**************************************************************
  Time library
  https://github.com/PaulStoffregen/Time
  Please rename in "TimeLib.h" var "DAYS_PER_WEEK" to "DAYS_PER_WEEK_ALT" as the same var is already used in NeoGPS library
**************************************************************/
#include <Time.h>
#include <TimeLib.h>

/**************************************************************
  Greiman/SdFat library
  https://github.com/greiman/SdFat
**************************************************************/
#include <SdFat.h>

/**************************************************************
  MLX90614 Infrared temperature sensor
  https://github.com/jfitter/MLX90614
**************************************************************/
#include <MLX90614.h>

/**************************************************************
  Modified dtostrf function defnition
  http://forum.arduino.cc/index.php?topic=368720.0
**************************************************************/
#include <dtostrf.h>

/**************************************************************
  NeoGPS library
  https://github.com/SlashDevin/NeoGPS

  GPSport.h :
  #define GPS_PORT Serial3
  #define GPS_PORT_NAME "GPS_PORT"
  #define DEBUG_PORT Serial
**************************************************************/
#include <NMEAGPS.h>
HardwareSerial & GPS_PORT = Serial5;

/**************************************************************
  Bluetooth (HM11 - CC2541)
  http://jnhuamao.cn/bluetooth.asp?id=1
  If you get counterfeit HM-11, some commands won't probably work. Try flash it with original firmware :
  https://forum.arduino.cc/index.php?topic=393655.0
**************************************************************/
#include <wiring_private.h> // pinPeripheral() function
Uart BLUETOOTH_PORT (&sercom5, 7, 6, SERCOM_RX_PAD_3, UART_TX_PAD_2); //Disable SERCOM5 in <APPDATA>\local\Arduino15\packages\arduino\hardware\samd\1.6.14\variants\arduino_mzero\variant.cpp (Serial + Sercom5)

/**************************************************************
  LCD library
  SPI ASCII OLED : https://github.com/greiman/SSD1306Ascii
**************************************************************/
// SPI ASCII OLED libraries
#include <SSD1306Ascii.h> // OLED LCD based on SSD1306 chip, ASCII only no graphics
#include <SSD1306AsciiSpi.h>
#define OLED_DC 9
#define OLED_CS 13

/**************************************************************
  Menu library
  https://github.com/jonblack/arduino-menusystem
**************************************************************/
#include <MenuSystem.h>

/**************************************************************
  ##################################################
  ############## Constants definition ##############
  ##################################################
**************************************************************/

// I/O Pins
const uint8_t inDigiBrakePin = 3; // D3, digital input
const uint8_t inDigiOpt1Pin = 11; // D11, digital input
const uint8_t inAnaThrottlePin = A5; // A5, analog input
const uint8_t inAnaGearPin = A2; // A2, analog input
const uint8_t inAnaOpt1Pin = A3; // A3, analog input
const uint8_t inAnaOpt2Pin = A4; // A4, analog input
const uint8_t sdCsPin = 5; // D5, Chip Select for SDCARD on SPI bus
const uint8_t mcp1IntPin = 8; // D8, Buttons interrupt (MCP23017)
const uint8_t mcp1But1 = 0; // Button 1 on MCP23017
const uint8_t mcp1But2 = 1; // Button 2 on MCP23017
const uint8_t mcp1But3 = 2; // Button 3 on MCP23017
const uint8_t mcp1But4 = 3; // Button 4 on MCP23017
const uint8_t mcp1Led1 = 11; // Led 1 on MCP23017
const uint8_t mcp1Led2 = 10; // Led 1 on MCP23017
const uint8_t mcp1Led3 = 9; // Led 1 on MCP23017
const uint8_t mcp1Led4 = 8; // Led 1 on MCP23017

// Others ...
const char csvDelim = ';'; // CSV file delimiter
const uint8_t maxTrackDistance = 5; // Autoselect nearest track (unit = km)
const uint8_t gearOffset = 20; // Each gear has a corresponding value, this value define the interval (value - gearOffset < measure < value + gearOffset)
const uint8_t maxMlx = 6; // Max MLX chips that could be declared (could be more but need to add object instances L.300, just before setup())
const uint8_t mlxEepAddr = 0x0E; // Internal EEPROM address on MLX chips
const uint8_t firstMlxAddress = 0x01; // First given MLX I2C address (then +1 on each discovered MLX sensor)

// GPS & Timing
const float rescaleGPS = 10000000.0; // We use "long" for GPS coordinates to keep precision ("float" on Arduino have only 6 decimal digits of precision) ### https://gis.stackexchange.com/questions/8650/measuring-accuracy-of-latitude-and-longitude

// GPS Configuration - General
const unsigned char ubxRate10Hz[] PROGMEM = {0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00};
const unsigned char ubxTimepulse[] PROGMEM = {0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x00};
//                                           |ID         |Lenght     |TP   |res  |res        |antCableD  |rfGrDelay  |freqPeriod             |freqPeriod lock        |Pulselenghtratio       |Pulselenghtratiolock   |UserConfigDelay        |Flags                 |
const unsigned char ubxPrtConf[] PROGMEM = {0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00}; // 115200bps
//                                         |ID         |Lenght     |Port |res  |TX Ready   |mode                   |baudrate               |inPrMask   |outPrMask  |flags      |res       |

// GPS Configuration - Enable/disable specific NMEA sentences
const unsigned char ubxEnableRMC[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableGLL[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableGSA[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableGSV[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableVTG[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableZDA[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// GPS Configuration - Backup configuration to non volatile memory
// const unsigned char ubxSave[] PROGMEM = {0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x01}; // No EEPROM => Backup battery
static NMEAGPS  gps;
static gps_fix  fix_data;
static gps_fix  fix_data_prev;

// Set these values to the offset of your timezone from GMT
static const int32_t          zone_hours   = +1L; // EST
static const int32_t          zone_minutes =  0L; // usually zero
static const NeoGPS::clock_t  zone_offset  = zone_hours * NeoGPS::SECONDS_PER_HOUR + zone_minutes * NeoGPS::SECONDS_PER_MINUTE;
#define EU_DST
#if defined(EU_DST)
static const uint8_t springMonth =  3;
static const uint8_t springDate  = 31; // latest last Sunday
static const uint8_t springHour  =  1;
static const uint8_t fallMonth   = 10;
static const uint8_t fallDate    = 31; // latest last Sunday
static const uint8_t fallHour    =  1;
#define CALCULATE_DST
#endif

// Texts
const static char ERROR_PROVIDE_NUMBER[] PROGMEM = "Error : you have to provide a number !";
char *inputsLabel[] = {"DIGI_SQR_RPM", "DIGI_SQR_OPT_1", "DIGI_BRAKE", "DIGI_OPT_1", "ANA_GEAR", "ANA_THROTTLE", "ANA_OPT_1", "ANA_OPT_2"};

/**************************************************************
  #############################################
  ############## Vars definition ##############
  #############################################
**************************************************************/

// Inputs
uint32_t inDigiSqrRpm; // Digital square input (RPM)
uint32_t inDigiSqrOpt1; // Digital square input (optional)
bool inDigiBrake; // Digital boolean input (brake)
bool inDigiOpt1; // Digital boolean input (optional)
uint16_t inAnaGear; // Digital analog input (GEAR)
uint16_t inAnaThrottle; // Digital analog input (THROTTLE)
uint16_t inAnaOpt1; // Digital analog input (optional)
uint16_t inAnaOpt2; // Digital analog input (optional)
uint8_t enabledInputsBits; // Store enabled inputs (use binary values, ex : 00000011 > inAnaOpt1 and inAnaOpt2 enabled)
uint8_t tmpComp, bitShift; // Used to compare values for enabled inputs checks
char gear; // Store GEAR name (N, 1, 2, 3 ...)
uint16_t inAnaGearCalib[7]; // Calibration values for GEAR input
uint16_t inAnaThrottleMax, inAnaOpt1Max, inAnaOpt2Max; // Max values for analog inputs calibration
uint8_t gearNCheck = 0;

// SD Card
SdFat sd;
File logFile; // One line every 100ms with all detailed data
File lapFile; // One line per lap with laptime
File trackFile; // One line per track with GPS coordinates of finish line
File historyFile; // History of all sessions
char filename[32];

// GPS & Timing
bool recordTrackData, addFinishLog, isRunning = false;
char trackName[16];
uint8_t lapCounter = 0;
int16_t runMinutes, runSeconds;
uint16_t trackId, lapId;
int32_t timeCsec, timeSec, lastFlSec, lastFlCsec, lapSec, lapCsec, posCrossLat, posCrossLon, flineLat1, flineLon1, flineLat2, flineLon2; // Finish line GPS coordinates
uint32_t lastPinRead = 0, lastLCDupdate = 0, lastSdSync = 0, fixCount = 0, elapsedTime = 0;
float coordsDistance, totalDistance, tToFl; // Time To Finish Line

// Bluetooth
char c, cmdBuffer[16]; // Buffers

// Infrared temp sensor
uint8_t mlxAddresses[maxMlx]; // Store each MLX I2C addresses
double mlxValues[maxMlx]; // Store MLX values

// RPM
uint8_t rpmFlywheelTeeth;
uint8_t rpmCorrectionRatio;

// Buttons & menu
volatile boolean mcp1Interrupt = false;
uint8_t mcp1PinTriggeredId; // Which pin of the MCP1 is triggered (detect which button is pressed)
bool mcp1PinTriggeredState; // Button is pressed or released ?
uint32_t buttonPressed = 0; // Time in ms a button is pressed
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
bool showSubMenu31 = false; // bluetooth menu
uint8_t showScreenId = 0; // Which screen are we displaying (0 = homepage, 1 = navigation menu, 2 ... 254 = specific pages)

#ifdef DEBUG
uint32_t Now = 0; // used to calculate integration interval
uint32_t count = 0, sumCount = 0; // used to control display output rate
float deltat = 0.0f, sum = 0.0f; // integration interval for both filter schemes
#endif

/*
* Function prototypes
*/
void printNav(uint8_t typeId);

/**************************************************************
  #################################################
  ############## Instantiate objects ##############
  #################################################
**************************************************************/

// MLX infrared temperature sensor
MLX90614 mlx[maxMlx] = {MLX90614(firstMlxAddress), MLX90614(firstMlxAddress + 1), MLX90614(firstMlxAddress + 2), MLX90614(firstMlxAddress + 3), MLX90614(firstMlxAddress + 4), MLX90614(firstMlxAddress + 5)};

// I/O Expander
Adafruit_MCP23017 MCP1;
Adafruit_MCP23017 MCP2;

// OLED
SSD1306AsciiSpi OLED_PORT; // 128x64 LCD OLED

// LCD
//Adafruit_ILI9341 OLED_PORT = Adafruit_ILI9341(TFT_CS, TFT_DC);

// MENU
class MyRenderer : public MenuComponentRenderer {
  public:
    void render(Menu const& menu) const {
      OLED_PORT.clear();
      if (showScreenId == 1) {
        OLED_PORT.setInvertMode(true);
        if (menu.get_name() == "") {
          OLED_PORT.println("DAWA Main Menu");
        } else {
          OLED_PORT.println(menu.get_name());
        }
        OLED_PORT.setInvertMode(false);
        for (int i = 0; i < menu.get_num_components(); ++i) {
          MenuComponent const* cp_m_comp = menu.get_menu_component(i);
          if (cp_m_comp->is_current()) {
            OLED_PORT.print(">");
          } else {
            OLED_PORT.print(" ");
          }
          cp_m_comp->render(*this);
          OLED_PORT.println();
        }
        printNav(0);
      }
    }
    void render_menu_item(MenuItem const& menu_item) const {
      OLED_PORT.print(menu_item.get_name());
    }
    void render_back_menu_item(BackMenuItem const& menu_item) const {
      OLED_PORT.print(menu_item.get_name());
    }
    void render_numeric_menu_item(NumericMenuItem const& menu_item) const {
      OLED_PORT.print(menu_item.get_name());
      OLED_PORT.print(" : ");
      OLED_PORT.print(menu_item.get_formatted_value());
    }
    void render_menu(Menu const& menu) const {
      OLED_PORT.print(menu.get_name());
    }
};
MyRenderer my_renderer;

// Forward declarations
void on_menu_1_1_1_1_selected(MenuComponent* p_menu_component);
void on_menu_1_1_1_2_selected(MenuComponent* p_menu_component);
void on_menu_1_1_1_3_selected(MenuComponent* p_menu_component);
void on_menu_1_1_2_1_selected(MenuComponent* p_menu_component);
void on_menu_1_1_2_2_selected(MenuComponent* p_menu_component);
void on_menu_1_1_2_3_selected(MenuComponent* p_menu_component);
void on_menu_1_1_2_4_selected(MenuComponent* p_menu_component);
void on_menu_1_2_1_1_selected(MenuComponent* p_menu_component);
void on_menu_1_2_1_2_selected(MenuComponent* p_menu_component);
void on_menu_1_2_2_1_selected(MenuComponent* p_menu_component);
void on_menu_1_2_2_2_selected(MenuComponent* p_menu_component);
void on_menu_1_2_3_1_selected(MenuComponent* p_menu_component);
void on_menu_1_2_3_2_selected(MenuComponent* p_menu_component);
void on_menu_1_2_4_1_selected(MenuComponent* p_menu_component);
void on_menu_1_2_4_2_selected(MenuComponent* p_menu_component);
void on_menu_1_2_4_3_selected(MenuComponent* p_menu_component);
void on_menu_1_3_1_1_1_selected(MenuComponent* p_menu_component);
void on_menu_1_3_1_2_1_selected(MenuComponent* p_menu_component);
void on_menu_1_3_2_1_1_selected(MenuComponent* p_menu_component);
void on_menu_1_3_2_2_1_selected(MenuComponent* p_menu_component);
void on_menu_2_selected(MenuComponent* p_menu_component);
void on_menu_3_selected(MenuComponent* p_menu_component);
void on_menu_4_selected(MenuComponent* p_menu_component);

const String format_int(const float value) {
  return String((int) value);
}

const String format_float(const float value) {
  return String(value);
}

// Menu variables
MenuSystem ms(my_renderer);
Menu menu_1("Configuration");
Menu menu_1_1("En./dis. inputs");
Menu menu_1_1_1("Default inputs");
MenuItem menu_1_1_1_1("GEAR", &on_menu_1_1_1_1_selected);
MenuItem menu_1_1_1_2("THROTTLE", &on_menu_1_1_1_2_selected);
MenuItem menu_1_1_1_3("RPM", &on_menu_1_1_1_3_selected);
Menu menu_1_1_2("Opt. inputs");
MenuItem menu_1_1_2_1("SQR 1", &on_menu_1_1_2_1_selected);
MenuItem menu_1_1_2_2("DIGI 1", &on_menu_1_1_2_2_selected);
MenuItem menu_1_1_2_3("ANALOG 1", &on_menu_1_1_2_3_selected);
MenuItem menu_1_1_2_4("ANALOG 2", &on_menu_1_1_2_4_selected);
Menu menu_1_2("Default inputs");
Menu menu_1_2_1("MLX");
MenuItem menu_1_2_1_1("View values", &on_menu_1_2_1_1_selected);
MenuItem menu_1_2_1_2("Autodetect", &on_menu_1_2_1_2_selected);
Menu menu_1_2_2("GEAR");
MenuItem menu_1_2_2_1("View raw values", &on_menu_1_2_2_1_selected);
MenuItem menu_1_2_2_2("Calibrate", &on_menu_1_2_2_2_selected);
Menu menu_1_2_3("THROTTLE");
MenuItem menu_1_2_3_1("View raw values", &on_menu_1_2_3_1_selected);
MenuItem menu_1_2_3_2("Calibrate", &on_menu_1_2_3_2_selected);
Menu menu_1_2_4("RPM");
MenuItem menu_1_2_4_1("View values", &on_menu_1_2_4_1_selected);
NumericMenuItem menu_1_2_4_2("Flywheel teeth", &on_menu_1_2_4_2_selected, 1, 1, 50, 1, format_int);
NumericMenuItem menu_1_2_4_3("RPM ratio", &on_menu_1_2_4_3_selected, 100, 100, 200, 1, format_int);
Menu menu_1_3("Opt. inputs");
Menu menu_1_3_1("SQR 1");
MenuItem menu_1_3_1_1("Calibrate", &on_menu_1_3_1_1_selected);
Menu menu_1_3_2("DIGI 1");
MenuItem menu_1_3_2_1("Calibrate", &on_menu_1_3_2_1_selected);
Menu menu_1_3_3("ANALOG 1");
MenuItem menu_1_3_3_1("Calibrate", &on_menu_1_3_3_1_selected);
Menu menu_1_3_4("ANALOG 2");
MenuItem menu_1_3_4_1("Calibrate", &on_menu_1_3_4_1_selected);
Menu menu_2("Tracks");
MenuItem menu_2_1("Select track", &on_menu_2_1_selected);
MenuItem menu_3("About", &on_menu_3_selected);
MenuItem menu_4("Quit", &on_menu_4_selected);

/**************************************************************
  #################################################
  ############## Initialisation ###################
  #################################################
**************************************************************/

void setup() {
  /**************************************************************
    I2C bus init
  **************************************************************/
  Wire.begin();
  delay(500);

#ifdef DEBUG
  while (!DEBUG_PORT);
  DEBUG_PORT.println(F("DEBUG is enabled"));
#endif

  /**************************************************************
    Menu Init
  **************************************************************/
  ms.get_root_menu().add_menu(&menu_1);
  menu_1.add_menu(&menu_1_1);  
    menu_1_1.add_menu(&menu_1_1_1);
      menu_1_1_1.add_item(&menu_1_1_1_1);
      menu_1_1_1.add_item(&menu_1_1_1_2);
      menu_1_1_1.add_item(&menu_1_1_1_3);
    menu_1_1.add_menu(&menu_1_1_2);
      menu_1_1_2.add_item(&menu_1_1_2_1);
      menu_1_1_2.add_item(&menu_1_1_2_2);
      menu_1_1_2.add_item(&menu_1_1_2_3);
      menu_1_1_2.add_item(&menu_1_1_2_4);
  menu_1.add_menu(&menu_1_2); 
    menu_1_2.add_menu(&menu_1_2_1);
      menu_1_2_1.add_item(&menu_1_2_1_1);
      menu_1_2_1.add_item(&menu_1_2_1_2);
    menu_1_2.add_menu(&menu_1_2_2);
      menu_1_2_2.add_item(&menu_1_2_2_1);
      menu_1_2_2.add_item(&menu_1_2_2_2);
    menu_1_2.add_menu(&menu_1_2_3);
      menu_1_2_3.add_item(&menu_1_2_3_1);
      menu_1_2_3.add_item(&menu_1_2_3_2);
    menu_1_2.add_menu(&menu_1_2_4);
      menu_1_2_4.add_item(&menu_1_2_4_1);
      menu_1_2_4.add_item(&menu_1_2_4_2);
      menu_1_2_4.add_item(&menu_1_2_4_3);   
  menu_1.add_menu(&menu_1_3);
    menu_1_3.add_menu(&menu_1_3_1);
        menu_1_3_1.add_item(&menu_1_3_1_1);
     menu_1_3.add_menu(&menu_1_3_2);   
        menu_1_3_2.add_item(&menu_1_3_2_1);
      menu_1_3.add_menu(&menu_1_3_3); 
        menu_1_3_3.add_item(&menu_1_3_3_1);
      menu_1_3.add_menu(&menu_1_3_4); 
        menu_1_3_4.add_item(&menu_1_3_4_1);
  ms.get_root_menu().add_menu(&menu_2);
  menu_2.add_item(&menu_2_1);  
  ms.get_root_menu().add_item(&menu_3);
  ms.get_root_menu().add_item(&menu_4);

  /**************************************************************
    Init I/O pins
  **************************************************************/
  pinMode(inDigiBrakePin, INPUT); // Digital boolean input (brake) (0v/12v)
  pinMode(inDigiOpt1Pin, INPUT); // Digital boolean input (optional) (0v/12v)
  pinMode(inAnaGearPin, INPUT); // Digital analog input (GEAR) (analogic from 0v to 5v)
  pinMode(inAnaThrottlePin, INPUT); // Digital analog input (THROTTLE) (analogic from 0v to 5v)
  pinMode(inAnaOpt1Pin, INPUT); // Digital analog input (optional)
  pinMode(inAnaOpt2Pin, INPUT); // Digital analog input (optional)
  pinMode(sdCsPin, OUTPUT); // Chip Select for SDCARD on SPI bus

  /**************************************************************
    I/O Expander 1 (Output = 4 leds, Input = 4 buttons)
  **************************************************************/
  MCP1.begin();
  MCP1.pinMode(mcp1Led1, OUTPUT);
  MCP1.pinMode(mcp1Led2, OUTPUT);
  MCP1.pinMode(mcp1Led3, OUTPUT);
  MCP1.pinMode(mcp1Led4, OUTPUT);
  MCP1.setupInterrupts(false, false, LOW);
  MCP1.pinMode(mcp1But1, INPUT);
  MCP1.pinMode(mcp1But2, INPUT);
  MCP1.pinMode(mcp1But3, INPUT);
  MCP1.pinMode(mcp1But4, INPUT);
  MCP1.pullUp(mcp1But1, HIGH); // Enable internal pullup
  MCP1.pullUp(mcp1But2, HIGH); // Enable internal pullup
  MCP1.pullUp(mcp1But3, HIGH); // Enable internal pullup
  MCP1.pullUp(mcp1But4, HIGH); // Enable internal pullup
  MCP1.setupInterruptPin(mcp1But1, CHANGE); // Interrupt on state change (2 interrupts : press and release button)
  MCP1.setupInterruptPin(mcp1But2, CHANGE); // Interrupt on state change (2 interrupts : press and release button)
  MCP1.setupInterruptPin(mcp1But3, CHANGE); // Interrupt on state change (2 interrupts : press and release button)
  MCP1.setupInterruptPin(mcp1But4, CHANGE); // Interrupt on state change (2 interrupts : press and release button)
  attachInterrupt(mcp1IntPin, callbackMcp1Interrupt, FALLING); // Interrupt triggered on port D8 when any button pressed or released
  MCP1.digitalWrite(mcp1Led1, LOW); // Power on led1, power off after successfull init
  MCP1.digitalWrite(mcp1Led2, LOW); // Power on led2, power off after successfull init
  MCP1.digitalWrite(mcp1Led3, LOW); // Power on led3, power off after successfull init
  MCP1.digitalWrite(mcp1Led4, LOW); // Power on led4, power off after successfull init

  /**************************************************************
    I/O Expander 2 (Output = 5v power for 6 devices like MLX sensors plugged on hub)
  **************************************************************/
  MCP2.begin(4);
  MCP2.pinMode(0, OUTPUT);
  MCP2.pinMode(1, OUTPUT);
  MCP2.pinMode(2, OUTPUT);
  MCP2.pinMode(3, OUTPUT);
  MCP2.pinMode(4, OUTPUT);
  MCP2.pinMode(5, OUTPUT);
  MCP2.pinMode(6, OUTPUT);
  MCP2.pinMode(7, OUTPUT);
  MCP2.digitalWrite(0, HIGH);
  MCP2.digitalWrite(1, HIGH);
  MCP2.digitalWrite(2, HIGH);
  MCP2.digitalWrite(3, HIGH);
  MCP2.digitalWrite(4, HIGH);
  MCP2.digitalWrite(5, HIGH);
  MCP2.digitalWrite(6, HIGH);
  MCP2.digitalWrite(7, HIGH);

  /**************************************************************
    Bluetooth Init (Serial2)
  **************************************************************/
  OLED_PORT.print(F("BLUETOOTH:"));                                               ///////////// !!!!!!!!!
  BLUETOOTH_PORT.begin(9600);
  pinPeripheral(6, PIO_SERCOM); // Pin D6 for RX
  pinPeripheral(7, PIO_SERCOM); // Pin D7 for TX
  // ADD A TRUE BLUETOOTH TEST HERE !!
  OLED_PORT.println(F("         OK"));
  BLUETOOTH_PORT.println(F("D.A.W.A. Initializing ..."));
  BLUETOOTH_PORT.println(F("BLUETOOTH : OK"));
#ifdef DEBUG
  DEBUG_PORT.println(F("D.A.W.A. Initializing ..."));
  DEBUG_PORT.println(F("BLUETOOTH : OK"));
#endif

  /**************************************************************
    OLED Screen Init (SPI)
  **************************************************************/
  OLED_PORT.begin(&SH1106_128x64, OLED_CS, OLED_DC); OLED_PORT.setFont(System5x7);
  delay(1000); // LCD init delay
  OLED_PORT.clear(); // Clear screen
  OLED_PORT.setCursor(0, 0); // Set cursor upper-left
  OLED_PORT.println(F("DAWA Init ..."));
#ifdef DEBUG
  DEBUG_PORT.println(F("OLED : OK"));
#endif

  /**************************************************************
    SD card Init (SPI)
  **************************************************************/
  OLED_PORT.print(F("SD:"));
  BLUETOOTH_PORT.print(F("SD : "));
  if (!sd.begin(sdCsPin, SD_SCK_MHZ(50))) {
    OLED_PORT.println(F("            FAILED"));
    BLUETOOTH_PORT.println(F("FAILED"));
#ifdef DEBUG
    DEBUG_PORT.println(F("SD : FAILED"));
#endif
    initError();
  } else {
    OLED_PORT.println(F("                OK"));
    BLUETOOTH_PORT.println(F("OK"));
#ifdef DEBUG
    DEBUG_PORT.println(F("SD : OK"));
#endif
  }

  /**************************************************************
    EEPROM Init (I2C)
  **************************************************************/
  OLED_PORT.print(F("EEPROM:"));
  BLUETOOTH_PORT.print(F("EEPROM : "));
  if (!eep.begin(twiClock100kHz) == 0) {
    OLED_PORT.println(F("        FAILED"));
    BLUETOOTH_PORT.println(F("FAILED"));
#ifdef DEBUG
    DEBUG_PORT.println(F("EEPROM : FAILED"));
#endif
    initError();
  } else {
    // Read RPM correction ratio
    EEPROM_readAnything(28, rpmCorrectionRatio) == sizeof(rpmCorrectionRatio);

    // Read RPM flywheel teeth
    EEPROM_readAnything(29, rpmFlywheelTeeth) == sizeof(rpmFlywheelTeeth);

    // Read enabled inputs > 8 bits
    EEPROM_readAnything(30, enabledInputsBits) == sizeof(enabledInputsBits);

    // Read max values for analogic inputs (throttle, anaopt1, anaopt2) > 3x16 bits
    EEPROM_readAnything(31, inAnaThrottleMax) == sizeof(inAnaThrottleMax);
    EEPROM_readAnything(33, inAnaOpt1Max) == sizeof(inAnaOpt1Max);
    EEPROM_readAnything(35, inAnaOpt2Max) == sizeof(inAnaOpt2Max);

    // Read GEAR calibration data > 16 bits
    EEPROM_readAnything(37, inAnaGearCalib) == sizeof(inAnaGearCalib);

    // Read saved MLX I2C Address (infrared temp sensors) > 6x8 bits
    EEPROM_readAnything(50, mlxAddresses) == sizeof(mlxAddresses);
    for (uint8_t i = 0; i < maxMlx; i++) {
      if (mlxAddresses[i] != 0x00) {
        mlx[i].begin();
      }
    }
    OLED_PORT.println(F("            OK"));
    BLUETOOTH_PORT.println(F("OK"));
#ifdef DEBUG
    DEBUG_PORT.println(F("EEPROM : OK"));
#endif
  }

  /**************************************************************
    GPS Init (Serial5 - Default on Arduino m0)
  **************************************************************/
  OLED_PORT.print(F("GPS:"));
  BLUETOOTH_PORT.print(F("GPS : "));
  GPS_PORT.begin(9600); // Start the UART @9600bps for the GPS device (default speed)
  sendUBX(ubxPrtConf, sizeof(ubxPrtConf)); // Set UART speed to 115200bps (Warning : @9600bps > ~5sec delay on GPS data)
  delay(100);
  GPS_PORT.end();
  GPS_PORT.begin(115200); // Start the UART @115200bps
  sendUBX(ubxRate10Hz, sizeof(ubxRate10Hz)); // Set refresh rate to 10Hz
  sendUBX(ubxTimepulse, sizeof(ubxTimepulse)); // Set timepulse output ON
  sendUBX(ubxEnableRMC, sizeof(ubxEnableRMC)); // Enable RMC trames, disable all others
  sendUBX(ubxDisableGLL, sizeof(ubxDisableGLL));
  sendUBX(ubxDisableGSA, sizeof(ubxDisableGSA));
  sendUBX(ubxDisableGSV, sizeof(ubxDisableGSV));
  sendUBX(ubxDisableVTG, sizeof(ubxDisableVTG));
  sendUBX(ubxDisableZDA, sizeof(ubxDisableZDA));
  while (fix_data.status <= 0) { // Test valid fix
    if (gps.available(GPS_PORT)) {
      fix_data = gps.read();
    }
  }
  OLED_PORT.println(F("               OK"));
  BLUETOOTH_PORT.println(F("OK"));
#ifdef DEBUG
  DEBUG_PORT.println(F("GPS : OK"));
#endif

  /**************************************************************
    Power off leds as soon as we get a valid gps signal (ready to go)
  **************************************************************/
  if (fix_data.valid.location) { // We need GPS fix before starting
    MCP1.digitalWrite(mcp1Led1, HIGH);
    MCP1.digitalWrite(mcp1Led2, HIGH);
    MCP1.digitalWrite(mcp1Led3, HIGH);
    MCP1.digitalWrite(mcp1Led4, HIGH);
  }

  /**************************************************************
    2 counters (TCC0 & TCC1)
    for RPM values and another optionnal square signal

    m0 pinout : https://github.com/arduino/ArduinoCore-samd/blob/master/variants/arduino_mzero/variant.cpp
    Big thanks to : https://forum.arduino.cc/index.php?topic=396804.45
  **************************************************************/
  REG_PM_APBCMASK |=  PM_APBCMASK_EVSYS;    // Switch on the event system peripheral
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;     // Enable TCC0 Bus clock (Timer counter control clock)
  PM->APBCMASK.reg |= PM_APBCMASK_TCC1;     // Enable TCC1 Bus clock (Timer counter control clock)

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK0 |     // .... on GCLK0...
                     GCLK_CLKCTRL_ID_EIC;         // ... to feed the GCLK0 to EIC peripheral
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK0 |     // ....on GCLK0...
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // ... to feed the GCLK5 to TCC0 and TCC1 peripheral
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  PORT->Group[PORTA].PMUX[19 >> 1].reg |= PORT_PMUX_PMUXO_A;     // Connect PA19 (pin 12 on m0) to peripheral A (EXTINT[3])
  PORT->Group[PORTA].PINCFG[19].reg |= PORT_PINCFG_PMUXEN;       // Enable pin peripheral multiplexation

  PORT->Group[PORTA].PMUX[18 >> 1].reg |= PORT_PMUX_PMUXO_A;     // Connect PA18 (pin 10 on m0) to peripheral A (EXTINT[3])
  PORT->Group[PORTA].PINCFG[18].reg |= PORT_PINCFG_PMUXEN;       // Enable pin peripheral multiplexation

  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO3;         // Enable event from pin on external interrupt 3 (EXTINT03)
  REG_EIC_CONFIG0 |= EIC_CONFIG_SENSE3_RISE;      // Set event on rising edge of signal

  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO2;         // Enable event from pin on external interrupt 2 (EXTINT02)
  REG_EIC_CONFIG0 |= EIC_CONFIG_SENSE2_RISE;      // Set event on rising edge of signal

  REG_EIC_CTRL |= EIC_CTRL_ENABLE;                // Enable EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  // EVSYS Configuration
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |                                // Attach the event user (receiver) to channel n=0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_0);              // Set the event user (receiver) as timer TCC0, event 1

  REG_EVSYS_USER = EVSYS_USER_CHANNEL(2) |                                // Attach the event user (receiver) to channel n=0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_0);              // Set the event user (receiver) as timer TCC0, event 1

  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event output edge detection
                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_3) |    // Set event generator (sender) as external interrupt 3
                      EVSYS_CHANNEL_CHANNEL(0);                           // Attach the generator (sender) to channel 0

  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event output edge detection
                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_2) |    // Set event generator (sender) as external interrupt 3
                      EVSYS_CHANNEL_CHANNEL(1);                           // Attach the generator (sender) to channel 0


  // TCC0 & TCC1 Configuration
  REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;            // Disable TCC0 peripheral
  REG_TCC1_CTRLA &= ~TCC_CTRLA_ENABLE;            // Disable TCC1 peripheral

  REG_TCC0_CTRLBCLR |= TCC_CTRLBCLR_DIR;          // Clear DIR bit to count up
  while (TCC0->SYNCBUSY.bit.CTRLB);               // Wait for (write) synchronization
  REG_TCC1_CTRLBCLR |= TCC_CTRLBCLR_DIR;          // Clear DIR bit to count up
  while (TCC1->SYNCBUSY.bit.CTRLB);               // Wait for (write) synchronization

  REG_TCC0_EVCTRL |= TCC_EVCTRL_TCEI0 |           // Enable the TCC event 0 input
                     TCC_EVCTRL_EVACT0_COUNT;     // Set up TCC timer/counter to count on event

  REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE;             // Enable TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  REG_TCC1_EVCTRL |= TCC_EVCTRL_TCEI0 |           // Enable the TCC event 0 input
                     TCC_EVCTRL_EVACT0_COUNT;     // Set up TCC timer/counter to count on event

  REG_TCC1_CTRLA |= TCC_CTRLA_ENABLE;             // Enable TCC0
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  // End Init
  OLED_PORT.print(F("              READY !"));
  BLUETOOTH_PORT.println(F("READY !"));
#ifdef DEBUG
  DEBUG_PORT.println(F("READY !"));
#endif
  delay(1000);
  OLED_PORT.clear();
}

/**************************************************************
  MAIN LOOP
**************************************************************/
void loop() {
#ifdef DEBUG
  // Here we calculate average update rate of main loop
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f);
  lastUpdate = Now;
  sum += deltat;
  sumCount++;
#endif

  /**************************************************************
    MCP1/Buttons interrupt
  **************************************************************/
  if (mcp1Interrupt) {
    handleMcp1Interrupt();
  }

  /**************************************************************
    Action on start/stop button
  **************************************************************/
  if (buttonPressed > 0 && mcp1PinTriggeredId == 3 && mcp1PinTriggeredState == 1 && showScreenId == 0) { // Call menu (short press on button 3, first left)
    buttonPressed = 0;
    showScreenId = 1;
    ms.display();
  }

  if (buttonPressed > 0 && mcp1PinTriggeredId == 1 && mcp1PinTriggeredState == 1 && showScreenId > 1) { // Back to menu (short press on button "BACK")
    buttonPressed = 0;
    showScreenId = 1;
    ms.display();
  }


  /*
    if (buttonPressed > 500 && mcp1PinTriggeredId == 3 && mcp1PinTriggeredState == 0) { // long press on power button (500ms)
      buttonPressed = 0;
      SdFile::dateTimeCallback(dateTimeSd);
      if (isRunning) { // If laptimer is running ...
        **************************************************************
          ******************* STOP RECORDING HERE *******************
        **************************************************************
        isRunning = false; // ... we stop recording
        logFile.close(); // Close file on SDcard
        lapFile.close(); // Close file on SDcard
        OLED_PORT.clear();
        BLUETOOTH_PORT.println(F("[Session stopped !]"));
      } else { // If laptimer is NOT running ...
        if (fix_data.valid.location) { // We need GPS fix before starting
          **************************************************************
            ******************* START RECORDING HERE *******************
          **************************************************************
          OLED_PORT.clear();
          OLED_PORT.println(F("Running ..."));
          BLUETOOTH_PORT.println(F("[Start new session !]"));

          **************************************************************
            Select nearest track from the file "TRACKS.csv" on sdcard
          **************************************************************
          recordTrackData = false;
          trackFile = sd.open("TRACKS.csv", FILE_READ);
          if (trackFile) {
            while (trackFile.available()) {
              csvReadUint16(&trackFile, &trackId, csvDelim);
              csvReadText(&trackFile, trackName, sizeof(trackName), csvDelim); // One line per track : "1;CAROLE;489799930;25224350;489800230;25226330" (<trackname>;<startline_a_lat>;<startline_a_lon>;<startline_b_lat>;<startline_b_lon>)
              csvReadInt32(&trackFile, &flineLat1, csvDelim);                  // Points A & B should be at the left and at the right of the finishline (a few meters)
              csvReadInt32(&trackFile, &flineLon1, csvDelim);
              csvReadInt32(&trackFile, &flineLat2, csvDelim);
              csvReadInt32(&trackFile, &flineLon2, csvDelim);

              // Calculate distance between 2 GPS coordinates
              coordsDistance = gpsDistance(fix_data.latitudeL(), fix_data.longitudeL(), flineLat1, flineLon1) / 1000;

              // If you are on a known track, then we select it
              if (coordsDistance <= maxTrackDistance) {
                recordTrackData = true;
                OLED_PORT.print(trackName);
                OLED_PORT.print(F(" ("));
                OLED_PORT.print(coordsDistance, 1);
                OLED_PORT.println(F("km)"));
                BLUETOOTH_PORT.print(F("Track selected : "));
                BLUETOOTH_PORT.print(trackName);
                BLUETOOTH_PORT.print(F(" ("));
                BLUETOOTH_PORT.print(coordsDistance, 1);
                BLUETOOTH_PORT.println(F("km)"));
                break; // Break here so last read values are the good ones !
              }
            }
            trackFile.close();
          }
          if (recordTrackData == false) {
            OLED_PORT.println(F("No track file !"));
            BLUETOOTH_PORT.println(F("No track file !"));
          }

          **************************************************************
            Create new datafile : history file (append)
          **************************************************************
          if (recordTrackData == true) { // No track = no history !
            sprintf(filename, "HISTORY.csv");
            if (historyFile.open(filename, O_CREAT | O_APPEND | O_WRITE)) {
              historyFile.print(fix_data.dateTime);
              historyFile.print(F(";"));
              historyFile.print(fix_data.dateTime.year);
              if (fix_data.dateTime.month < 10) historyFile.print(F("0")); // Leading zeros
              historyFile.print(fix_data.dateTime.month);
              if (fix_data.dateTime.date < 10) historyFile.print(F("0")); // Leading zeros
              historyFile.print(fix_data.dateTime.date);
              historyFile.print(F("-"));
              if (fix_data.dateTime.hours < 10) historyFile.print(F("0")); // Leading zeros
              historyFile.print(fix_data.dateTime.hours);
              if (fix_data.dateTime.minutes < 10) historyFile.print(F("0")); // Leading zeros
              historyFile.print(fix_data.dateTime.minutes);
              if (fix_data.dateTime.seconds < 10) historyFile.print(F("0")); // Leading zeros
              historyFile.print(fix_data.dateTime.seconds);
              historyFile.print(F(";"));
              historyFile.println(trackId);
              historyFile.close(); // Close file on SDcard
              BLUETOOTH_PORT.print(F("Append file : "));
              BLUETOOTH_PORT.println(filename);
            } else {
              initError();
            }
          }

          **************************************************************
            Create new datafile : log file (create new)
          **************************************************************
          sprintf(filename, "%02u%02u%02u-%02u%02u%02u.csv", fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);
          if (logFile.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
            // Time, distance and lap (always printed)
            logFile.print(F("Time;Distance;Lap;"));

            // Digital & analog inputs (printed if enabled)
            bitShift = B00000001;
            for (uint8_t i = 0; i < 8; i++) {
              tmpComp = bitShift & enabledInputsBits;
              if (tmpComp == bitShift) {
                logFile.print(inputsLabel[i]);
                logFile.print(F(";"));
              }
              bitShift = bitShift << 1;
            }

            // KPH (GPS), Orientation, ambiant temperature (always printed)
            logFile.print(F("KPH;Heading;Roll;Temp;"));

            // Infrared temperature (printed if enabled)
            for (uint8_t i = 0; i < maxMlx; i++) {
              if (mlxAddresses[i] != 0x00) {
                logFile.print(F("IRTemp"));
                logFile.print(i);
                logFile.print(F(";"));
              }
            }

            // Latitude & longitude (always printed)
            logFile.println(F("Latitude;Longitude"));
            BLUETOOTH_PORT.print(F("Create new file : "));
            BLUETOOTH_PORT.println(filename);
          } else {
            initError();
          }

          **************************************************************
            Create new datafile : laptime file (create new)
          **************************************************************
          sprintf(filename, "%02u%02u%02u-%02u%02u%02u-LAPTIMES.csv", fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);
          if (lapFile.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
            BLUETOOTH_PORT.print(F("Create new file : "));
            BLUETOOTH_PORT.println(filename);
          } else {
            initError();
          }

          **************************************************************
            Init some vars
          **************************************************************
          isRunning = true; // ... we start recording
          lapCounter = 0; // Lap 0 (we start from paddocks)
          totalDistance = 0;
          addFinishLog = false;
        }
      }
    }*/

  /**************************************************************
    Get GPS frames through serial port (RX/TX)
    This is CRITICAL, data could be sent at any moment by the GPS so main loop should be executed in a minimum of time
    More information : https://github.com/SlashDevin/NeoGPS/blob/master/extras/doc/Troubleshooting.md#quiet-time-interval
  **************************************************************/
  if (gps.available(GPS_PORT)) {
    /**************************************************************
      Read last available GPS data
    **************************************************************/
    fix_data_prev = fix_data; // Memorize previous values before next GPS fix
    fix_data = gps.read(); // Get GPS data
    adjustTime( fix_data.dateTime );
    fixCount++;

    /**************************************************************
      Read INPUTS
    **************************************************************/
    elapsedTime = millis() - lastPinRead; // Used to have precise measures on RPM and SPEED
    lastPinRead = millis(); // Reset last pin read
    bitShift = B00000001;
    for (uint8_t i = 0; i < 8; i++) {
      tmpComp = bitShift & enabledInputsBits;
      if (tmpComp == bitShift) {
        switch (i) {
          // bit 0 : inDigiSqrRpm;
          case 0:
            REG_TCC1_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC;  // Trigger a read synchronization on the COUNT register
            while (TCC1->SYNCBUSY.bit.CTRLB);               // Wait for the CTRLB register write synchronization
            while (TCC1->SYNCBUSY.bit.COUNT);               // Wait for the COUNT register read sychronization
            inDigiSqrRpm = REG_TCC1_COUNT;                       // Read TCNT1 register (timer1 counter)
            REG_TCC1_COUNT = 0x0000;                        // Clear timer's COUNT value
            while (TCC1->SYNCBUSY.bit.COUNT);               // Wait for synchronization
            inDigiSqrRpm = inDigiSqrRpm * rpmCorrectionRatio * 600 / elapsedTime / rpmFlywheelTeeth; // Ratio between pulse and rpm (22 > flywheel has 22 teeth ### 600 > with check every 100ms, RPM is by minute) ### rpmCorrectionRatio > *(100+corr) / elapsedTime) > if we read counter @101ms or 102ms values should be adjusted
            break;
          // bit 1 : inDigiSqrOpt1;
          case 1:
            REG_TCC0_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC;  // Trigger a read synchronization on the COUNT register
            while (TCC0->SYNCBUSY.bit.CTRLB);               // Wait for the CTRLB register write synchronization
            while (TCC0->SYNCBUSY.bit.COUNT);               // Wait for the COUNT register read sychronization
            inDigiSqrOpt1 = REG_TCC0_COUNT;                     // Read TCNT0 register (timer0 counter)
            REG_TCC0_COUNT = 0x0000;                        // Clear timer's COUNT value
            while (TCC0->SYNCBUSY.bit.COUNT);               // Wait for synchronization
            inDigiSqrOpt1 = inDigiSqrOpt1 * 100 / elapsedTime; // Ratio is set to 1 so no maths ### (* 100 / elapsedTime) > if we read counter @99ms, 101ms or 102ms values should be adjusted
            break;
          // bit 2 : inDigiBrake;
          case 2:
            inDigiBrake = digitalRead(inDigiBrakePin); // Read "inDigiBrakePin" (pin is plugged on the "+" of the stop light)
            break;
          // bit 3 : inDigiOpt1;
          case 3:
            inDigiOpt1 = digitalRead(inDigiOpt1Pin); // Read "inDigiOpt1Pin"
            break;
          // bit 4 : inAnaGear;
          case 4:
            inAnaGear = constrain(analogRead(inAnaGearPin), 0, 1024);
            if (inAnaGear <= inAnaGearCalib[1] + gearOffset) {
              gear = '1';
            }
            if (inAnaGear > inAnaGearCalib[2] - gearOffset && inAnaGear <= inAnaGearCalib[2] + gearOffset) {
              gear = '2';
            }
            if (inAnaGear > inAnaGearCalib[3] - gearOffset && inAnaGear <= inAnaGearCalib[3] + gearOffset) {
              gear = '3';
            }
            if (inAnaGear > inAnaGearCalib[4] - gearOffset && inAnaGear <= inAnaGearCalib[4] + gearOffset) {
              gear = '4';
            }
            if (inAnaGear > inAnaGearCalib[5] - gearOffset && inAnaGear <= inAnaGearCalib[5] + gearOffset) {
              gear = '5';
            }
            if (inAnaGear > inAnaGearCalib[6] - gearOffset && inAnaGear <= inAnaGearCalib[6] + gearOffset) {
              gear = '6';
            }
            if (inAnaGear > inAnaGearCalib[0] - gearOffset && inAnaGear <= inAnaGearCalib[0] + gearOffset) {
              if (gearNCheck > 3) { // We test 3 times to prevent displaying "N" between 2 gears
                gear = 'N';
              } else {
                gearNCheck++;
              }
            } else {
              gearNCheck = 0;
            }
            break;
          // bit 5 : inAnaThrottle;
          case 5:
            inAnaThrottle = constrain(map(analogRead(inAnaThrottlePin), 0, inAnaThrottleMax, 0, 100), 0, 100);
            break;
          // bit 6 : inAnaOpt1;
          case 6:
            inAnaOpt1 = constrain(map(analogRead(inAnaOpt1Pin), 0, inAnaOpt1Max, 0, 100), 0, 100);
            break;
          // bit 7 : inAnaOpt2;
          case 7:
            inAnaOpt2 = constrain(map(analogRead(inAnaOpt2Pin), 0, inAnaOpt2Max, 0, 100), 0, 100);
            break;
        }
      }
      bitShift = bitShift << 1;
    }

    /**************************************************************
      Sync files on SDcard every 300 fixes (300x100ms = 30sec) to avoid dataloss on power failure
    **************************************************************/
    if (isRunning && (fixCount - lastSdSync >= 300)) {
      lastSdSync = fixCount;
      SdFile::dateTimeCallback(dateTimeSd);
      logFile.sync();
      lapFile.sync();
    }

    /**************************************************************
      1 second loop (every 10 fixes > 10x100ms = 1sec), could be used for :
      - Displaying on OLED screen
      - Get values with a low refresh rate like ambiant temperature
      - ...
    **************************************************************/
    if (fixCount - lastLCDupdate >= 10) {
      lastLCDupdate = fixCount;

#ifdef DEBUG
      DEBUG_PORT.print("rate = ");
      DEBUG_PORT.print((float)sumCount / sum, 2);
      DEBUG_PORT.println(" Hz");
      sumCount = 0;
      sum = 0;
#endif

      /**************************************************************
        Read and store MLX temperature in array
      **************************************************************/
      for (uint8_t i = 0; i < maxMlx; i++) {
        if (mlxAddresses[i] != 0x00) {
          mlxValues[i] = mlx[i].readTemp(MLX90614::MLX90614_SRC01, MLX90614::MLX90614_TC);
          /*if(mlx[i].rwError) {
            BLUETOOTH_PORT.print(i);
            BLUETOOTH_PORT.println(":Err MLX");
            }*/
        }
      }

      /**************************************************************
        Read ambiant temperature
      **************************************************************/
      //temperature = bno.getTemp();

      /**************************************************************
        Read BNO calibration state
      **************************************************************/
      //bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);

      if (isRunning) {
        /**************************************************************
          Print data on OLED screen when laptimer IS running - NOT TESTED
        **************************************************************/
        // No other data is printed on LCD because it can be time consumming and we'll loose some GPS frames
      } else {
        /**************************************************************
          Print data on OLED screen when laptimer IS NOT running
        **************************************************************/
        showData(showScreenId); // 0 = Home, 1 = menu navigation, 2 to 254 = menu actions
        //printData1(fix_data, OLED_PORT); // Select one of the 2 options to print different data on OLED screen
        //printData2(fix_data, OLED_PORT)
      }
    }

    /**************************************************************
      If laptimer is running :
      - Calculate distances, laptime with GPS data
      - Log data on SDcard

      If laptimer is NOT running :
      - Check bluetooth comands
    **************************************************************/
    if (isRunning) {
      // Calculate total distance (for SeriousRacing)
      coordsDistance = gpsDistance(fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), fix_data.latitudeL(), fix_data.longitudeL());
      totalDistance += coordsDistance;

      // Check if we pass the finishline (2x2 coordinates for finish line points + 2x2 coordinates for last position + position now)
      if (recordTrackData == true) {
        if (segIntersect(fix_data.latitudeL(), fix_data.longitudeL(), fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), flineLat1, flineLon1, flineLat2, flineLon2, posCrossLat, posCrossLon)) { // *********** TEST ONLY *********** Add >  || inAnaThrottle == 100
          // Calculate Time To Finish Line (from last know position by GPS just before crossing the finish line / format : sss.ms) ### tToFl = (distance between previous position and finish line (Ex : 0.00112km) / distance between previous position and actual position (Ex : 0.00254km)) * (time of actual fix - time of previous fix)
          tToFl = (gpsDistance(fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), posCrossLat, posCrossLon) / gpsDistance(fix_data.latitudeL(), fix_data.longitudeL(), fix_data_prev.latitudeL(), fix_data_prev.longitudeL())) * ((fix_data.dateTime.hours * 3600 + fix_data.dateTime.minutes * 60 + fix_data.dateTime.seconds + fix_data.dateTime_cs / 100.00) - (fix_data_prev.dateTime.hours * 3600 + fix_data_prev.dateTime.minutes * 60 + fix_data_prev.dateTime.seconds + fix_data_prev.dateTime_cs / 100.00));
          //tToFl = random(1, 100) / 100.00; // *********** TEST ONLY ***********

          // Add "Time to finish line (tToFl)" to the last known Epoch Time (fix_data_prev)
          timeAdd(tToFl, fix_data_prev.dateTime, fix_data_prev.dateTime_cs, timeSec, timeCsec);

          /*BLUETOOTH_PORT.println(F("TEST :");
            BLUETOOTH_PORT.println(tToFl);
            BLUETOOTH_PORT.println(fix_data_prev.dateTime);
            BLUETOOTH_PORT.println(fix_data_prev.dateTime_cs);
            BLUETOOTH_PORT.println(timeSec);
            BLUETOOTH_PORT.println(timeCsec);*/


          // Calculate total laptime (substract previous finish laptime to actual laptime)
          if (lapCounter > 0) { // Get first laptime at the end of the lap 1 (lapCounter = 1 / We start from the paddocks)
            timeSubstract(timeSec, timeCsec, lastFlSec, lastFlCsec, lapSec, lapCsec);

            /*BLUETOOTH_PORT.println(lastFlSec);
              BLUETOOTH_PORT.println(lastFlCsec);
              BLUETOOTH_PORT.println(lapSec);
              BLUETOOTH_PORT.println(lapCsec);*/

            runMinutes = lapSec / 60;
            runSeconds = lapSec % 60;

            lapFile.print(lapCounter);
            lapFile.print(F(";"));
            lapFile.print(runMinutes); // Store laptime mm:sss:ms (human readable)
            lapFile.print(F(":"));
            if (runSeconds < 10) lapFile.print(F("0")); // Leading zeros (remember "runSeconds" is an integer !!)
            lapFile.print(runSeconds);
            lapFile.print(F(":"));
            if (lapCsec < 10) lapFile.print(F("0")); // Leading zeros (remember "lapCsec" is an integer !!)
            lapFile.print(lapCsec);
            lapFile.print(F(";")); // Store laptime sss.ms (enable float comparaison for best lap or other calculations)
            lapFile.print(lapSec);
            lapFile.print(F("."));
            lapFile.println(lapCsec);

            OLED_PORT.print(lapCounter);
            OLED_PORT.print(F(" : "));
            OLED_PORT.print(runMinutes);
            OLED_PORT.print(F(":"));
            if (runSeconds < 10) OLED_PORT.print(F("0")); // Leading zeros (remember "runSeconds" is an integer !!)
            OLED_PORT.print(runSeconds);
            OLED_PORT.print(F("."));
            if (lapCsec < 10) OLED_PORT.print(F("0")); // Leading zeros (remember "timeCsec" is an integer !!)
            OLED_PORT.println(lapCsec);
          }

          // Store timestamp (sec+ms) at the finish line to calculate next lap time
          lastFlSec = timeSec;
          lastFlCsec = timeCsec;

          // Write the finish log line + inc lapCounter
          addFinishLog = true;
          lapCounter++;
        } else {
          addFinishLog = false;
        }
      }

      /**************************************************************
        Write all data to file on SD card (IMU, GPS, inAnaThrottle, gear, rpm, temperature sensors)
      **************************************************************/
      if (addFinishLog == true) {
        logFile.print(timeSec);
        logFile.print(F("."));
        if (timeCsec < 10) logFile.print(F("0")); // Leading zeros (remember "timeCsec" is an integer !!)
        logFile.print(timeCsec);
      } else {
        logFile.print(fix_data.dateTime);
        logFile.print(F("."));
        if (fix_data.dateTime_cs < 10) logFile.print(F("0")); // Leading zeros (remember "fix_data.dateTime_cs" is an integer !!)
        logFile.print(fix_data.dateTime_cs);
      }
      logFile.print(F(";"));
      logFile.print(totalDistance, 3);
      logFile.print(F(";"));
      logFile.print(lapCounter);
      logFile.print(F(";"));
      bitShift = B00000001;
      for (uint8_t i = 0; i < 8; i++) {
        tmpComp = bitShift & enabledInputsBits;
        if (tmpComp == bitShift) {
          switch (i) {
            // bit 0 : inDigiSqrRpm;
            case 0:
              logFile.print(inDigiSqrRpm);
              logFile.print(F(";"));
              break;
            // bit 1 : inDigiSqrOpt1;
            case 1:
              logFile.print(inDigiSqrOpt1);
              logFile.print(F(";"));
              break;
            // bit 2 : inDigiBrake;
            case 2:
              logFile.print(inDigiBrake);
              logFile.print(F(";"));
              break;
            // bit 3 : inDigiOpt1;
            case 3:
              logFile.print(inDigiOpt1);
              logFile.print(F(";"));
              break;
            // bit 4 : inAnaGear;
            case 4:
              logFile.print(gear);
              logFile.print(F(";"));
              break;
            // bit 5 : inAnaThrottle;
            case 5:
              logFile.print(inAnaThrottle);
              logFile.print(F(";"));
              break;
            // bit 6 : inAnaOpt1;
            case 6:
              logFile.print(inAnaOpt1);
              logFile.print(F(";"));
              break;
            // bit 7 : inAnaOpt2;
            case 7:
              logFile.print(inAnaOpt2);
              logFile.print(F(";"));
              break;
          }
        }
        bitShift = bitShift << 1;
      }
      logFile.print(fix_data.speed_kph(), 0);
      logFile.print(F(";"));
      logFile.print(fix_data.heading(), 1);
      logFile.print(F(";"));
      logFile.print("ROLL");
      logFile.print(F(";"));
      logFile.print("TEMP");
      logFile.print(F(";"));
      for (uint8_t i = 0; i < maxMlx; i++) {
        if (mlxAddresses[i] != 0x00) {
          logFile.print(mlxValues[i]);
          logFile.print(F(";"));
        }
      }
      if (addFinishLog == true) {
        logFile.print((posCrossLat / rescaleGPS), 9);
        logFile.print(F(";"));
        logFile.println((posCrossLon / rescaleGPS), 9);
      } else {
        logFile.print(fix_data.latitude(), 9);
        logFile.print(F(";"));
        logFile.println(fix_data.longitude(), 9);
      }
    } else { // isRunning == false
      /**************************************************************
        Get bluetooth commands (only when not running)
        Wait for orders
      **************************************************************/
      while (BLUETOOTH_PORT.available()) {
        c = processCharInput(cmdBuffer, BLUETOOTH_PORT.read());
        if (c == '\n') {
          if (showSubMenu31 == true) {
            if (strcmp("c", cmdBuffer) != 0) {
              fChangeEnabledInputs(atoi(cmdBuffer));
            }
            showSubMenu31 = false;
          } else {
            if (strcmp("help", cmdBuffer) == 0) {
              fGetHelp();
            }
            if (strcmp("1", cmdBuffer) == 0) {
              fGetData();
            }
            if (strcmp("2", cmdBuffer) == 0) {
              fGetLastRuns(12);
            }
            if (strcmp("3", cmdBuffer) == 0) {
              fGetLastRuns(48);
            }
            if (strcmp("4", cmdBuffer) == 0) {
              fGetTrackBest();
            }
            if (strcmp("20", cmdBuffer) == 0) {
              fCalThrottle(BLUETOOTH_PORT);
            }
            if (strcmp("21", cmdBuffer) == 0) {
              fCalAnaOpt1();
            }
            if (strcmp("22", cmdBuffer) == 0) {
              fCalAnaOpt2();
            }
            if (strcmp("23", cmdBuffer) == 0) {
              fCalGear(BLUETOOTH_PORT);
            }
            if (strcmp("24", cmdBuffer) == 0) {
              fCal9axis();
            }
            if (strcmp("30", cmdBuffer) == 0) {
              fShowEnabledInputs();
            }
            if (strcmp("31", cmdBuffer) == 0) {
              BLUETOOTH_PORT.println(F("Enter new value (c to cancel) : "));
              showSubMenu31 = true;
            }
            if (strcmp("40", cmdBuffer) == 0) {
              fListMlxAddr();
            }
            if (strcmp("41", cmdBuffer) == 0) {
              fDetectMlx(BLUETOOTH_PORT);
            }
          }
          cmdBuffer[0] = 0;
        }
      }
    }
  }
}
