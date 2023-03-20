#include <avr/pgmspace.h>

//------------------------------------------------------------------------------------------------------
//StaticTrack is adapted from https://github.com/daveake/FlexTrack

//This program uses multiple connected files: aprs, gps, misc, rtty and a sine_table.h 

//The most important settings are located here in the config section.


// CONFIGURATION SECTION.


// Main settings
#define Activate_APRS 
#define Activate_GPS
//#define Activate_RTTY //if you place "//" before this you can deactivate it.

#define minimumSatellites 4                 //Amount of connected GPS satellites necessary to transmit APRS messages (to test can be set to 0)

// GPS Power settings
//#define POWERSAVING                          // Comment out to disable GPS power saving. THIS NEEDS TO BE TESTED!

// RTTY settings
#define RTTY_PAYLOAD_ID   "ON6GMZ"          // Do not use spaces.
#define RTTY_FREQUENCY    434.65               // For devices that are frequency-agile
#define RTTY_BAUD          75               // Comment out if not using RTTY
#define RTTY_SHIFT        425                // Only used on boards where PWM is used for RTTY.

// APRS settings
#define APRS_CALLSIGN    "ON6GMZ"               // Callsign (Max 6 characters)
#define APRS_SSID            11                 // Balloon launch
#define APRS_PATH_ALTITUDE   1500              // Below this altitude, (metres), path will switch to WIDE1-1, WIDE2-1.  Above it will be or path or WIDE2-1 (see below)
#define APRS_HIGH_USE_WIDE2    1                 // 1 means WIDE2-1 is used at altitude; 0 means no path is used

#define APRS_TX_INTERVAL      60                 // APRS Transmission Interval in seconds
#define APRS_RANDOM          5                // Adjusts time to next transmission by up to +/1 this figure, in seconds.
                                               // So for interval of 60 (seconds), and random(30), each gap could be 30 - 90 seconds.
                                               // Set to 0 to disable!
                                              
#define APRS_COMMENT     "www.dirkgeeroms.com"   
#define APRS_TELEM_INTERVAL  1                // How often to send telemetry packets.  Comment out to disable
#define APRS_PRE_EMPHASIS                      // Comment out to disable 3dB pre-emphasis.


//------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------

// HARDWARE DEFINITIONS

//Habduino board pins 

#define LED_WARN           9
#define LED_OK             8
#define GPS_ON             2
#define RTTY_ENABLE        7
#define RTTY_DATA          4
#define APRS_ENABLE        6
#define APRS_DATA          3                // Comment out to disable APRS  
#define A0_MULTIPLIER      4.9              
#define WIREBUS            5
#define MTX2


//------------------------------------------------------------------------------------------------------

//GPS configuration, it is recommended to not change anything here!

#define EXTRA_FIELD_FORMAT    ",%d,%d,%d"          // List of formats for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
#define EXTRA_FIELD_LIST           ,(int)((GPS.Speed * 13) / 7), GPS.Direction, GPS.Satellites

// #define EXTRA_FIELD_FORMAT      ",%d,%d,%d,%d,%d,%d"          // List of formats for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
// #define EXTRA_FIELD_LIST            ,(int)((GPS.Speed * 13) / 7), GPS.Direction, GPS.Satellites, DS18B20_Temperatures[0], Channel0Average, GPS.CutdownStatus
                                                                // List of variables/expressions for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
#define SENTENCE_LENGTH      100                  // This is more than sufficient for the standard sentence.  Extend if needed; shorten if you are tight on memory.

    /*
            "$%s,%d,%02d:%02d:%02d,%s,%s,%05.5u,%d,%d,%d",
            PAYLOAD_ID,
            SentenceCounter,
	    GPS.Hours, GPS.Minutes, GPS.Seconds,
            LatitudeString,
            LongitudeString,
            GPS.Altitude,
            (int)((GPS.Speed * 13) / 7),
            GPS.Direction,
            GPS.Satellites);
    */


//------------------------------------------------------------------------------------------------------
//
//  Global objects, if you want to learn about this -> https://www.tutorialspoint.com/structs-in-arduino-program

struct TBinaryPacket
{
	uint8_t 	PayloadIDs;
	uint16_t	Counter;
	uint16_t	BiSeconds;
	float		Latitude;
	float		Longitude;
	int32_t  	Altitude;
};  //  __attribute__ ((packed));

struct TGPS
{
  int Hours, Minutes, Seconds;
  unsigned long SecondsInDay;					// Time in seconds since midnight
  float Longitude, Latitude;
  long Altitude;
  unsigned int Satellites;
  int Speed;
  int Direction;
  byte FixType;
  byte psm_status;
  float InternalTemperature;
  float BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte FlightMode;
  byte PowerMode;
  int CutdownStatus;
} GPS;

//Amount of RTTY packages transmitted
int SentenceCounter=0;

//------------------------------------------------------------------------------------------------------

#define GPS_SERIAL Serial
#define DEBUG_SERIAL Serial

void setup() { //Setup and annoucements
  #ifdef GPS_SERIAL
    GPS_SERIAL.begin(9600);
  #endif
	
  DEBUG_SERIAL.begin(9600);
  Serial.println("");
  Serial.print("StaticTrack Flight Computer, payload ID(s):");
    
    #ifdef Activate_RTTY
      Serial.print("RTTY: ");
      Serial.print(RTTY_PAYLOAD_ID);
      Serial.println("");
    #endif 
     
    #ifdef Activate_APRS
      Serial.print("APRS: ");
      Serial.print(APRS_CALLSIGN);
      Serial.print("-");
      Serial.print(APRS_SSID);
      Serial.println("");
    #endif  
      
  Serial.println("");
  
  Serial.println("Active systems:");

    #ifdef Activate_GPS
      Serial.println(F("GPS enabled"));
      SetupGPS();
    #endif
    
    #ifdef Activate_RTTY
      Serial.println(F("RTTY telemetry enabled"));
      SetupRTTY();
    #endif

    #ifdef Activate_APRS 
      Serial.println(F("APRS telemetry enabled"));
      SetupAPRS();
    #endif

  Serial.print(F("Free memory = "));
  Serial.println(freeRam());
   
}


void loop() {  
  CheckLEDs(); //control leds on habduino
  //Each Check function updates the data or starts the correct radio functions
  #ifdef Activate_GPS
    CheckGPS();
  #endif
  
  #ifdef Activate_RTTY
    CheckRTTY();
  #endif

  #ifdef Activate_APRS
    CheckAPRS();
  #endif
  
}

//With this function you can find out how much room you have left on the Arduino.
int freeRam(void)
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
