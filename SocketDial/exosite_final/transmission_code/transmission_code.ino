// Data transmission Code from Nimblink.com edited by Zechariah Palmeter

char temp_char; // need this before includes for some Arduino IDE versions
                // otherwise "'Serial' was not declared in this scope" error occurs

#include <SoftwareSerial.h>

#define NL_SW_LTE_GELS3   // VZW LTE Cat 1 Modem

#define NL_SWDK          // Skywire Development Kit

/*
 * Define Arduino type
 * Uncomment only one!
 * 
 * NOTE:
 *    Debug info on the Serial Monitor is unreliable when using
 *    the Arduino Uno with the Cat 1 modem (at any baud rate)
 */
#define NL_ARDUINO_LEONARDO
//#define NL_ARDUINO_UNO

// the pinouts on the SWDK are not compatible with the Arduino Uno
#if defined NL_ARDUINO_UNO && defined NL_SWDK
#error Arduino + SWDK not supported
#endif

#if defined NL_ARDUINO_UNO && defined NL_SW_LTE_GELS3
#pragma message("Debug info is unreliable for Arduino Uno + Cat 1 modem")
#endif

// Assign APN if 3G GSM or LTE Modem
#if defined NL_SW_HSPAP || defined NL_SW_HSPAPG || defined NL_SW_HSPAPE || defined NL_SW_LTE_TSVG || defined NL_SW_LTE_TNAG || defined NL_SW_LTE_TEUG || defined NL_SW_LTE_GELS3
/* -- CHANGE TO YOUR APN -- */
  #define APN  (String)"NIMBLINK.GW12.VZWENTP"
#endif

/*
 *  Assign device ID
 *  Type your MEID/IMEI here
 *  MEID/IMEI is located on top of your Skywire modem
 */
/* -- CHANGE TO YOUR DEVICE ID: IMEI OR MEID -- */
#define DEVICE_ID (String)"45665482"

// define Serial ports based on Arduino board type
#if defined NL_ARDUINO_LEONARDO
  #define Debug Serial
  #define SW_Serial Serial1
#elif defined NL_ARDUINO_UNO
  #define Debug Serial
  SoftwareSerial SW_Serial(2, 8);  // RX, TX
#endif

/* 
 *  define "Skywire_ON" signal level depending on shield type
 *  The NL_AB_ST_NCL shield has a transistor that connects ON_OFF to ground,
 *  requiring a HIGH output on pin 12 in order to drive ON_OFF LOW
 */ 
#if defined NL_SWDK
  #define SW_ON LOW
#elif defined NL_AB_ST_NCL
  #define SW_ON HIGH
#endif

// initialize a few variables
char incomingByte = 0;
unsigned int dweetCtr = 0;

// code that initializes the serial ports and modem, waits for valid network connection
void setup()
{
  String currentString = "";
  String modemResponse = "";
  bool connectionGood = false;
  
  // initialize serial debug communication with PC over USB connection
  Debug.begin(115200);
  while (!Debug) ; // wait for serial debug port to connect to PC

  for (int q = 5; q > 0; q--)
  {
    Debug.println(q, DEC);
    delay(250);
  }
  Debug.println("Socket Dial and Sending Information to Dweet.io Example");

  // Start cellular modem
  Debug.println("Starting Cellular Modem");
  
  /*
   *  Arduino I/O pin 12 is connected to modem ON_OFF signal.
   *  ON_OFF has internal 200k pullup resister and needs to be driven low 
   *  by external signal for >1s to startup.
   *  Arduino defaults to I/O pin as input with no pullup on reset.
   */
  //Turn off internal PU resistor
  digitalWrite(12, LOW);

  // Configure I/O pin 12 as output
  pinMode(12, OUTPUT);

  pinMode(10, OUTPUT);  // SW_DTR pin (must be driven low if unused)
  pinMode(11, OUTPUT);  // SW_RTS pin (must be driven low if unused)
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);

  // Turn on Skywire modem
  digitalWrite(12, SW_ON);
  
  #if defined NL_SW_LTE_TSVG || defined NL_SW_LTE_TNAG || defined NL_SW_LTE_TEUG || defined NL_SW_HSPAP || defined NL_SW_HSPAPG || defined NL_SW_HSPAPE
    delay(5100); // modem requires >5s pulse
  #else
    delay(1100); // modem requires >1s pulse
  #endif

  // Return I/O pin 12 to input/hi-Z state  
  pinMode(12, INPUT);
  
  #if defined NL_SW_LTE_TSVG || defined NL_SW_LTE_TNAG || defined NL_SW_LTE_TEUG
    delay(15100); // Wait > 15 seconds for initialization
  #elif defined NL_SW_LTE_GELS3
    delay(30100); // Wait > 30 seconds for initialization
  #else
    delay(10100);
  #endif 
  
  // Initialize serial port to communicate with modem
  Debug.println("Initializing modem COM port");
  SW_Serial.begin(115200);
  while (!SW_Serial) ;

  // send "AT" command to confirm modem is turned on
  Debug.println("Test AT command");
  WaitForResponse("AT\r", "OK", 100, modemResponse);

  // turn on echo for Cat 1 modem
  #if defined NL_SW_LTE_GELS3
    WaitForResponse("ATE1\r", "OK", 500, modemResponse);
  #endif

  // Soft reset of modem
  Debug.println("Reseting modem");
  #if defined NL_SW_LTE_GELS3
    WaitForResponse("AT+SOFTRESET\r", "OK", 500, modemResponse);
  #else
    WaitForResponse("ATZ\r", "OK", 500, modemResponse);
  #endif
  
  /* 
   *  In order for SoftwareSerial (only used with the Uno) to provide a reliable output,
   *  the data rate between the Arduino and Skywire modem must be lowered to 38400
   */ 
  #if defined NL_ARDUINO_UNO
    // change modem data rate
    SW_Serial.print("AT+IPR=38400\r");
    delay(1000);

    // restart SoftwareSerial module at lower data rate
    SW_Serial.end();
    SW_Serial.begin(38400);
    while (!SW_Serial) ;
    while(PrintModemResponse() > 0) ; // print any characters sent after data rate change
  #endif

  // turn off URC (unsolicited result code) messages for Cat 1 modem
  #if defined NL_SW_LTE_GELS3
    WaitForResponse("AT+CEREG=0\r", "OK", 500, modemResponse);
  #endif

  //  SIM-based Skywire-specific setup
  #if defined NL_SW_GPRS || defined NL_SW_HSPAP || defined NL_SW_HSPAPG || defined NL_SW_HSPAPE
    // activate SIM detect
    Debug.println("Activating SIM card detect");
    WaitForResponse("AT#SIMDET=1\r", "OK", 500, modemResponse);
  #endif

  // turn on verbose error messages
  WaitForResponse("AT+CMEE=2\r", "OK", 1000, modemResponse);
  
  #if defined NL_SW_GPRS || defined NL_SW_HSPAP || defined NL_SW_HSPAPG || defined NL_SW_HSPAPE || defined NL_SW_LTE_TSVG || defined NL_SW_LTE_TNAG || defined NL_SW_LTE_TEUG || defined NL_SW_LTE_GELS3
    // Setup PDP context
    Debug.println("Setting up PDP context");
    String pdp;

    //*********
    SendModemCommand("AT^SISC=0\r","OK", 500, modemResponse);
    WaitForResponse("AT^SISS=0,\"srvType\",\"Socket\"\r","OK", 500, modemResponse);
    WaitForResponse("AT^SISS=0,\"conId\",3\r","OK", 500, modemResponse);
    WaitForResponse("AT^SISS=0,\"address\",\"socktcp://m2.exosite.com:80\"\r","OK", 500, modemResponse); //*******
    
    delay(10000);
  #endif

  // Check signal strength
  WaitForResponse("AT+CSQ\r", "OK", 500, modemResponse);

  // send command to modem to get firmware version
  WaitForResponse("AT+CGMR\r", "OK", 500, modemResponse);

  // activate PDP context
  bool contextActivated = false;
  String modemResp = "";
  
  /*
  // Check for network connection
  Debug.println("Waiting for network connection");
  connectionGood = false;
  while(!connectionGood)
  {
    // send command to modem to get network status
    #if defined NL_SW_1xRTT_A || defined NL_SW_1xRTT_S || defined NL_SW_1xRTT_V || defined NL_SW_EVDO_A || defined NL_SW_EVDO_V
      SW_Serial.print("AT+CREG?\r");
    #elif defined NL_SW_LTE_GELS3
      SW_Serial.print("AT+CEREG?\r");
    #else
      SW_Serial.print("AT+CGREG?\r");
    #endif
    currentString = "";
    delay(1000);
    
    // Read serial port buffer1 for UART connected to modem and print that message back out to debug serial over USB
    while(SW_Serial.available() > 0) 
    {
      //read incoming byte from modem
      incomingByte=SW_Serial.read();
      //write byte out to debug serial over USB
      Debug.print(incomingByte);
      
      // add current byte to the string we are building
      currentString += char(incomingByte);
  
      // check currentString to see if network status is "0,1" or "0,5" which means we are connected
      if((currentString.substring(currentString.length()-3, currentString.length()) == "0,1") || 
         (currentString.substring(currentString.length()-3, currentString.length()) == "0,5"))
      {
        connectionGood = true;
        while(PrintModemResponse() > 0);  // consume rest of message once 0,1 or 0,5 is found
      }
    }
  }
*/
  delay(2000);
}

// Main loop that sends the information to dweet.io
void loop()
{
  String modemResponse = "";
  int http_timeout = 0;
  
  // increment dummy counter to send to dweet
  //dweetCtr = (dweetCtr + 1) % 100;  
  
  // Setup HTTP connection to dweet.io
  Debug.println("Setting up connection");
  WaitForResponse("AT^SICA=1,3\r", "OK", 500, modemResponse);
  WaitForResponse("AT^SISO=0\r", "OK", 500, modemResponse);
  SendModemCommand("AT^SISW=0,233\r", "^SISW: 0,233,0", 500, modemResponse);  //*******
  
  /*
   * HTTP POST example to dweet.io. Sends variables temp and pressure
   * to dweet.io page for your device ID
   */
  String http_command;
  Debug.println("Sending data to exosite");
  
  // Build string to send to dweet.io
  http_command = "POST /onep:v1/stack/alias HTTP/1.1\n"; //*******
  http_command += "Host: m2.exosite.com\n";
  http_command += "X-Exosite-CIK: 5f76c698acb4a7094fb9bf96eeaac7655703d029\n";
  http_command += "Content-Type: application/x-www-form-urlencoded; charset=utf-8\n";
  http_command += "Accept: application/xhtml+xml\n";
  http_command += "Content-Length: 9\n\n";
  http_command += "temp=30.4";
  Debug.print(http_command);
  SW_Serial.print(http_command);
  delay(5000);
  while (PrintModemResponse() > 0);

  WaitForResponse("AT^SISC=0\r","OK", 500, modemResponse); // Close connection
  
  // Print output
  Debug.println("\nInformation sent to exosite.");
  delay(60000);
}

// sends a command to the modem, waits for the specified number of milliseconds,
// checks whether the modem response contains the expected response, and 
// appends the remaining response characters to the out parameter respOut
// returns true if command received the expected response
bool SendModemCommand(String command, String expectedResp, int msToWait, String& respOut)
{
  int cmd_timeout = 0;
  SW_Serial.print(command);
  delay(msToWait);

  // wait for data to become available, but timeout eventually if no response is received
  while(!SW_Serial.available()) 
  {
    cmd_timeout++;
    if (cmd_timeout == 1000)
    {
      Debug.println("command timeout");
      return false;
    }
    delay(10);
  }

  // read response from modem
  String resp = "";
  respOut = "";
  while(SW_Serial.available() > 0)
  {
    resp += char(SW_Serial.read());
    if(resp.endsWith(expectedResp))
    {
      respOut = resp;
      while(SW_Serial.available() > 0)
        respOut += char(SW_Serial.read());  // append remaining response characters (if any)
      return true;
    }
  }
  respOut = resp;
  return false;
}

// empty read buffer 
void ConsumeModemResponse()
{
  while(SW_Serial.available())
    SW_Serial.read();
}

// repeatedly sends command to the modem until correct response is received
void WaitForResponse(String command, String expectedResp, int msToWait, String& respOut)
{
  bool isExpectedResp;
  do {
    isExpectedResp = SendModemCommand(command, expectedResp, msToWait, respOut);
    Debug.println(respOut);
    SW_Serial.flush();        // just in case any characters weren't transmitted
    ConsumeModemResponse();   // just in case any characters remain in RX buffer
  } while(!isExpectedResp);
}

// returns modem response as a String
String GetModemResponse()
{
  String resp = "";
  while(SW_Serial.available() > 0)
  {
    resp += char(SW_Serial.read());
  }
  return resp;
}

// consumes and prints modem response
int PrintModemResponse()
{
  String resp = "";
  while(SW_Serial.available() > 0) 
  {
    // read incoming modem response into temporary string
    resp += char(SW_Serial.read());
  }
  Debug.println(resp);
  
  //return number of characters in modem response buffer -- should be zero, but some may have come in since last test
  return SW_Serial.available();
}
