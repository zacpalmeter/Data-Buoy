


/*
 Name:    BuoyControl.ino
 Created: 1/24/2018 10:59:20 AM
 Author:  Zac Palmeter, Joshua Girgis, Austin Kidder, Kale Prentice, Justin Cottle
*/////////////////////////////////////////////////////////////////////////////////////
#include <math.h> //Include math library
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h> // For communication with the modem

/* Data Transmission Definitions */
#define NL_SW_LTE_GELS3   // VZW LTE Cat 1 Modem
#define NL_SWDK          // Skywire Development Kit
#define APN  (String)"NIMBLINK.GW12.VZWENTP" //* -- CHANGE TO YOUR APN, this is the APN for a Nimbeling verizon data plan

// define Serial ports for Arduino Mega 2560
#define Debug Serial
#define SW_Serial Serial1

/* 
 *  define "Skywire_ON" signal level depending on shield type
 *  The NL_AB_ST_NCL shield has a transistor that connects ON_OFF to ground,
 *  requiring a HIGH output on pin 12 in order to drive ON_OFF LOW
 */ 
#if defined NL_SWDK
  #define SW_ON LOW
#endif

float transmitTemperature; // Value of temperature to transmit
float transmitSalinity; // Value of Salinity to transmit
String transmitGPS; // GPS location in DDMM.MMMM_DDMM.MMMM
/* End Data Tranmission Definitions */

SoftwareSerial GPS_Serial(50, 51);	//GPS: Setting up the PWM ports for serial use
Adafruit_GPS GPS(&GPS_Serial);		//GPS: calling the ports to the GPS library

#define dataLimit 100

//Echoing the GPS to the serial console
#define GPSECHO true			//GPS: set to to true to observe the raw GPS sentence 
boolean usingInterrupt = false; //GPS:keeping track of the interrupt function
void useInterrupt(boolean);

double temperature[dataLimit];  // allocate memory for temperature reading

int d_i;             //Data reading increment

//Kalman Filtering Variables
//Thermistor
int tempPower = 52;
double T_p_k[dataLimit];   //init prior error covariance.
double T_hat_k[dataLimit]; //init temperature estimation
double T_c;          //Measurement State space. Basically Thermistor is sole contributor
double T_r;       //Measurement noise covariance. Thermistor fluctuates ? degrees on avg 
double T_q;      //Process noise covariance for Thermistor
//Turbidity Sensor

/////////////////////////////////////////////////////////////////////////////////////


void setup() {
// the setup function runs once when you press reset or power the board
//Serial.begin(9600);

/* Thermister setup */
pinMode(tempPower, OUTPUT);						// sets the digital pin 52 as output
initKalman();


/* GPS Setup */
GPS.begin(9600);
GPS_Serial.begin(9600);
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);	//Initializing RMC and GGA of NMEA sentences
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);		//Updating rate setting to 1Hz
GPS.sendCommand(PGCMD_ANTENNA);					//asking for updates on the antenna statues
//GPS: initializing the inturrupt function to read the data every millesecond (if needed)
#ifdef __arm__
usingInterrupt = false;							//NOTE - we don't want to use interrupts on the Due
#else
useInterrupt(true);
#endif
delay(1000);
/* End GPS Setup */


/* Modem Setup */
String modemResponse = ""; // Modem responce to a command
  
  // initialize serial debug communication with PC over USB connection
  Debug.begin(115200);
  while (!Debug) ; // wait for serial debug port to connect to PC

  for (int q = 5; q > 0; q--)
  {
    Debug.println(q, DEC);
    delay(250);
  }
  Debug.println("Socket Dial To Send data to Exosite");

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
  
  delay(1100); // modem requires >1s pulse

  // Return I/O pin 12 to input/hi-Z state  
  pinMode(12, INPUT);
  
  delay(30100); // Wait > 30 seconds for initialization
  
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
  #endif
  
  /* 
   *  In order for SoftwareSerial (only used with the Uno) to provide a reliable output,
   *  the data rate between the Arduino and Skywire modem must be lowered to 38400
   */ 

  // turn off URC (unsolicited result code) messages for Cat 1 modem
  #if defined NL_SW_LTE_GELS3
    WaitForResponse("AT+CEREG=0\r", "OK", 500, modemResponse);
  #endif

  // turn on verbose error messages
  WaitForResponse("AT+CMEE=2\r", "OK", 1000, modemResponse);
  
  #if defined NL_SW_LTE_GELS3
    // Setup PDP context
    Debug.println("Setting up PDP context");

    //*********
    SendModemCommand("AT^SISC=0\r","OK", 500, modemResponse);
    WaitForResponse("AT^SISS=0,\"srvType\",\"Socket\"\r","OK", 500, modemResponse);
    WaitForResponse("AT^SISS=0,\"conId\",3\r","OK", 500, modemResponse); // Set to use PDP context 3
    WaitForResponse("AT^SISS=0,\"address\",\"socktcp://m2.exosite.com:80\"\r","OK", 500, modemResponse); // Configure socket to use TCP on port 80
    
    delay(10000);
  #endif

  // Check signal strength
  WaitForResponse("AT+CSQ\r", "OK", 500, modemResponse);

  // send command to modem to get firmware version
  WaitForResponse("AT+CGMR\r", "OK", 500, modemResponse);

  // activate PDP context
  String modemResp = "";
  
  delay(2000);
  /* End Modem Setup */
}


#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
	char c = GPS.read();
	/*
#ifdef UDR0
	if (GPSECHO)
	if (c) UDR0 = c;	// writing direct to UDR0 is much much faster than Serial.print 
						// but only one character can be written at a time. 
#endif
	*/
}

void useInterrupt(boolean v) {
	if (v) {
		// Timer0 is already used for millis() - we'll just interrupt somewhere
		// in the middle and call the "Compare A" function above
		OCR0A = 0xAF;
		TIMSK0 |= _BV(OCIE0A);
		usingInterrupt = true;
	}
	else {
		// do not call the interrupt function COMPA anymore
		TIMSK0 &= ~_BV(OCIE0A);
		usingInterrupt = false;
	}
}
#endif //#ifdef__AVR__

uint32_t timer = millis();	//setting the timer

// Watchdog Interrupt Service. This is executed when watchdog timed out.
ISR(WDT_vect) 
{
  wdt_disable();  // disable watchdog
}


void enterSleep(const byte interval)
{
  MCUSR = 0;                          // reset various flags
  WDTCSR |= 0b00011000;               // see docs, set WDCE, WDE
  WDTCSR =  0b01000000 | interval;    // set WDIE, and appropriate delay

  wdt_reset();
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  sleep_mode();            // now goes to Sleep and waits for the interrupt
  } 


void initKalman() {
   d_i = 1;    //Start at 1 for first increment value for Kalman estimate
   //Thermistor Setup
   T_c = 1;    //Measurement State space. Basically Thermistor is sole contributor
   T_r = 0.55;  //Measurement noise covariance. Thermistor fluctuates .55 degrees on avg 
   T_q = 0.25; //Process noise covariance for Thermistor
   T_p_k[0] = 1; // Thermistor's prior error covariance. Assume safe start at 1
   //Turbidity Sensor Setup
  
}


double KalmanFilter(double c, double r, double q, double xhat_0, double z_k, double p_k_0, double p_k, double xhat_k) {
// Stand alone Extended Kalman Filter, filters out a signal based on state space, noise estimates, and measurements
// Provides one estimation point in real time
    double g; //init kalman gain
  //predict
  xhat_k = xhat_0;  // estimate future prediction based on prior prediction
  p_k = p_k_0 + q;  // prior error covariance.

  //update
  g = p_k * c / (c*p_k*c + r);  //Kalman gain

  xhat_k = xhat_k + g * (z_k - c * xhat_k);    //update estimate
  p_k = (1 - g * c)*p_k;             //estimate error covariance
  return xhat_k;
}

// the loop function runs over and over again until power down or reset

double readThermistor(int sensorPin) {
  double temp;
  double sensorValue = 0;  // variable to store the value coming from the sensor
  digitalWrite(tempPower,HIGH);
  //Reads Thermistor and Filters signal
  sensorValue = analogRead(sensorPin);  //Read Analog signal
  sensorValue = sensorValue * (5.0 / (1023.0));  //Convert to voltage
  //Calibration Equation  
  temp = -12.531*pow(sensorValue, 3) + 118.81*pow(sensorValue, 2) - 413.27*sensorValue + 671.85;
  
  return temp;
}


void on(){
  // data taking on
   //first value Temperature for Kalman Filter
   temperature[0] =readThermistor(A0);
   T_hat_k[0] = temperature[0];  // start at last known reading!

}

//I wasn't sure of the output of the GPS so I figured this would do for now, so we are able to test the whole system
//Exosite expects the coordinates to come in the format: DDMM.MMMM_DDMM.MMMM
void gpsSensor() {
	/* if you would like to debug and observe the NMEA sentence
	if (!usingInterrupt) {
		
		char c = GPS.read();				// read data from the GPS in the 'main loop'
		
		if (GPSECHO)
		if (c) Serial1.print(c);
	}
	*/
	// if a sentence is received, we can check the checksum, parse it
	if (GPS.newNMEAreceived()) {
		
		if (!GPS.parse(GPS.lastNMEA()))		// this also sets the newNMEAreceived() flag to false
			return;							// we can fail to parse a sentence in which case we should just wait for another
	}

	
	if (timer > millis())  timer = millis();// if millis() or timer wraps around, we'll just reset it

	// approximately every 2 seconds or so, print out the current stats
	if (millis() - timer > 2000) {
		timer = millis();					// reset the timer

		Serial.print("\nTime: ");
		Serial.print(GPS.hour, DEC); Serial.print(':');
		Serial.print(GPS.minute, DEC); Serial.print(':');
		Serial.print(GPS.seconds, DEC); Serial.print('.');
		Serial.println(GPS.milliseconds);
		Serial.print("Date: ");
		Serial.print(GPS.day, DEC); Serial.print('/');
		Serial.print(GPS.month, DEC); Serial.print("/20");
		Serial.println(GPS.year, DEC);

		if (GPS.fix) {
			Serial.print("Location: ");
			Serial.print(GPS.latitude); Serial.print(GPS.lat);
			Serial.print(", ");
			Serial.print(GPS.longitude); Serial.println(GPS.lon);
		}
	}
}

/* Data Transmission Functions */
// Transmits data to exosite, data should be stored in strings transmitTemperature, transmitSalinity, transmitGPS
void sendData() {
  String modemResponse = "";
  
  // Setup HTTP connection to exosite
  Debug.println("Setting up connection");
  WaitForResponse("AT^SICA=1,3\r", "OK", 500, modemResponse);
  WaitForResponse("AT^SISO=0\r", "OK", 500, modemResponse);
  SendModemCommand("AT^SISW=0,233\r", "^SISW: 0,233,0", 500, modemResponse);  // Set command length to total number of characters of the POST command http_command
  
  /*
   * Send data to exosite with post command
   * Use varName=value with appropriate content length for each data point
   */
  String http_command;
  Debug.println("Sending data to exosite");
  
  // Build string to send to exosite
  http_command = "POST /onep:v1/stack/alias HTTP/1.1\n"; //*******
  http_command += "Host: m2.exosite.com\n";
  http_command += "X-Exosite-CIK: 5f76c698acb4a7094fb9bf96eeaac7655703d029\n";
  http_command += "Content-Type: application/x-www-form-urlencoded; charset=utf-8\n";
  http_command += "Accept: application/xhtml+xml\n";
  http_command += "Content-Length: 9\n\n"; // Set content length to number of characters in next line
  http_command += "temp=24.5";
  Debug.print(http_command);
  SW_Serial.print(http_command);
  delay(5000);
  while (PrintModemResponse() > 0);
  // Could put code here to check for data transmission error "+CME ERROR" 

  WaitForResponse("AT^SISC=0\r","OK", 500, modemResponse); // Close connection
  
  // Print output
  Debug.println("\nInformation sent to exosite.");
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
/* End of Data Transmission Functions */



void loop() {
  // this function loops repeatedly until a data limit is reached then it sleeps in 
  // a low power state and overwrites the data.
  sendData();
  
  on();  //inserted for seemless data loop
  if (d_i>=dataLimit){           // when does data taking needs to stop?
    //////////////////////////////////////////////////
    //Insert time for one hour later!
    ///////////////////////////////////////////////////
  //  enterSleep(0b100001);  // 8 seconds
  //  enterSleep(0b100001);  // 8 seconds
  //  enterSleep(0b100000);  // 4 seconds
    ///////////////////////////////////////////////////
    d_i=1; // turns on data taking again
    }
  else {
    ///////////////////////////////////////////////////
    //Insert Data Taking Below!
    ///////////////////////////////////////////////////
    temperature[d_i] = readThermistor(A0);    //take temperature data from A0 pin
    //Filter temperature data using Extended Kalman Filter
    T_hat_k[d_i] = KalmanFilter(T_c, T_r, T_q, T_hat_k[d_i - 1], temperature[d_i], T_p_k[d_i - 1],T_p_k[d_i], T_hat_k[d_i]);
    
    ///////////////////////////////////////////////////
	gpsSensor();
    ///////////////////////////////////////////////////
  /**/
    //print data to plotter for testing
    Serial.print(temperature[d_i]);
    Serial.print("\t");
    Serial.println(T_hat_k[d_i]);//*/
    ///////////////////////////////////////////////////
     d_i++;  //next data point
    
  }
  delay(100);  // somewhat of a delay
}
