
/*
 Name:    BuoyControl.ino
 Created: 1/24/2018 10:59:20 AM
 Author:  Zac Palmeter, Joshua Girgis, Austin Kidder, Kale Prentice, Justin Cottle
*/////////////////////////////////////////////////////////////////////////////////////
#include <math.h> //Include math library
#include <avr/sleep.h>
#include <avr/wdt.h>
#define dataLimit 100

double temperature[dataLimit];  // allocate memory for temperature reading

int d_i;             //Data reading increment

//Kalman Filtering Variables
//Thermistor
int tempPower = 13;
double T_p_k[dataLimit];   //init prior error covariance.
double T_hat_k[dataLimit]; //init temperature estimation
double T_c;          //Measurement State space. Basically Thermistor is sole contributor
double T_r;       //Measurement noise covariance. Thermistor fluctuates ? degrees on avg 
double T_q;      //Process noise covariance for Thermistor
//Turbidity Sensor

/////////////////////////////////////////////////////////////////////////////////////


void setup() {
  // the setup function runs once when you press reset or power the board
  Serial.begin(9600);
  pinMode(tempPower, OUTPUT);          // sets the digital pin 13 as output
  initKalman();
}

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


void loop() {
  // this function loops repeatedly until a data limit is reached then it sleeps in 
  // a low power state and overwrites the data.
  
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
    //TESTING
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
