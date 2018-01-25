
/*
 Name:    BuoyControl.ino
 Created: 1/24/2018 10:59:20 AM
 Author:  Zac Palmeter, Joshua Girgis, Austin Kidder, Kale Prentice, Justin Cottle
*/////////////////////////////////////////////////////////////////////////////////////
#include <math.h> //Include math library
double temperature[100];  // allocate memory for temperature reading

//Kalman Filtering Variables
//Thermistor
double T_p_k[100];   //init prior error covariance.
double T_hat_k[100]; //init temperature estimation
double T_c;          //Measurement State space. Basically Thermistor is sole contributor
double T_r;       //Measurement noise covariance. Thermistor fluctuates ? degrees on avg 
double T_q;      //Process noise covariance for Thermistor
int t_i;             //temp reading increment
/////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // the setup function runs once when you press reset or power the board
  Serial.begin(9600);
  initKalman();
}

void initKalman() {
  //Temperature Setup
   T_c = 1;    //Measurement State space. Basically Thermistor is sole contributor
   T_r = 0.2;  //Measurement noise covariance. Thermistor fluctuates ? degrees on avg 
   T_q = 0.25; //Process noise covariance for Thermistor
   t_i = 1;    //Start at 1 for first temp value for Kalman estimate
   T_p_k[0] = 1; // Thermistor's prior error covariance. Assume safe start at 1

  //first value Temperature for Kalman Filter
   temperature[0] =readThermistor(A0);
   T_hat_k[0] = temperature[0];  // start at last known reading!
}


double KalmanFilter(double c, double r, double q, double xhat_0, double z_k, double p_k_0, double p_k, double xhat_k) {
// Stand alone Filters out a signal based on state space, noise estimates, and measurements
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
  //Reads Thermistor and Filters signal
  sensorValue = analogRead(sensorPin);  //Read Analog signal
  sensorValue = sensorValue * (5.0 / (1023));  //Convert to voltage
  //Calibration Equation
  temp = -12.531*pow(sensorValue, 3) + 118.81*pow(sensorValue, 2) - 413.27*sensorValue + 671.85;
  return temp;
}


void loop() {
  // put your main code here, to run repeatedly:
  temperature[t_i] = readThermistor(A0);
  T_hat_k[t_i] = KalmanFilter(T_c, T_r, T_q, T_hat_k[t_i - 1], temperature[t_i], T_p_k[t_i - 1],T_p_k[t_i], T_hat_k[t_i]);
 
 //TESTING////////////////
  /**/
  Serial.print(temperature[t_i]);
  Serial.print("\t");
  Serial.println(T_hat_k[t_i]);//*/
//////////////////////////
  t_i++;
  if (t_i>=100) t_i=1;
  delay(100);
}
