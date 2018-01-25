#include <math.h>
//Hi josh this is my code
int sensorPin = A0;    // select the input pin for the thermistor
float sensorValue = 0;  // variable to store the value coming from the sensor
double temperature[500];

int t_i = 1;   //first temp value
//Kalman Setup
/////////////////////////// 
      double p_k[500]; //init
      double T_hat_k[500]; //init
      double g; //init
      // settings
      double c = 1;    // Measurement State space. Basically sensor is sole contributor
      double r = 0.2;  //Measurement noise covariance. For example thermistor fluctuates 0.8 degrees on avg ?
      double q = 0.25; //Process noise covariance


//////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
//Kalman
///////////////////////////
      p_k[0] = 1;
      T_hat_k[0] =0;
 
 //first value Temperature for Kalman Filter
      sensorValue = analogRead(sensorPin);
      sensorValue = sensorValue*(5.0/(1023));
      temperature[0] = -12.531*pow(sensorValue,3)+118.81*pow(sensorValue,2)-413.27*sensorValue+671.85;
      Serial.print(temperature[0]);
///////////////////////////

}

double KalmanFilter(double c, double r, double q, double xhat_0, double z_k, double p_k_0, double p_k, double xhat_k )
{

    //predict
    xhat_k =xhat_0;
    p_k = p_k_0+q;
    
    //update
    g = p_k*c/(c*p_k*c+r);
   
    xhat_k = xhat_k+g*(z_k-c*xhat_k);
    p_k= (1-g*c)*p_k;
    return xhat_k;
  }


void loop() {
  // put your main code here, to run repeatedly:
  
//Thermistor
/////////////////////////// 
      sensorValue = analogRead(sensorPin);
      sensorValue = sensorValue*(5.0/(1023));
      temperature[0] = -12.531*pow(sensorValue,3)+118.81*pow(sensorValue,2)-413.27*sensorValue+671.85;
      temperature[t_i] = -12.531*pow(sensorValue,3)+118.81*pow(sensorValue,2)-413.27*sensorValue+671.85;
////////////////////////////


T_hat_k[t_i]=KalmanFilter(c,r,q,T_hat_k[t_i-1],temperature[t_i],p_k[t_i-1],p_k[t_i],T_hat_k[t_i]);
Serial.print(temperature[t_i]);
Serial.print("\t");
Serial.println(T_hat_k[t_i]);
t_i++;

delay(1000);
}
