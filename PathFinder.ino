/***
Move to Destination based on GPS,
Use Zonar to avoid obstacles
Communicate with Web Client through Ethernet
***/
/***
Setup Motor Controller
LP          A0
LN          A1
RP          A2
RN          A3  
RLED        A4
LLED        A5
Sonar Echo  D5
Sonar Trig  D4

***/
/***
Setup GPS shied
TX          D3
RX          D2

Setup Magnometer
SCL         SCL
SDA         SDA

Set declinationAngle ;

 float declinationAngle = 0.03;
 Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
 Find yours here: http://www.magnetic-declination.com/
 Mine is: 2* 1' W, which is ~2 Degrees, or (which we need) PI*2/180=0.03 radians
 If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
***/

#include <NewPing.h>

#define motor_left_positive  A0
#define motor_left_negative  A1    
#define motor_right_positive  A2   
#define motor_right_negative  A3   
#define MAX_DISTANCE 200  

int trig = 4;
int echo = 5;
unsigned int collisionDistance = 0;

NewPing sonar(trig, echo, MAX_DISTANCE); 
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;
// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// For magnometer
#include "Wire.h"
#include "HMC5883L.h"
HMC5883L compass;                                     //Copy the folder "HMC5883L" in the folder "C:\Program Files\Arduino\libraries" and restart the arduino IDE.

float xv, yv, zv, cur_course;
//calibrated_values[3] is the global array where the calibrated data will be placed
//calibrated_values[3]: [0]=Xc, [1]=Yc, [2]=Zc
float calibrated_values[3]; 

String mode;              // STOP, SELF, MAN
double dest_lat,dest_long,cur_lat, cur_long,dest_course,cur_altitude,cur_speed, distanceMeters; 
int sat_count;
String state;
String command;
int GPSSignal = A4;
int collisionSignal = 9;
void setup() {
  // put your setup code here, to run once:
  ss.begin(GPSBaud);

  dest_lat = 6.7971231, dest_long = 79.8995026;
  mode = "SELF";
  pinMode(motor_left_positive, OUTPUT);
  pinMode(motor_left_negative, OUTPUT);  
  pinMode(motor_right_positive, OUTPUT); 
  pinMode(motor_right_negative, OUTPUT);

  Serial.begin(9600);

  //Left Signal LED
  pinMode(GPSSignal, OUTPUT);

  // Right Signal LED
  pinMode(collisionSignal, OUTPUT);

  // Magnometer
  Wire.begin();  
  compass = HMC5883L();  
  setupHMC5883L();
}

void loop() {
  Serial.println("asda");
  smartDelayReadGPS(300);
  avoidCollision();
  updateLocation();
  calc_course_from_magnometer();
  
  if(mode.equals("SELF") && !state.equals("IGPS")){
    moveToDest();
    }
  else if(mode.equals("MAN")){
   execCommand(); 
   }
}
/*
 * Move to the destination location (dest Lat, dest_long, course)
 */
void moveToDest(){
  double courseShift = cur_course- dest_course;
  double directionSensitiveness = 10.0;
  if(courseShift>0){
    if( courseShift< 180 && courseShift > directionSensitiveness){
        moveLeft();
      }
    else{
        moveRight();
        smartDelayReadGPS(300);
      }    
    }
   else{
      if(abs(courseShift) < 180 && abs(courseShift) > directionSensitiveness ) {
        moveRight();
        state = "R";
        }
      else{
        
        moveLeft();
        smartDelayReadGPS(300);
        state = "L";
        }
    }
    if(distanceMeters > 1.0){
      moveForward();
      smartDelayReadGPS(300);

    }
    else {
      state = "S";
      moveStop();
      mode = "STOP";
      }
  }

/*
 * Execute the turning or moving command from server
 */
void execCommand(){

    if(command.equals("L")){
      moveLeft();
      smartDelayReadGPS(300);
      }
    else if(command.equals("R")){
      moveRight();
      smartDelayReadGPS(300);
    }
    else if(command.equals("F")){
      moveRight();
      smartDelayReadGPS(300);
      }

    else if(command.equals  ("B")){
      moveRight();
      smartDelayReadGPS(300);
      }
    else if(command.equals("S")){
      moveBackward();
      smartDelayReadGPS(300);
      }
  }

/*
 * Read GPS data while delaying this saves delay time efficiently
 */
void smartDelayReadGPS(unsigned long ms){
  
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
 }


 /*
  * Read GPS satellite data and update the variables
  */

void updateLocation(){

    if(gps.location.isValid()){
        
        state = "GPS";

        cur_lat = gps.location.lat();
        cur_long = gps.location.lng();
        cur_course = gps.course.deg();
        dest_course = TinyGPSPlus::courseTo(cur_lat,cur_long,dest_lat,dest_long);
        sat_count = gps.satellites.value();
        cur_altitude = gps.altitude.meters();
        cur_speed = gps.speed.mps();
        distanceMeters = TinyGPSPlus::distanceBetween(cur_lat, cur_long,dest_lat,dest_long);
        Serial.println("LAT : " + String(cur_lat,6) + " Long : "
        + String(cur_long,6) + " Course : " + String(cur_course,2) + " Des Course : " + String(dest_course,2) + " Des Dist : " + String(distanceMeters, 8));

        }
      else{
        Serial.println("Invalid GPS");
        state = "IGPS";
        digitalWrite(GPSSignal,HIGH);
        smartDelayReadGPS(500);
        digitalWrite(GPSSignal,LOW);
        }
    
  }
/*
 * Avoid Collisions by going arround the collision.
 */
void avoidCollision(){
  
   collisionDistance = sonar.ping_cm();
   if((collisionDistance<30) && collisionDistance >3)
   {

    state = "COL";
    moveBackward();
    Serial.println("Collision");
    digitalWrite(collisionSignal, HIGH);
    smartDelayReadGPS(1000);
    digitalWrite(collisionSignal, LOW);
    while(collisionDistance<30 && collisionDistance >3){
        moveLeft();
        collisionDistance = sonar.ping_cm();
        digitalWrite(collisionSignal, HIGH);
        smartDelayReadGPS(100);
        digitalWrite(collisionSignal, LOW);      
      }
     moveForward();
     smartDelayReadGPS(1000);   
    }
    else{
        smartDelayReadGPS(1000);
      }

  }
/*******************************
 * Move Forward * 
 ******************************/

void moveForward()
{
    Serial.println("Moving Forward");
    digitalWrite(motor_left_positive, HIGH); 
    digitalWrite(motor_left_negative, LOW);  
    digitalWrite(motor_right_positive, HIGH);
    digitalWrite(motor_right_negative, LOW); 
}

/*******************************
 * Move Backward * 
 ******************************/
void moveBackward()
{
    Serial.println("Moving Backward");
    digitalWrite(motor_left_positive, LOW); 
    digitalWrite(motor_left_negative, HIGH);
    digitalWrite(motor_right_positive, LOW);
    digitalWrite(motor_right_negative, HIGH);
}
/*******************************
 * Move Left * 
 ******************************/
void moveLeft()
{
    Serial.println("Moving Left");
    digitalWrite(motor_left_positive, HIGH); 
    digitalWrite(motor_left_negative, HIGH); 
    digitalWrite(motor_right_positive, HIGH);
    digitalWrite(motor_right_negative, LOW); 
}
/*******************************
 * Move Right * 
 ******************************/
void moveRight()
{
    Serial.println("Moving Right");
    digitalWrite(motor_left_positive, HIGH); 
    digitalWrite(motor_left_negative, LOW); 
    digitalWrite(motor_right_positive, HIGH);
    digitalWrite(motor_right_negative, HIGH);
}
/*******************************
 * Stop Miving * 
 ******************************/
void moveStop()
{
    Serial.println("Moving Stop");
    digitalWrite(motor_left_positive, HIGH);  
    digitalWrite(motor_left_negative, HIGH);  
    digitalWrite(motor_right_positive, HIGH); 
    digitalWrite(motor_right_negative, HIGH); 
}

//transformation(float uncalibrated_values[3]) is the function of the magnetometer data correction 
//uncalibrated_values[3] is the array of the non calibrated magnetometer data
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc
void transformation(float uncalibrated_values[3])    
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = 
  {
    {11.049, -17.341, -24.102},
    {64.505, 39.692, 37.927},
    {40.788, 4.177, 27.529}  
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
    -124.567,
    -298.841,
    176.174
  };  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

void setupHMC5883L()
{  
  compass.SetScale(0.88);
  compass.SetMeasurementMode(Measurement_Continuous);
}
 
void getHeading()
{ 
  MagnetometerRaw raw = compass.ReadRawAxis();
  xv = (float)raw.XAxis;
  yv = (float)raw.YAxis;
  zv = (float)raw.ZAxis;
}


void calc_course_from_magnometer(){
  
  float values_from_magnetometer[3];
  float heading;
  getHeading();
  values_from_magnetometer[0] = xv;
  values_from_magnetometer[1] = yv;
  values_from_magnetometer[2] = zv;
  transformation(values_from_magnetometer);

  heading = atan2(calibrated_values[1],calibrated_values[0]);
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0457;
  heading += declinationAngle;
    // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  cur_course = heading * 180/M_PI; 
  }

