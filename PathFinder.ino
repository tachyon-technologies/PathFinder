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


String mode;              // STOP, SELF, MAN
double dest_lat,dest_long,cur_lat, cur_long,cur_course,dest_course,cur_altitude,cur_speed, distanceMeters; 
int sat_count;
String state;
String command;
void setup() {
  // put your setup code here, to run once:
  ss.begin(GPSBaud);

  dest_lat = 6.7972243, dest_long = 79.8995026;
  mode = "SELF";
  pinMode(motor_left_positive, OUTPUT);
  pinMode(motor_left_negative, OUTPUT);  
  pinMode(motor_right_positive, OUTPUT); 
  pinMode(motor_right_negative, OUTPUT);

  Serial.begin(9600);
  
}

void loop() {
  Serial.println("asda");
  smartDelayReadGPS(300);
  avoidCollision();
  updateLocation();

  if(mode.equals("SELF")){
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
  if(courseShift>0){
    if( courseShift< 180 && courseShift > 5.0){
        moveLeft();
      }
    else{
        moveRight();
        smartDelayReadGPS(300);
      }    
    }
   else{
      if(abs(courseShift) < 180 && abs(courseShift) > 5.0 ) {
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
    }
    else {
      state = "D";
      moveStop();
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

    else if(command.equals("B")){
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
        }
    
  }
/*
 * Avoid Collisions by going arround the collision.
 */
void avoidCollision(){
  
   collisionDistance = sonar.ping_cm();
   if((collisionDistance<30)&&(collisionDistance>3))
   {
        state = "COL";
        moveBackward();
        smartDelayReadGPS(1000);
        moveLeft();
        smartDelayReadGPS(300);
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


