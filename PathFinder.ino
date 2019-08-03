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
Setup GPS shied         In Mega
TX          D3            D13
RX          D2            D12

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

/*
 * Setup Sim Module for broadband communication
 * For Sim 900 Tx=7, Rx=8         ON Mega 10,11
 * For Sim7000 Tx=8, Rx=7
 */

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

//static const int RXPin = 3, TXPin = 2;
static const int RXPin = 13, TXPin = 12;      //In Mega
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial gpsSoftSerial(RXPin, TXPin);

// For magnometer
//#include "Wire.h"
#include <HMC5883L.h>
HMC5883L compass;                                     //Copy the folder "HMC5883L" in the folder "C:\Program Files\Arduino\libraries" and restart the arduino IDE.

#include <Wire.h>
#include <DFRobot_SIM7000.h>

SoftwareSerial simClient(10,11);
DFRobot_SIM7000 sim7000;

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
//   GPS shield serial client
  gpsSoftSerial.begin(GPSBaud);
//
  dest_lat = 6.7971231, dest_long = 79.8995026;
  mode = "SELF";
  pinMode(motor_left_positive, OUTPUT);
  pinMode(motor_left_negative, OUTPUT);  
  pinMode(motor_right_positive, OUTPUT); 
  pinMode(motor_right_negative, OUTPUT);
//
  Serial.begin(115200);
//
  //Left Signal LED
  pinMode(GPSSignal, OUTPUT);

  // Right Signal LED
  pinMode(collisionSignal, OUTPUT);

  // Magnometer
  Wire.begin();  
  compass = HMC5883L();  
  setupHMC5883L();

  //SIM900
  simClient.begin(19200);
  delay(500);

  initSIM();
  connectGPRS();

  smartDelayReadGPS(300);
  updateLocation();
  sendGETRequest("http://ancient-fjord-80490.herokuapp.com/device-pos-details?lat=3.652456&long=2.542663&direction=25.455&sat="+String(sat_count)+"&speed="+String(cur_speed)+"&mode="+String(mode),10);
  smartDelayReadGPS(1000);
  readGetResponse();


}

void loop() {

//  String requestLink = 
  sendGETRequest("http://ancient-fjord-80490.herokuapp.com/device-pos-details?lat="+String(cur_lat)+"&long="+String(cur_long)+"&direction="+String(cur_course)+"&sat="+String(sat_count)+"&speed="+String(cur_speed)+"&mode="+mode,10);

  smartDelayReadGPS(300);
  avoidCollision();
  updateLocation();
//  calc_course_from_magnometer();
  if(mode.equals("S") && !state.equals("IGPS")){
    moveToDest();
    }
  else if(mode.equals("M")){
   execCommand(); 
   }
  readGetResponse();

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
  
  if(ms >=50){
      gpsSoftSerial.listen();
      unsigned long start = millis();
      do 
      {
        while (gpsSoftSerial.available())
          gps.encode(gpsSoftSerial.read());
      } while (millis() - start < ms);
      simClient.listen();
    }
   else{
      delay(ms);
    }
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


/*
 *Setting the GPRS settings for the Sim Module
 * 
 */

void connectGPRS()
{ 
//  Serial.println("connection type");
  sendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"",100);

//  Serial.println("Set APN");
  sendCommand("AT+SAPBR=3,1,\"APN\",\"ppwap\"",100);//APN
  
  sendCommand("AT+SAPBR=1,1",100);

  sendCommand("AT+SAPBR=2,1",100);
}

void sendGETRequest(String URL, int delayt){
   
//  Serial.println("Initi HTTP");
  sendCommand("AT+HTTPINIT",100);

//  Serial.println("Opening Bearer Profile");
  sendCommand("AT+HTTPPARA=\"CID\",1",100);
  String requestLink = "AT\+HTTPPARA=\"URL\",\""+URL+"\"";
  Serial.println(requestLink);
//  sendCommand("AT+HTTPPARA=\"URL\",\"http://xxx.xxx.xx/Listener/\"");//Public server IP address
  sendCommand(requestLink,100);//Public server address
  
//  Serial.println("Set content type");
//  sendCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"",100);
  
//  simClient.println("AT+HTTPACTION=0");
//  Serial.println("send get request");
  sendCommand("AT+HTTPACTION=0",delayt);

//  sendCommand("AT+HTTPREAD=0,142",1000);

  }


void readGetResponse(){

  simClient.println("AT+HTTPREAD=0,142");
  delay(200);

  String resp = "|";
  while(simClient.available()!=0){
    
    resp += simClient.readStringUntil('\n')+"|";
    }
//    Serial.println(resp);
    resp = getValue(resp,'|',3);
    if(!resp.equals("")){
      float destLat = getValue(resp,',',0).toFloat();
      float destLong = getValue(resp,',',1).toFloat();
      String rMode = getValue(resp,',',2);
      String rCommand = getValue(resp,',',3);
      Serial.println("Response results dl : "+String(destLat) + " DLo : "+ String(destLong) + " Mode : " + mode + " Command : " + command);
      if(destLat !=0){
        dest_lat = destLat;
        }
      if(destLong !=0){
        dest_long = destLong;
        }

      if(!rMode.equals("")){
        mode = rMode;
        }
      if(!rCommand.equals("")){
        command = rCommand;
        }
      }

//  Serial.println("Terminating");
  sendCommand("AT+HTTPTERM",100);

  }
void ShowSerialData()
{
//  readInFromSIM = "";
 while(simClient.available()!=0)
  {
//    readInFromSIM+= simClient.readString();
//  Serial.println(simClient.readStringUntil('\n'));
    Serial.write(simClient.read());
  }
//  Serial.println(readInFromSIM);
}


void initSIM(){

//  turnOnBoard();

  //Set Baud rate to 19200 for reliable communication
//  sendCommand("AT+IPR=19200",200);
    sim7000.begin(simClient);
    Serial.println("Turn ON SIM7000......");
    if(sim7000.turnON()){                                       //Turn ON SIM7000
        Serial.println("Turn ON !");
    }

    Serial.println("Check SIM card......");
    if(sim7000.checkSIMStatus()){                               //Check SIM card
        Serial.println("SIM card READY");
    }else{
        Serial.println("SIM card ERROR, Check if you have insert SIM card and restart SIM7000");
        while(1);
    }
  }

void sendCommand(String command,int delayTime){
  
  simClient.println(command);
  smartDelayReadGPS(delayTime);
//  readInFromSIM = "";
  ShowSerialData();
  }

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
