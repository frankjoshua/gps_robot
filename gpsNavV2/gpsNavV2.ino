/**
  Designed to be light weight gps nav software for the arduino
  Transmits messages using Easytransfer Library
   email frankjoshua -AT- gmail -DOT- com with questions 
   
   PINS
   GPS 2
   SOFTSERIAL 4,5
*/

#define PIN_GPS 2

#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h> //Needed by the Adafruit BNO055 library
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <EasyTransfer.h>

/** Edit these to tune navigation */
//Minimumn distance in meters to waypoint
#define MIN_DISTANCE 2
//Degrees off course before turn begins
#define MAX_TURN_RANGE 10
//Degrees off course before turn ends
#define TURNING_TURN_RANGE 5
//Turn range will never go below this value
#define MIN_TURN_RANGE 5
//Meters - below this distance turns will be MIN_TURN_RANGE
#define CONSTRAIN_MIN 0
//Meters - below this distance turns will start to become tighter moving from MAX_TURN_RANGE to MIN_TURN_RANGE
#define CONSTRAIN_MAX 100
//Speed of motors from -127 to 127
#define MOTOR_SPEED 70

//Turn directions
#define TURN_LEFT 1
#define TURN_RIGHT 2

//Holds the latitude and longitude
float flat;
float flon;

//Talks to GPS over software serial
TinyGPS mGps;
SoftwareSerial mGpsSerial(PIN_GPS, 255);    //used for gps rx and tx pins in use

//Store Waypoints
#define MAX_WAYPOINTS 6
float mLat[MAX_WAYPOINTS]; //38.628967, -90.270577 Science Center
float mLon[MAX_WAYPOINTS];
byte mCurrentWayPoint = 0;

/* true if in a turn */
boolean turning = false;
byte mSpeed = MOTOR_SPEED;

float mCompassHeading;
int mHeadingOffset = 0;

/** Compass */
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//START Easy Transfer
EasyTransfer etData;
struct COM_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int tar;
  int cmd;
  int val;
  int dur;
};

//give a name to the group of data
COM_DATA_STRUCTURE dataStruct;
//END Easy Transfer

SoftwareSerial SWSerial(4, 5);

/**
  * Read gps a return true if new data found
  */
bool feedgps(){
  bool newData = false;
  while (mGpsSerial.available()){
    char data = mGpsSerial.read();
    if (mGps.encode(data))
      newData = true;
  }
  return newData;
}

/**
* This method updates flat and flon
*/
void gpsdump(TinyGPS &gps){
    unsigned long age, chars;
    unsigned short sentences, failed;
  
    feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors
    //Read position
    gps.f_get_position(&flat, &flon, &age);

    feedgps();
    
    //Read erros
    gps.stats(&chars, &sentences, &failed);
}
  
/**
  * Returns distance from wayPoint in meters
  * Pass in the current Latitude and Longitude
  */
int getDistance(float currentLat, float currentLon, float destinationLat, float destinationLon){
  return calc_dist(currentLat, currentLon, destinationLat, destinationLon);
}

 /**
 * Function to calculate the distance between two waypoints
 */
float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;
  
  //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));
  
  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;
  
  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  
  dist_calc*=6371000.0; //Converting to meters
  //Serial.println(dist_calc);
  return dist_calc;
}

/**
* Returns heading to a waypoint in degrees
* Pass in the current lattitude and longitude
*/
float getHeading(float currentLat, float currentLon, float destinationLat, float destinationLon){
    //Convert everthing to radians
    float waypointLat = radians(destinationLat);
    float waypointLon = radians(destinationLon);
    currentLat = radians(currentLat);
    currentLon = radians(currentLon);
    
    double arg1 = sin(waypointLon-currentLon)*cos(waypointLat);
    double arg2 = cos(currentLat)*sin(waypointLat)-sin(currentLat)*cos(waypointLat)*cos(waypointLon-currentLon);
    float heading = atan2(arg1, arg2);//,2*3.1415926535;
    heading = heading*180/3.1415926535;  // convert from radians to degrees

    //Adjust heading manualy
    heading += mHeadingOffset;
    
    //if the heading is negative then add 360 to make it positive
    if(heading < 0){
      heading+=360;   
    }
    
    return heading;
}

/**
* Reades the heading from BNO055 and update compassHeaging in degrees
*/
sensors_event_t event;
imu::Vector<3> vector;
void updateCompassHeading(){
    
//    vector = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//    mCompassHeading = (float) vector.z();
    
  bno.getEvent(&event);
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.print("\tT: ");
  Serial.print(event.timestamp);
  Serial.println("");
  mCompassHeading = event.orientation.x; 
//  float heading = atan2(event.orientation.y, event.orientation.x);
//  
//  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
//  // Find yours here: http://www.magnetic-declination.com/
//  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
//  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
//  float declinationAngle = 0.01;
//  heading += declinationAngle;
//
//  // Correct for when signs are reversed.
//  if(heading < 0)
//    heading += 2*PI;
//      
//  // Check for wrap due to addition of declination.
//  if(heading > 2*PI)
//    heading -= 2*PI;
// 
//  // Convert radians to degrees for readability.
//  mCompassHeading = heading * 180/M_PI; 

}
  
void setup() {
  
    Serial.begin(115200);
    Serial.println("Starting...");
    
    SWSerial.begin(115200);
    
    //Start Compass
    bno.begin();
    bno.setExtCrystalUse(true);
    
    //Set gps 9600 baud
    mGpsSerial.begin(9600);
    
    //Setup test waypoint
    mLat[0] = 38.637919;
    mLon[0] = -90.272353;
    mLat[1] = 38.638300;
    mLon[1] = -90.272542;
    mLat[2] = 38.638103;
    mLon[2] = -90.272003;
    mLat[3] = 38.63889;
    mLon[3] = -90.271611;
    mLat[4] = 38.638233;
    mLon[4] = -90.271758;
    mLat[5] = 38.637919;
    mLon[5] = -90.272353;
    
    //Start EasyTransfer
    etData.begin(details(dataStruct), &SWSerial);
}

void loop() {
    delay(100);
    //Read from gps
    if (feedgps()){
      //Dump gps if new data
      gpsdump(mGps);
    }
    updateCompassHeading();
    transmitHeading();
}

/**
* Send the heading information to what ever is listening for it.
*/
void transmitHeading(){
    float destHeading = getHeading(flat, flon, mLat[0], mLon[0]);
    int distance = getDistance(flat, flon, mLat[0], mLon[0]);
    navigate(distance, destHeading, mCompassHeading);
    Serial.print(distance);
    Serial.print(",");
    Serial.print(destHeading);
    Serial.print(",");
    Serial.print(mCompassHeading);
    Serial.print(",");
    Serial.print(mLon[0]);
    Serial.print(",");
    Serial.print(mLat[0]);
    Serial.print(",");
    Serial.print(flon);
    Serial.print(",");
    Serial.println(flat);
}

 /**
 * Main navitation code
 */
 void navigate(int distance, float destheading, float currentHeading){
   
   //Check if distance is less then 2 meters
   if(distance <= MIN_DISTANCE){
     //Advance to next way point
     mCurrentWayPoint++;
     if(mCurrentWayPoint >= MAX_WAYPOINTS){
       //Call stop method
       stop();
       return;
     } 
   } else {
     //Navigate to the way point
     //Adjust max amount to be off course
     int maxTurnRange = MAX_TURN_RANGE;
     if(turning){
       maxTurnRange = TURNING_TURN_RANGE;
     } else {
       maxTurnRange = MAX_TURN_RANGE;
     }
     
     //Base turning range off of distance
     int turnRange = constrain(map(distance, CONSTRAIN_MIN, CONSTRAIN_MAX, MIN_TURN_RANGE, maxTurnRange), MIN_TURN_RANGE, maxTurnRange);     
     int turndirection = getdirection(currentHeading, destheading);

     if(abs(currentHeading - destheading) < turnRange){
       //Forward
       turning = false;
       forward(); 
     } else if (turndirection == TURN_LEFT) {
        //Left 
       turning = true;
       left();
     } else if (turndirection == TURN_RIGHT) {
       //Right 
       turning = true;
       right(); 
     }
   }
 }

/**
* JAYS get direction function
* Returns TURN_LEFT for left, TURN_RIGHT for right
*/
int getdirection( int currentHeading, int destheading ){

  int distance = destheading - currentHeading;
   
  int innerdistance = 0;
  int outerdistance = 0;

  if (distance < 0) {
    innerdistance = 360 + distance; 
  } else {
    innerdistance = distance; 
  }

  outerdistance = 360 - innerdistance; 

  if (innerdistance < outerdistance) {
     return TURN_RIGHT;
  } else {
     return TURN_LEFT;
  }

}
 
 
 /**
 * Send stop command
 */
 void stop(){
   dataStruct.tar = 20;
   dataStruct.val = 0;
   etData.sendData();
   dataStruct.tar = 21;
   dataStruct.val = 0;
   etData.sendData();
   Serial.print("stop ");
 }
 
 /**
 * Send forward command
 */
 void forward(){
  dataStruct.tar = 20;
  dataStruct.val = mSpeed / 2;
  etData.sendData();
  dataStruct.tar = 21;
  dataStruct.val = -mSpeed / 2;
  etData.sendData();
  Serial.print("forward ");
 }
 
 /**
 * Send left command
 */
 void left(){
  dataStruct.tar = 20;
  dataStruct.val = mSpeed;
  etData.sendData();
  dataStruct.tar = 21;
  dataStruct.val = -mSpeed;
  etData.sendData();
  Serial.print("left ");
 }
 
 /**
 * Send right command
 */
 void right(){
  dataStruct.tar = 20;
  dataStruct.val = -mSpeed;
  etData.sendData();
  dataStruct.tar = 21;
  dataStruct.val = mSpeed;
  etData.sendData();
  Serial.print("right ");
 }
