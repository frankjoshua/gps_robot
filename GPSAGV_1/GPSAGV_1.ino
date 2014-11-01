//compiled on 1.5.1r2
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>  
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

//Compass stuff
int HMC6352Address = 0x42;
int slaveAddress;
byte headingData[2];
int i, headingValue;
// tiny gps library code
float flat, flon;
float heading=0;
int headinggps;
TinyGPS gps;
SoftwareSerial mySerial(2, 3);    //used for gps rx and tx pins in use

//Compass Stuff
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float compassHeading;

//LCD stuff
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); 
// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5
//Menu System
#define MENU_LENGTH 4
#define CMD_DIRECTION 0
#define CMD_SHOW_POSITION 1
#define CMD_ADD_WAY_POINT 2
char* mMenu[]={"Direction", "Show Position", "Add Way Point", "Delete Way Point",
"Clear Way Points"};
int mMenuPosition = 0;

#define led 13
int x4=0;
#define buttons 0

//Current position
float x2lat;
float x2lon;

//Store Waypoints
int waypoints;   // number of waypoints
int waycont=1; //Current waypoint
#define MAX_WAYPOINTS 20
float mLat[MAX_WAYPOINTS]; //28.324603,-82.267419 home testing
float mLon[MAX_WAYPOINTS];

//Method definitions
void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int digits = 2);


  void setup(){
    lcd.begin(16, 2);              // start the library
    lcd.setCursor(0,0);
    lcd.print("NEW TESTING..."); // print a simple message
 
    //Setup compass
    slaveAddress = HMC6352Address >> 1;  
    Wire.begin();
    //Set up diagnostic output
    Serial.begin(115200);
    Serial.println("Booting ...");
    //Set gps
    mySerial.begin(4800);
    //Setup output led
    pinMode(led, OUTPUT);
    //Setup input buttons
    pinMode(buttons, INPUT);
    /* Initialise the compass */
    if(!mag.begin()){
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while(1);
    }
    //Setup test waypoint
    mLon[0] = -82.267419;
    mLat[1] = 28.324603;
    //Show initial menu
    displayMenu();
  }

  void loop(){ 
    //Read from gps
    if (feedgps()){
      //Dump gps if new data
      gpsdump(gps);
    }
    updateHeading();
    updateDisplay();
  }

  /**
  * Reades the heading from HMC5883 and update compassHeagin in degrees
  */
  void updateHeading(){
    /* Get a new sensor event */ 
    sensors_event_t event; 
    mag.getEvent(&event);
    
    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.01;
    heading += declinationAngle;
    
    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
      
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
     
    // Convert radians to degrees for readability.
    compassHeading = heading * 180/M_PI; 
  }
  
  void updateDisplay(){
    //Check if a button was pressed
    int btnPressed = read_LCD_buttons();
    
    boolean isPress = true;
    switch(btnPressed){
      case btnUP:
        //Move menu up
        if(mMenuPosition > 0){
          mMenuPosition--;
        }
      break; 
      case btnDOWN:
        //Move menu down
        if(mMenuPosition < MENU_LENGTH - 1){
          mMenuPosition++;
        }
      break;
      case btnSELECT:
        executeCommand(mMenuPosition);
      break;
      default:
        //No buttons pressed
        isPress = false;
      break;  
    }
    
    //Diplay menu if changed
    if(isPress){
      displayMenu();
    }
    //Short delay to prevent rebounce
    delay(150);
  }

  void displayMenu(){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(">");
      lcd.print(mMenu[mMenuPosition]);
      if(mMenuPosition < MENU_LENGTH){
        lcd.setCursor(0,1);
        lcd.print(mMenu[mMenuPosition + 1]);
      }
  }

  void executeCommand(int cmd){
    switch(cmd){
       case CMD_SHOW_POSITION:
         //Display current position
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print("LAT: ");
         lcd.print(flat);
         lcd.print(" ");
         lcd.print(compassHeading);
         lcd.setCursor(0,1);
         lcd.print("LON: ");
         lcd.print(flon);
         lcd.print(" ");
         lcd.print(getDistance(0, flat, flon));
         delay(500);
       break;
       case CMD_DIRECTION:
         
       break;
    }
    
  }
  
 
  void gpsdump(TinyGPS &gps){
    unsigned long age, date, time, chars;
    unsigned short sentences, failed;
  
    feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors
    //Read position
    gps.f_get_position(&flat, &flon, &age);

    feedgps();
    
    //Read erros
    gps.stats(&chars, &sentences, &failed);
    
  }
  
  /**
  * Read gps a return true if new data found
  */
  bool feedgps(){
    bool newData = false;
    while (mySerial.available()){
      if (gps.encode(mySerial.read()))
        newData = true;
    }
    return newData;
  }
  
  /**
  * Returns distance from wayPoint in meters
  * Pass in the current Latitude and Longitude
  */
  int getDistance(int wayPoint, float currentLat, float currentLon){
    //Convert everthing to radians
    float waypointLat = radians(mLat[wayPoint]);
    float waypointLon = radians(mLon[wayPoint]);
    currentLat = radians(currentLat);
    currentLon = radians(currentLon);
    
    //Find the differance of lat an lon
    float diffLat = waypointLat - currentLat;
    float diffLon = waypointLon - currentLon;
    
    //Calculate the distance
    float distance = (sin(diffLat/2.0)*sin(diffLat/2.0));
    float distance2 = cos(currentLat);
    distance2 *= cos(waypointLat);
    distance2 *= sin(diffLon/2.0);
    distance2 *= sin(diffLon/2.0);
    distance += distance2;
    distance = (2*atan2(sqrt(distance),sqrt(1.0-distance)));
    
    //Convert to meters
    distance *= 6371000.0;
    
    return distance;
  }
  
//------------------------------below is the part of the code where everything is calculated
  void distance(){
    x2lat = mLat[waycont - 1]; 
    x2lon = mLon[waycont - 1];
   
    float flat1=flat;            
    float flon1=flon;
    float dist_calc=0;
    float dist_calc2=0;
    float diflat=0;
    float diflon=0;
    
    //------------------------------------------ distance formula below. Calculates distance from current location to waypoint
    diflat=radians(x2lat-flat1);  //notice it must be done in radians
    flat1=radians(flat1);
    x2lat=radians(x2lat);
    diflon=radians((x2lon)-(flon1));
    
    dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
    dist_calc2= cos(flat1);
    dist_calc2*=cos(x2lat);
    dist_calc2*=sin(diflon/2.0);                                       
    dist_calc2*=sin(diflon/2.0);
    dist_calc +=dist_calc2;
    
    dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
    
    dist_calc*=6371000.0; //Converting to meters
    Serial.println("distance");
    Serial.println(dist_calc);
    if(dist_calc<2){
      if(waycont==waypoints){
        done();
      }
      waycont+=1;
    }
    
    //-----------------------------------------heading formula below. Calculates heading to the waypoint from the current locaiton
    flon1 = radians(flon1);  //also must be done in radians
    x2lon = radians(x2lon);
    
    heading = atan2(sin(x2lon-flon1)*cos(x2lat),cos(flat1)*sin(x2lat)-sin(flat1)*cos(x2lat)*cos(x2lon-flon1)),2*3.1415926535;
    heading = heading*180/3.1415926535;  // convert from radians to degrees
    int head =heading; 
    if(head<0){
      heading+=360;   //if the heading is negative then add 360 to make it positive
    }
    
    Serial.println("heading:");
    Serial.println(heading);
    //-------------------------------------------------------------
    
    int turn=0;
    Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
    Wire.write("A");              
    Wire.endTransmission();
    delay(10);                  
    Wire.requestFrom(slaveAddress, 2);       
    i = 0;
    while(Wire.available() && i < 2){ 
      headingData[i] = Wire.read();
      i++;
    }
    headingValue = headingData[0]*256 + headingData[1];
    int pracheading = headingValue / 10;      // this is the heading of the compass
    if(pracheading>0){
       headinggps=pracheading;
    }
    Serial.println("current heaDING:");
    Serial.println(headinggps);
    x4=headinggps-heading;   //getting the difference of our current heading to our needed heading
    
    //Serial.println(absolute);
    int x5;
    //-------------------------------------- below tells us which way we need to turn
    if(x4>=-180){
      if(x4<=0){
        turn=8;    //set turn =8 which means "right"         
      }
    }
    if(x4<-180){
      turn=5;      //set turn = 5 which means "left"
    }
    if(x4>=0){
      if(x4<180){
        turn=5;   //set turn = 5 which means "left"
      }
    }
    if(x4>=180){     //set turn =8 which means "right"
      turn=8;
    }
    //-----------------------------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------real important turning stuff. DO NOT TOUCH!!!
    float hd = headinggps;
    if(hd==heading){
        turn=3;   //then set turn = 3 meaning go "straight" 
    }
    
    if(turn==3){   
      Serial.println("straight");
      //digitalWrite(mo1, LOW);               //go straight 
      //digitalWrite(mo2, HIGH);
      //digitalWrite(mo3, LOW);
      //digitalWrite(mo4, LOW);
    }
    //-------------------------------------------------------------------------------------turn right
    if(turn==8){
       rightturn();
    }
    //------------------------------------------------------------------------------------------turn left
    if(turn==5){
       leftturn();
    }
     //-------------------------------------------------------------------------
  }

  

  void done(){
    //Stop Motors Here
  }
//----------------------------------------------------------------------------------
//---------------------------------------------------------------------------
//------------------------------------------------------right turning
  void rightturn(){
    if(headinggps+2>heading){
      if(headinggps-2<heading){
//        digitalWrite(mo3, LOW);
//        digitalWrite(mo4, LOW);
        return;
      }
    }
          x4=headinggps-heading;  
    if(x4<-180){
    return;
    }
    if(x4>=0){
      if(x4<180){
      return;
      }
    }
    
//      digitalWrite(mo1, LOW);
//      digitalWrite(mo2, HIGH);   
//      digitalWrite(mo3, HIGH);
//      digitalWrite(mo4, LOW); 
      Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
      Wire.write("A");              
      Wire.endTransmission();
      delay(10);                  
      Wire.requestFrom(slaveAddress, 2);       
      i = 0;
      while(Wire.available() && i < 2)
      { 
        headingData[i] = Wire.read();
        i++;
      }
      headingValue = headingData[0]*256 + headingData[1];  
    headinggps = headingValue / 10;      // this is the heading of the compass
    rightturn();
  }

//------------------------------------------------------------------------------
//------------------------------------------------------------------
//----------------------------------------------left turning
  void leftturn(){
    if(headinggps+2>heading){
      if(headinggps-2<heading){
//         digitalWrite(mo3, LOW);
//        digitalWrite(mo4, LOW);
        return;
      }
    }
    x4=headinggps-heading;  
    if(x4>=-180){
      if(x4<=0){
         return;         
      }
    }
  
    if(x4>=180){     
      return;
    }
  
//    digitalWrite(mo1, LOW);
//    digitalWrite(mo2, HIGH);
//    digitalWrite(mo3, LOW);
//    digitalWrite(mo4, HIGH); 
    Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
    Wire.write("A");              
    Wire.endTransmission();
    delay(10);                  
    Wire.requestFrom(slaveAddress, 2);       
    i = 0;
    while(Wire.available() && i < 2)
    { 
      headingData[i] = Wire.read();
      i++;
    }
    headingValue = headingData[0]*256 + headingData[1];  
    headinggps = headingValue / 10;      // this is the heading of the compass
    leftturn();
  }

  // read the buttons
  int read_LCD_buttons()
  {
   adc_key_in = analogRead(0);      // read the value from the sensor
   // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
   // we add approx 50 to those values and check to see if we are close
   if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
   if (adc_key_in < 50)   return btnRIGHT; 
   if (adc_key_in < 195)  return btnUP;
   if (adc_key_in < 380)  return btnDOWN;
   if (adc_key_in < 555)  return btnLEFT;
   if (adc_key_in < 790)  return btnSELECT;  
   return btnNONE;  // when all others fail, return this...
  }

