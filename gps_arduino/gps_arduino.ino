//josh--compiled on 1.5.1r2
//Atom --compiled on 1.0.4 for uno "20x4 LCD tied to SDA A4, SCL A5" displaying data   
// pins used 
//lcd  8, 9, 4, 5, 6, 7 A0
//gps 3,4
//mag A4 A5
//Heading Adjust A2
//Speed Adjust A1

#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>  
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <SabertoothSimplified.h>
#include <EasyTransfer.h>
#include <Kalman.h>

//Optional second 4 line 20 char LCD for displaying data 

#include <LiquidCrystal_I2C.h>

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

//Motor speed -127 to 127
#define SPEED 80
#define TURN_SPEED 55


#define I2C_ADDR 0x27  // Define I2C Address where the PCF8574A is
                          // Address can be changed by soldering A0, A1, or A2
                          // Default is 0x27

// map the pin configuration of LCD backpack for the LiquidCristal class
#define BACKLIGHT_PIN 3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

LiquidCrystal_I2C lcd2(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin,BACKLIGHT_PIN, POSITIVE);

// tiny gps library code
float flat, flon;

TinyGPS gps;
SoftwareSerial mySerial(2, 255);    //used for gps rx and tx pins in use

//Compass Stuff
/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

//Compass stuff
float compassHeading;
int mHeadingOffset = 0;
#define HEADING_ADJUST_PIN A2

//Speed control
#define SPEED_ADJUST_PIN A1
int mSpeedCap = 0;

// 2x16 LCD stuff
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
#define CMD_SHOW_POSITION -2
#define CMD_SHOW_WAY_POINT 0
#define CMD_ADD_WAY_POINT 1
#define CMD_DELETE_WAY_POINT 2
#define CMD_NEXT_WAY_POINT 3
#define CMD_LAST_WAY_POINT 4
#define CMD_DELETE_ALL 5
#define CMD_NAVIGATE 6

int mMenuPosition = CMD_SHOW_WAY_POINT;
#define MENU_LENGTH 4
char* mMenu[]={
"  Show",
"  Add",
" Delete",
"  Next",
"Previous"};
#define MODE_POSITION 0
#define MODE_MENU 1
#define MODE_NAVIGATE 2
#define NUMBER_OF_MODES 3
int mMode = MODE_MENU;
/* Time in millis that the naivagtion should update new data or not */
#define REFRESH_RATE 1000
unsigned long lastUpdate = 0;

//Store Waypoints
#define MAX_WAYPOINTS 20
float mLat[MAX_WAYPOINTS]; //38.628967, -90.270577 Science Center
float mLon[MAX_WAYPOINTS];
int mCurrentWayPoint = 0;

//Kalman filter for smoothing compass value
Kalman compassFilter(0.125,32,1023,0); //suggested initial values for high noise filtering

//Method definitions
void gpsdump(TinyGPS &gps);
bool feedgps();

//Sabertooth motor driver
#define LEFT 1
#define RIGHT 2
#define MOTOR_PIN 3
SoftwareSerial SWSerial(NOT_A_PIN, MOTOR_PIN); // RX on no pin (unused), TX on MOTOR_PIN (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

  void setup(){
    //Used for Motor Controler
    SWSerial.begin(9600);
    
    
    lcd.begin(16, 2);              // start the library
    lcd.setCursor(0,0);
    lcd.print("Starting..."); // print a simple message
 
    //Set up diagnostic outputs
    Serial.begin(9600);
    etData.begin(details(dataStruct), &Serial);
    Serial.println("Starting ...");
    
    //Set gps
    mySerial.begin(9600);

    /* Initialise the compass */
    if(!accel.begin()){
    /* There was a problem detecting the LSM303 ... check your connections */
    lcd.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
   }
    if(!mag.begin()){
      /* There was a problem detecting the HMC5883 ... check your connections */
      lcd.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while(1);
    }
    //Setup test waypoint. Start 38.631514, -90.271908
    mLat[0] = 38.631158;
    mLon[0] = -90.272172;
    mLat[1] = 38.631392;
    mLon[1] = -90.272881;
    mLat[2] = 38.632092;
    mLon[2] = -90.272706;
    mLat[3] = 38.632581;
    mLon[3] = -90.272767;
    mLat[4] = 38.632583;
    mLon[4] = -90.272778;
    mLat[5] = 38.632019;
    mLon[5] = -90.272031;
    mLat[6] = 38.631514;
    mLon[6] = -90.271908;
    
    //Show initial menu
    bootScreen();
    delay (3000);
    displayMenu();
    display2Menu();

  }

  void loop(){ 
    bool newData = false;
    //Read from gps
    if (feedgps()){
      //Dump gps if new data
      gpsdump(gps);
      newData = true;
    }
    updateHeading();
    //Force refresh after set amount of time
    unsigned long currentMillis = millis();
    if(currentMillis - lastUpdate > REFRESH_RATE){
       lastUpdate = currentMillis;
       newData = true;
    } 
    updateDisplay(newData);

    //Update heading offset
    mHeadingOffset = analogRead(HEADING_ADJUST_PIN);            // reads the value of the potentiometer (value between 0 and 1023) 
    mHeadingOffset = map(mHeadingOffset, 0, 1023, 180, -180);     // scale it to use it with the heading (value between 0 and 180) 
  
    //Adjust speed
    mSpeedCap = analogRead(SPEED_ADJUST_PIN);            // reads the value of the potentiometer (value between 0 and 1023) 
    mSpeedCap = map(mSpeedCap, 0, 1023, 0, TURN_SPEED);  
  }

  /**
  * Reades the heading from HMC5883 and update compassHeagin in degrees
  */
  void updateHeading(){
    /* Get a new sensor event */ 
    sensors_event_t event; 
    sensors_event_t accel_event;
    accel.getEvent(&accel_event);
    mag.getEvent(&event);
    
    if(!dof.magTiltCompensation(SENSOR_AXIS_Z, &event, &accel_event)){
      return;
    }
    
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
    //Adjust heading
    compassHeading += mHeadingOffset;
    if(compassHeading >= 360){
      compassHeading -= 360;
    } else if (compassHeading < 0){
      compassHeading += 360;
    }
    
    //Filter compass value to smooth out values
    compassHeading = compassFilter.getFilteredValue(compassHeading);
  }
  
  void updateDisplay(bool newData){
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
        if(mMenuPosition < MENU_LENGTH){
          mMenuPosition++;
        }
      break;
      case btnRIGHT:
        //Next Mode
        if(mMode < NUMBER_OF_MODES - 1){
           mMode++; 
        }
      break;
      case btnLEFT:
        //Last Mode
        if(mMode > 0){
           mMode--; 
        }
      break;
      case btnSELECT:
        if(mMode == MODE_MENU){
          executeCommand(mMenuPosition);
        }
      break;
      default:
        //No buttons pressed
        isPress = false;
      break;  
    }
    
    //Diplay menu if changed
    switch(mMode){
       case MODE_MENU:
        if(isPress){
          displayMenu();
          display2Menu();
        }
       break;
       case MODE_NAVIGATE:
         if(isPress || newData){
           executeCommand(CMD_NAVIGATE);
         }
       break;
       case MODE_POSITION:
         if(isPress || newData){
           executeCommand(CMD_SHOW_POSITION);
         }
       break;
    }
    
    //Short delay to prevent rebounce
    if(isPress){
      delay(250);
    }
  }
  void bootScreen(){
      lcd.clear();
      lcd.setCursor ( 0, 0 );            // go to the top left corner
      lcd.print("  Navigation "); // write this string on the top row
      lcd.setCursor ( 0, 1 );            // go to the 2nd row
      lcd.print(" Systems Ready");
     
      lcd2.begin(20,4);        // 20 columns by 4 rows on display
      lcd2.clear();
      lcd2.setBacklight(HIGH); // Turn on backlight, LOW for off
      lcd2.setCursor(0,0);
      lcd2.print("P-ath");
      lcd2.setCursor(0,1);
      lcd2.print(" I-ntelligence  ! !");
      lcd2.setCursor(0,2);
      lcd2.print("  K-ontrol      o o");
      lcd2.setCursor(0,3);
      lcd2.print("   A-ssembly    ---");
         //delay 1000);

      }


  void displayMenu(){
      lcd.clear();
      lcd.setCursor ( 0, 0 );            // go to the top left corner
      lcd.print("<  Navigation  >"); // write this string on the top row
      lcd.setCursor ( 0, 1 );            // go to the 2nd row
      lcd.print("<     Setup    >");

      }
  
  void display2Menu(){
      lcd2.clear();
      lcd2.setCursor(0,0);
      lcd2.print("Current Waypoint:");
      lcd2.print(mCurrentWayPoint);
      
      lcd2.setCursor(0,1);
      lcd2.print("   --------------");
      lcd2.setCursor(3,2);
      lcd2.print("I");
      lcd2.setCursor(6,2);
      lcd2.print(mMenu[mMenuPosition]);
      lcd2.setCursor(16,2);
      lcd2.print("I");
      lcd2.setCursor(0,3);
      lcd2.print("   --------------");
  }
  void executeCommand(int cmd){
    float heading;
    int distance;
    switch(cmd){
       case CMD_SHOW_POSITION:
         //Display 1 current position menu
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print("    Current    >");
         lcd.setCursor(0,1);
         lcd.print("    Position   >");
         
         //Display 2 current position data
         lcd2.clear();
         lcd2.setCursor(0,0);
         lcd2.print("LAT: ");
         lcd2.print(flat, 8);
         lcd2.setCursor(0,1);
         lcd2.print("LON: ");
         lcd2.print(flon, 8);
         lcd2.setCursor(0,2);
         lcd2.print("MAG: ");
         lcd2.print(compassHeading);
         lcd2.print(" (");
         lcd2.print(mHeadingOffset);
         lcd2.print(")");
         lcd2.setCursor(0,3);
         lcd2.print("SAT: ");
         lcd2.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? DEC: gps.satellites());
         //delay 1000);
        
       break;
       case CMD_SHOW_WAY_POINT:
         //Display current position
         //Display current position
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print("    Current");
         lcd.setCursor(0,1);
         lcd.print("    Waypoint");
         
         lcd2.clear();
         lcd2.setCursor(0,0);
         lcd2.print("--- Waypoint :");
         lcd2.print(mCurrentWayPoint);
         lcd2.setCursor(17,0);
         lcd2.print("---");
         lcd2.setCursor(0,1);
         lcd2.print(" LAT: ");
         lcd2.print(mLat[mCurrentWayPoint], 8);
         lcd2.setCursor(0,2);
         lcd2.print(" LON: ");
         lcd2.print(mLon[mCurrentWayPoint], 8);
         lcd2.setCursor(0,3);
         lcd2.print("--------------------");
         delay(5000);
       break;
       case CMD_NAVIGATE:
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print("<   Navigating ");
         lcd.setCursor(0,1);
         lcd.print("<   Waypoints ");
         
         lcd2.clear();
         lcd2.setCursor(0,0);
         lcd2.print("--- Waypoint: ");
         lcd2.print(mCurrentWayPoint);
         lcd2.setCursor(16,0);
         lcd2.print(" ---");
         lcd2.setCursor(0,1);
         lcd2.print("MAG:");
         lcd2.print(compassHeading);
         
         heading = getHeading(mCurrentWayPoint, flat, flon);  
         distance = getDistance(mCurrentWayPoint, flat, flon);
         
         //Check if distance is less then 2 meters
         if(distance < 2){
           //Advance to next way point
           mCurrentWayPoint++;
           lcd2.setCursor(0,3);
           lcd2.print("**Waypoint Reached**");
         } else {
           //Navigate to the way point
           if(abs(compassHeading - heading) < 5){
             lcd2.setCursor(0,3);
             lcd2.print("  Command: FORWARD");
             forward(); 
           } else if (compassHeading > heading) {
             lcd2.setCursor(0,3);
             lcd2.print("  Command: LEFT"); 
             left();
           } else {
             lcd2.setCursor(0,3);
             lcd2.print("  Command: RIGHT");
             right(); 
           }
         }
         lcd2.setCursor(11,1);
         lcd2.print("G:");
         lcd2.print(heading);
         lcd2.setCursor(0,2);
         lcd2.print("Distance: ");
         lcd2.print(formatDistance(distance));
       break;
       case CMD_ADD_WAY_POINT:
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print("     Adding");
         lcd.setCursor(0,1);
         lcd.print("    Waypoint");
         lcd2.clear();
         lcd2.setCursor(0,0);
         lcd2.print("------ Saving ------");
         lcd2.setCursor(0,1);
         lcd2.print(" LAT: ");
         lcd2.print(flat, 8);
         lcd2.setCursor(0,2);
         lcd2.print(" LON: ");
         lcd2.print(flon, 8);
         lcd2.setCursor(0,3);
         lcd2.print("-- To Waypoint :");
         lcd2.print(mCurrentWayPoint);
         lcd2.setCursor(18,3);
         lcd2.print("--");

         mLat[mCurrentWayPoint] = flat;
         mLon[mCurrentWayPoint] = flon;
         delay(5000);
       break;
       case CMD_NEXT_WAY_POINT:
         if(mCurrentWayPoint < MAX_WAYPOINTS)
           mCurrentWayPoint++;
       break;
       case CMD_LAST_WAY_POINT:
         if(mCurrentWayPoint > 0)
           mCurrentWayPoint--;
       break;
              case CMD_DELETE_WAY_POINT:
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print("    Deleting ");
                  lcd.setCursor(0,1);
         lcd.print("    Waypoint ");

         lcd2.clear();
         lcd2.setCursor(0,0);
         lcd2.print("----- Deleting -----");
         lcd2.setCursor(0,1);
         lcd2.print(" LAT: ");
         lcd2.print(mLat[mCurrentWayPoint], 8);
         lcd2.setCursor(0,2);
         lcd2.print(" LON: ");
         lcd2.print(mLon[mCurrentWayPoint], 8);
         lcd2.setCursor(0,3);
         lcd2.print("--- Waypoint :");
         lcd2.print(mCurrentWayPoint);
         lcd2.setCursor(17,3);
         lcd2.print("---");
         mLat[mCurrentWayPoint] = 0.0;
         mLon[mCurrentWayPoint] = 0.0;
         delay(5000);
       break;
       case CMD_DELETE_ALL:
       //ADD DELETE ALL FUNCTION HERE
         //mLat[mCurrentWayPoint] = clearlat;
         //mLon[mCurrentWayPoint] = clearlon;
       break;  
    }
    
  }
 
 void forward(){
   //Send data
    dataStruct.tar = 20;
    dataStruct.val = SPEED - mSpeedCap;
    etData.sendData();
    dataStruct.tar = 21;
    dataStruct.val = SPEED - mSpeedCap;
    etData.sendData();
   ST.motor(RIGHT, SPEED - mSpeedCap);
   ST.motor(LEFT, SPEED - mSpeedCap);
 }
 
 void left(){
      //Send data
    dataStruct.tar = 20;
    dataStruct.val = SPEED - mSpeedCap;
    etData.sendData();
    dataStruct.tar = 21;
    dataStruct.val = TURN_SPEED - mSpeedCap;
    etData.sendData();
   ST.motor(RIGHT, SPEED - mSpeedCap);
   ST.motor(LEFT, TURN_SPEED - mSpeedCap);
 }
 
 void right(){
      //Send data
    dataStruct.tar = 20;
    dataStruct.val = TURN_SPEED - mSpeedCap;
    etData.sendData();
    dataStruct.tar = 21;
    dataStruct.val = SPEED - mSpeedCap;
    etData.sendData();
   ST.motor(RIGHT, TURN_SPEED - mSpeedCap);
   ST.motor(LEFT, SPEED - mSpeedCap);
 }
 
 float formatDistance(int meters){
  return meters / (float) 1000;
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
      char data = mySerial.read();
      Serial.print(data);
      if (gps.encode(data))
        newData = true;
    }
    return newData;
  }
  
  /**
  * Returns distance from wayPoint in meters
  * Pass in the current Latitude and Longitude
  */
  int getDistance(int wayPoint, float currentLat, float currentLon){
    //Convert to radians
    float waypointLat = mLat[wayPoint];
    float waypointLon = mLon[wayPoint];
      
    return calc_dist(currentLat, currentLon, waypointLat, waypointLon);
  }
  
  /*************************************************************************
 * //Function to calculate the distance between two waypoints
 *************************************************************************/
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
  * Pass in the wayPoint and current lattitude and longitude
  */
  float getHeading(int wayPoint, float currentLat, float currentLon){
    //Convert everthing to radians
    float waypointLat = radians(mLat[wayPoint]);
    float waypointLon = radians(mLon[wayPoint]);
    currentLat = radians(currentLat);
    currentLon = radians(currentLon);
    
    double arg1 = sin(waypointLon-currentLon)*cos(waypointLat);
    double arg2 = cos(currentLat)*sin(waypointLat)-sin(currentLat)*cos(waypointLat)*cos(waypointLon-currentLon);
    float heading = atan2(arg1, arg2);//,2*3.1415926535;
    heading = heading*180/3.1415926535;  // convert from radians to degrees
    //Way points are backwards flip them 180
    heading += 180;
    //if the heading is negative then add 360 to make it positive
    if(heading < 0){
      heading+=360;   
    }
    
    return heading;
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



