//compiled on 1.5.1r2
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>  
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// tiny gps library code
float flat, flon;

TinyGPS gps;
SoftwareSerial mySerial(2, 255);    //used for gps rx and tx pins in use

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
#define CMD_SHOW_POSITION -2
#define CMD_NAVIGATE -1
#define CMD_SHOW_WAY_POINT 0
#define CMD_ADD_WAY_POINT 1
#define CMD_NEXT_WAY_POINT 2
#define CMD_LAST_WAY_POINT 3
int mMenuPosition = CMD_SHOW_WAY_POINT;
#define MENU_LENGTH 4
char* mMenu[]={
"Show Way Point",
"Add Way Point",
"Next Way Point",
"Last Way Point",
"Delete Way Point",
"Clear Way Points"};
#define MODE_POSITION 0
#define MODE_MENU 1
#define MODE_NAVIGATE 2
#define NUMBER_OF_MODES 3
int mMode = MODE_MENU;

//Store Waypoints
#define MAX_WAYPOINTS 20
float mLat[MAX_WAYPOINTS]; //38.628967, -90.270577 Science Center
float mLon[MAX_WAYPOINTS];
int mCurrentWayPoint = 0;

//Method definitions
void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int digits = 2);


  void setup(){
    lcd.begin(16, 2);              // start the library
    lcd.setCursor(0,0);
    lcd.print("Starting..."); // print a simple message
 
    //Set up diagnostic output
    Serial.begin(115200);
    Serial.println("Starting ...");
    
    //Set gps
    mySerial.begin(9600);

    /* Initialise the compass */
    if(!mag.begin()){
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while(1);
    }
    //Setup test waypoint
    mLat[0] = 38.628967;
    mLon[0] = -90.270577;
    
    //Show initial menu
    displayMenu();
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
    updateDisplay(newData);
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

  void displayMenu(){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(mCurrentWayPoint);
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
         lcd.print(flat, 6);
         lcd.setCursor(0,1);
         lcd.print("LON: ");
         lcd.print(flon, 6);
         //delay(1000);
       break;
       case CMD_SHOW_WAY_POINT:
         //Display current position
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print("LAT: ");
         lcd.print(mLat[mCurrentWayPoint], 6);
         lcd.setCursor(0,1);
         lcd.print("LON: ");
         lcd.print(mLon[mCurrentWayPoint], 6);
         delay(2000);
       break;
       case CMD_NAVIGATE:
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print("M:");
         lcd.print(compassHeading);
         lcd.setCursor(0,1);
         lcd.print("G:");
         lcd.print(getHeading(mCurrentWayPoint, flat, flon));
         lcd.print(" ");
         lcd.print("D:");
         lcd.print(formatDistance(getDistance(mCurrentWayPoint, flat, flon)));
         //delay(100);
       break;
       case CMD_ADD_WAY_POINT:
         mLat[mCurrentWayPoint] = flat;
         mLon[mCurrentWayPoint] = flon;
       break;
       case CMD_NEXT_WAY_POINT:
         if(mCurrentWayPoint < MAX_WAYPOINTS)
           mCurrentWayPoint++;
       break;
       case CMD_LAST_WAY_POINT:
         if(mCurrentWayPoint > 0)
           mCurrentWayPoint--;
       break;
    }
    
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

