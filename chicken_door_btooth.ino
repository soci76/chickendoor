#include <SimpleTimer.h>  // load the SimpleTimer library to make timers, instead of delays & too many millis statements
#include <DS1302.h>
#include <SoftwareSerial.h>
#include <OneWire.h> 
#include <DallasTemperature.h>

// Data wire is plugged into pin 7 on the Arduino (Thermometer pin)
#define ONE_WIRE_BUS 7

namespace {

// Set the appropriate digital I/O pin connections. These are the pin
// assignments for the Arduino as well for as the DS1302 chip. See the DS1302
// datasheet:
//
//   http://datasheets.maximintegrated.com/en/ds/DS1302.pdf
const int kCePin   = 10;  // Chip Enable
const int kIoPin   = 9;  // Input/Output
const int kSclkPin = 8;  // Serial Clock

// Create a DS1302 object.
DS1302 rtc(kCePin, kIoPin, kSclkPin);

//Print the current date and time with new line
void printlnTime() {
  // Get the current time and date from the chip.
  Time t = rtc.time();

  // Format the time and date and insert into the temporary buffer.
  char buf[50];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           t.yr, t.mon, t.date,
           t.hr, t.min, t.sec);

  // Print the formatted string to serial so we can see the time.
  Serial.println(buf);
}

//Print the current date and time without new line
void printTime() {
  // Get the current time and date from the chip.
  Time t = rtc.time();

  // Format the time and date and insert into the temporary buffer.
  char buf[50];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d  --  ",
           t.yr, t.mon, t.date,
           t.hr, t.min, t.sec);

  // Print the formatted string to serial so we can see the time.
  Serial.print(buf);
}

}  // namespace
/*
(before running the sketch, remove spaces before and after the above ">" "<" characters -  the instructables editor wouldn't publish the includes unless i added those spaces)
*/
 
/*
* Copyright 2013, David Naves (http://daveworks.net, <a rel="nofollow"> http://davenaves.com)
</a>
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 3
* of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 
* 02110-1301, USA. 
 
*/
 
 /*
* I'm hoping that if you use/modify this code, you will share your
* coop project with me and the world (pictures, whatever)
* I'm big on sharing.
* Cheers,
* //D
*/
 
//pins

  // THERMOMETER
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs) 
  OneWire oneWire(ONE_WIRE_BUS); 
  // Pass our oneWire reference to Dallas Temperature. 
  DallasTemperature sensors(&oneWire);

  // BLUETOOTH
  // arduino>>bluetooth
  // D11 (as RX)   >>>  Tx
  // D12 (as TX)  >>>  Rx
  SoftwareSerial bluetooth(11, 12); // RX, TX

  const int ledPin = A0;                       // photocell connected to analog 0 

  const int photocellPin = A1;                 // photocell connected to analog 0  
  const int enableCoopDoorMotorB = 6;          // enable motor b - pin 7
  const int directionCloseCoopDoorMotorB = 3;  // direction close motor b - pin 3
  const int directionOpenCoopDoorMotorB = 2;   // direction open motor b - pin 2
  const int lowerSwitchPin = A2;               // bottom switch is connected to pin 11
  const int topSwitchPin = A3;                 // top switch is connected to pin 12

//Not used pins
  const int feedCoopMotorA = 4;                // direction close motor b - pin 4
  const int feedCoopMotorB = 5;                // direction open motor b - pin 5
  const int feedSwitchPin = A4;                // top switch is connected to pin 13  

//consts
  //Time constants
  const int doorClosed = 20;
  const int doorOpened = 6;

  //Special constants
  const int wait_time = 5000;
  const int stop_time = 10000;

  // Door working types: 1 - controlled by time ('time'), 2 - controlled by dusk switch('dusk'), default - controlled by time and dusk switch('mixed')
  const int handleType = 2; 

  // suntimes array. Elements : day of months, sun up time at first day of month in secounds, sun down time at first day of month in minutes(times in Hungaryan region)
  const int suntimes[ 12 ][ 3 ]= { 
    { 31, 451, 962 },
    { 28, 432, 1003 },
    { 31, 387, 1046 },
    { 30, 385, 1151 },
    { 31, 329, 1193 },
    { 30, 291, 1231 },
    { 31, 289, 1244 },
    { 31, 319, 1220 },
    { 30, 360, 1167 },
    { 31, 400, 1106 },
    { 30, 385, 988 },
    { 31, 428, 955 }
  };

  //Feed hour and fed system rotation numbers
  const int feedTime = 8;
  const int feedRotation = 3;

// vars

  // photocell
  int photocellReading;                        // analog reading of the photocel
  int photocellReadingLevel;                   // photocel reading levels (dark, twilight, light)
 
  // sunStates
  int sunIsUp;                                  // Sun is up - calculated from time
  int sunIsDown;                                // Sun is down - calculated from time

  // Controlled the door by manual. (if the door is synchronously with automatic drive then will reinit the state variable)
  bool ManualDrive;                 // bottom switch var for to hold the switch state

  //Feed variables
  bool todayFeed = false;

  // just need 3 SimpleTimer object
  SimpleTimer duskTimer;
  SimpleTimer timeTimer;
  SimpleTimer feedTimer;

// ************************************** the setup **************************************
 
void setup(void) {
  //Serial.begin(9600);
  bluetooth.begin(38400);
  Serial.begin(9600);

  sensors.begin(); 

  // Initialize a new chip by turning off write protection and clearing the
  // clock halt flag. These methods needn't always be called. See the DS1302
  // datasheet for details.
  rtc.writeProtect(false);
  rtc.halt(false);

  // Make a new time object to set the date and time.
  // Sunday, September 22, 2013 at 01:38:50.
  Time t(2019, 4, 20, 19, 33, 00, Time::kThursday);

  // Set the time and date on the chip.
  rtc.time(t);
 
  //Led on
  pinMode(ledPin, INPUT);                   // set bottom switch pin as input
  analogWrite(ledPin, 255);               // activate bottom switch resistor
    
  // coop door motor
  pinMode (enableCoopDoorMotorB, OUTPUT);            // enable motor pin = output
  pinMode (directionCloseCoopDoorMotorB, OUTPUT);    // motor close direction pin = output
  pinMode (directionOpenCoopDoorMotorB, OUTPUT);     // motor open direction pin = output
  pinMode (feedCoopMotorA, OUTPUT);    // feed motor pin = output
  pinMode (feedCoopMotorB, OUTPUT);     // feed motor pin = output
 
  // coop door switches
  pinMode(lowerSwitchPin, INPUT);                    // set lower switch pin as input
  pinMode(topSwitchPin, INPUT);                      // set top switch pin as input
  pinMode(feedSwitchPin, INPUT);                     // set feed switch pin as input
  pinMode(photocellPin, INPUT);                      // set photocell pin as input

  // init driving
  ManualDrive = false;

  Serial.println("---Chicken door driver-bluetooth version---");
  Serial.println("----------------Version 7.0----------------");

  printTime();
  // timed actions setup
  feedTimer.setInterval(600000, autoFeed);      // read the photocell every 10 minutes
  switch (handleType) {
     case 1:
       // Timer controll
       Serial.println(" Checking doCoopDoor: every 10 minutes for the time to open or close door");
       calculateSunTimes();
       timeTimer.setInterval(600000, calculateSunTimes);      // calculate sun states every 10 minutes
       break;
     case 2:
       // Dusk controll
       Serial.println(" Checking doCoopDoor: every 10 minutes for light levels to open or close door");
       readPhotoCell();
       duskTimer.setInterval(600000, readPhotoCell);      // read the photocell every 10 minutes
       break;
     default:
       // Mixed controll
       Serial.println(" Checking doCoopDoor: every 10 minutes for times and light levels to open or close door");
       calculateSunTimes();
       timeTimer.setInterval(600000, calculateSunTimes);      // calculate sun states every 10 minutes
       readPhotoCell();
       duskTimer.setInterval(600000, readPhotoCell);      // read the photocell every 10 minutes
       break;
   }
  Serial.println();

}
 
// functions

  // Read the photocell state dark to light
  void readPhotoCell() { // function to be called repeatedly - per dusktimer set in setup
        Serial.print("Read begin");
    photocellReading = analogRead(photocellPin);
    printTime(); 
    Serial.print("Photocel Analog Reading = ");
    Serial.print(photocellReading);
 
    //  set photocel threshholds
    //0-5 or 10 dark
    if (photocellReading >= 0 && photocellReading <= 49) {
      photocellReadingLevel = 1;
      Serial.print(" - reading level:");
      Serial.println(" Dark");
    //10-200 twilight
    }  else if (photocellReading  >= 50 && photocellReading <= 249){
       photocellReadingLevel = 2;
       Serial.print(" - reading level:");
       Serial.println(" Twilight");
    }  else if (photocellReading  >= 250 ) {
       photocellReadingLevel = 3;
       Serial.print(" - reading level:");
       Serial.println(" Light");
    }
  }
  
  //Exemine the door switch states -> debug function
  void readSwitchesState() { 
    printTime();
    if (digitalRead(lowerSwitchPin) == HIGH) {
      Serial.println("-> Chicken door in lower state");
    } else if (digitalRead(topSwitchPin) == HIGH) { 
      Serial.println("-> Chicken door in top state");
    } else  { 
      Serial.println("-> Chicken door elsewhere in middle state");
    }
  }
  
  //Exemine the top door switch state
  bool TopSwitchState() { 
    if (digitalRead(topSwitchPin) == HIGH) {
      Serial.println("->Top State Check: Chicken door in top state");
     return true;
    } else {
      Serial.println("->Top State Check: Chicken door not in top state");
      return false;
    }
  }

  //Exemine the lower door switch state
  bool LowerSwitchState() { 
    if (digitalRead(lowerSwitchPin) == HIGH) {
      Serial.println("->Lower State Check: Chicken door in lower state");
      return true;
    } else {
      Serial.println("->Lower State Check: Chicken door not in lower state");
      return false;
    }
  }

  //Exemine the door driver button states -> debug function
  char readButtonState() { 
    if(bluetooth.available() > 0){     
      return bluetooth.read();
    }
  }

  // Based on the array, determine the sun up or sun down times of the day in minutes
  int CalculateTime(int actmonth, int actday, int type) {
    int nextmonth = actmonth + 1;
    int actminutes;
    int nextminutes;
    int days;
    int calcminutes;
        
    if (nextmonth == 13) {
      nextmonth = 1;
    }

    days = suntimes[actmonth-1][0];
    switch (type) {
      case 1:
        // Sun up time
        actminutes = suntimes[actmonth-1][1];
        nextminutes = suntimes[nextmonth-1][1];
        break;
      case 2:
        // Sun down time
        actminutes = suntimes[actmonth-1][2];
        nextminutes = suntimes[nextmonth-1][2];
        break;
      default:
        // False values
        actminutes = 700;
        nextminutes = 700;
        break;
    }
    calcminutes = actminutes + (((nextminutes - actminutes) / days) * actday);
    return calcminutes;
    
  }

  // Change the actual hour and minutes to minutes data
  int ChangetoMinutes(int acthour, int actminutes) {
    int calcminutes;
      
    calcminutes = (acthour * 60) + actminutes;
    return calcminutes;
      
  }

  // Calculate the sun state dates sun is up state or down state
  void calculateSunTimes() { // function to be called repeatedly - per timetimer set in setup
    Time t = rtc.time();
    int currentinmin = ChangetoMinutes(t.hr, t.min);
    int openTime = CalculateTime(t.mon, t.date, 1);
    int closeTime = CalculateTime(t.mon, t.date, 2);             
    //Log information
    int openTime_hour = openTime / 60;
    int openTime_minutes = openTime - (openTime_hour * 60);
    int closeTime_hour = closeTime / 60;
    int closeTime_minutes = closeTime - (closeTime_hour * 60);
    Serial.print("Today : ");
    printTime(); 
    Serial.print("sun up time : ");
    Serial.print(openTime_hour);
    Serial.print(":");
    Serial.print(openTime_minutes); 
    Serial.print(" - sun down time - ");  
    Serial.print(closeTime_hour);
    Serial.print(":");
    Serial.print(closeTime_minutes);  
      
    if (currentinmin >= openTime && currentinmin <= closeTime) {
      Serial.println("-> Sun is up door Up Time");
      sunIsUp = true;
      sunIsDown = false;
    } else if (currentinmin < openTime || currentinmin > closeTime) {
      Serial.println("-> Sun is down door Down Time");
      sunIsDown = true;  
      sunIsUp = false;
    }
  }
 
  // Stop the coop door motor
  void stopCoopDoorMotorB(){
    Serial.println("->Chicken motor event: Stop motor!");
    digitalWrite (directionCloseCoopDoorMotorB, LOW);        // turn off motor close direction
    digitalWrite (directionOpenCoopDoorMotorB, LOW);         // turn on motor open direction
    analogWrite (enableCoopDoorMotorB, 0);                   // enable motor, 0 speed
  }

  // Delayed stop the coop door motor - the locks should have time to lock
  void delayedstopCoopDoorMotorB(){
    delay(stop_time);
    Serial.println("->Chicken motor event: Stop motor!");
    digitalWrite (directionCloseCoopDoorMotorB, LOW);        // turn off motor close direction
    digitalWrite (directionOpenCoopDoorMotorB, LOW);         // turn on motor open direction
    analogWrite (enableCoopDoorMotorB, 0);                   // enable motor, 0 speed
  }
 
  // Close the coop door motor (motor dir close = clockwise) 
  void closeCoopDoorMotorB() {  
     Serial.println("->Chicken motor event: Drive down motor!");
     digitalWrite (directionCloseCoopDoorMotorB, HIGH);       // turn on motor close direction
     digitalWrite (directionOpenCoopDoorMotorB, LOW);         // turn off motor open direction
     analogWrite (enableCoopDoorMotorB, 255);                 // enable motor, full speed 
  }
 
  // Open the coop door (motor dir open = counter-clockwise)
  void openCoopDoorMotorB() { 
    Serial.println("->Chicken motor event: Drive up motor!");
    digitalWrite(directionCloseCoopDoorMotorB, LOW);    // turn off motor close direction
    digitalWrite(directionOpenCoopDoorMotorB, HIGH);    // turn on motor open direction
    analogWrite(enableCoopDoorMotorB, 255);           // enable motor, full speed
   }

  // Drive the coop door by time base
  void timeDrive_CoopDoor(){
    // Reinit manual driving stae when door is in synchronous state
    if ((sunIsUp && TopSwitchState()) || (sunIsDown && LowerSwitchState())) {
      ManualDrive = false;
    }
    
    if (!ManualDrive) {
      // If sun is up
      if (sunIsUp && !TopSwitchState()) {
        printTime();
        Serial.println("->Chicken door event: Coop Door open");
        if(bluetooth.available() > 0){  
          bluetooth.println("->Chicken door event: Coop Door open");
        }
        openCoopDoorMotorB();
        do {
          delay(1000);
        } while (!TopSwitchState());
        stopCoopDoorMotorB();
      }

      if (sunIsDown && !LowerSwitchState()) {
        // If sun is down
        printTime();
        Serial.println("->Chicken door event: Coop Door close");
        if(bluetooth.available() > 0){          
          bluetooth.println("->Chicken door event: Coop Door close");
        }
        closeCoopDoorMotorB();
        do {
          delay(1000);
        } while (!LowerSwitchState());
        delayedstopCoopDoorMotorB();
      }
    }
  }

  // Drive the coop door by dusk controller base
  void duskDrive_CoopDoor(){
    // Reinit manual driving stae when door is in synchronous state
    if ((photocellReadingLevel  >= 2 && TopSwitchState()) || (photocellReadingLevel  == 1 && LowerSwitchState())) {
      ManualDrive = false;
    }

    if (!ManualDrive) {
      // If (twilight or [>=2]) light
      if (photocellReadingLevel  > 2  && !TopSwitchState()) {
        printTime();
        Serial.println("->Chicken door event: Coop Door open");
        if(bluetooth.available() > 0){  
          bluetooth.println("->Chicken door event: Coop Door open");
        }
        openCoopDoorMotorB();
        do {
          delay(1000);
        } while (!TopSwitchState());
          stopCoopDoorMotorB();
      }
      // If dark
      if (photocellReadingLevel  == 1 && !LowerSwitchState()) {
        printTime();
        Serial.println("->Chicken door event: Coop Door close");
        if(bluetooth.available() > 0){  
          bluetooth.println("->Chicken door event: Coop Door close");
        }
        closeCoopDoorMotorB();
        do {
          delay(1000);
        } while (!LowerSwitchState());
        delayedstopCoopDoorMotorB();
      }
    }
  }

  // Drive the coop door by dusk controller and time base  
  void mixedDrive_CoopDoor(){
    // Reinit manual driving stae when door is in synchronous state
    if ((sunIsUp && photocellReadingLevel  >= 2 && TopSwitchState()) || (sunIsDown && photocellReadingLevel  == 1 && LowerSwitchState())) {
      ManualDrive = false;
    }
    
    if (!ManualDrive) {
      // If twilight or light and sun is up
      if (sunIsUp && photocellReadingLevel  >= 2  && !TopSwitchState()) {
        printTime();
        Serial.println("->Chicken door event: Coop Door open");
        if(bluetooth.available() > 0){    
          bluetooth.println("->Chicken door event: Coop Door open");
        }
        openCoopDoorMotorB();
        do {
          delay(1000);
        } while (!TopSwitchState());
        stopCoopDoorMotorB();
      }
      // If dark and sun is down
      if (sunIsDown && photocellReadingLevel  == 1 && !LowerSwitchState()) {
        printTime();
        Serial.println("->Chicken door event: Coop Door close");
        if(bluetooth.available() > 0){  
          bluetooth.println("->Chicken door event: Coop Door close");
        }
        closeCoopDoorMotorB();
        do {
          delay(1000);
        } while (!LowerSwitchState());
        delayedstopCoopDoorMotorB();
      }
    }
  }

  // Drive the coop door by manual
  void manualDrive_CoopSystem(){
    
    switch (readButtonState()) {
      case 'u':
        if (!TopSwitchState()) {
          ManualDrive = true;
          Serial.println("->Chicken door event: -u- pressed - Manual Coop Door open");
          if(bluetooth.available() > 0){      
            bluetooth.println("->Chicken door event: -u- pressed - Manual Coop Door open");
          }
          openCoopDoorMotorB();
          do {
            delay(1000);
          } while (!TopSwitchState());
          stopCoopDoorMotorB();
        }
        break;
      case 'd':
        if (!LowerSwitchState()) {
          ManualDrive = true;
          Serial.println("->Chicken door event: -d- pressed - Manual Coop Door close");
          if(bluetooth.available() > 0){  
            bluetooth.println("->Chicken door event: -d- pressed - Manual Coop Door close");
          }
          closeCoopDoorMotorB();
          do {
            delay(1000);
          } while (!LowerSwitchState());
          delayedstopCoopDoorMotorB();
        }   
        break;
      case 'i':
          float temperature = getTemperature();
          printTime();
          Serial.print("-> Temperature is: "); 
          Serial.println(temperature);
          if(bluetooth.available() > 0){  
            bluetooth.print(" ->Temperature is: "); 
            bluetooth.println(temperature);
          }
          switch (handleType) {
            case 1:
              // Timer controll
              if (sunIsUp) {
                Serial.print("-> The sun is up at this moment."); 
                if(bluetooth.available() > 0){  
                  bluetooth.print("-> The sun is up at this moment."); 
                }  
              } else if (sunIsDown) {
                Serial.print("-> The sun down up at this moment.");
                if(bluetooth.available() > 0){  
                  bluetooth.print("-> The sun is up at this moment.");  
                }
              }
              break;
            case 2:
              // Dusk controll
              Serial.print("-> Reading level(1-dark, 2-twilight, 3-light): ");
              Serial.println(photocellReadingLevel);
              if(bluetooth.available() > 0){  
                bluetooth.print("-> Reading level(1-dark, 2-twilight, 3-light): ");
                bluetooth.println(photocellReadingLevel);              
              }
              break;
            default:
              // Mixed controll
              Serial.print("-> Reading level(1-dark, 2-twilight, 3-light): ");
              Serial.println(photocellReadingLevel);
              if(bluetooth.available() > 0){  
                bluetooth.print("-> Reading level(1-dark, 2-twilight, 3-light): ");
                bluetooth.println(photocellReadingLevel);
              }
              if (sunIsUp) {
                Serial.print("-> The sun is up at this moment."); 
                if(bluetooth.available() > 0){  
                  bluetooth.print("-> The sun is up at this moment."); 
                }
              } else if (sunIsDown) {
                Serial.print("-> The sun down up at this moment.");
                if(bluetooth.available() > 0){  
                  bluetooth.print("-> The sun is up at this moment.");  
                }
              }  
              break;
          }
          break;
        case 't':
          break;
        default:
          // statements
          break;
     }

   /*  if(bluetooth.available() > 0){  
       bluetooth.flush(); 
     }*/       
  }

  // Get the actual temperature
  float getTemperature() {
    sensors.requestTemperatures(); // Send the command to get temperature readings 
    return sensors.getTempCByIndex(0);
   
  }

  // Close the coop feed motor (motor dir close = clockwise) 
  void rotateFeedMotor() {  
     Serial.println("->Feed motor event: Rotate feed motor!");
     digitalWrite (feedCoopMotorA, HIGH);       // turn on motor close direction
     digitalWrite (feedCoopMotorB, LOW);         // turn off motor open direction
     analogWrite (enableCoopDoorMotorB, 255);                 // enable motor, full speed 
  }

    // Stop the coop feed motor
  void stopFeedMotorB(){
    Serial.println("->Feed motor event: Stop feed motor!");
    digitalWrite (feedCoopMotorA, LOW);        // turn off motor close direction
    digitalWrite (feedCoopMotorB, LOW);         // turn on motor open direction
    analogWrite (enableCoopDoorMotorB, 0);                   // enable motor, 0 speed
  }

  //Exemine the feed switch state
  bool FeedSwitchState() { 
    if (digitalRead(feedSwitchPin) == HIGH) {
      Serial.println("->Feed State Check: Chicken feed in switch state");
      return true;
    } else {
      Serial.println("->Feed State Check: Chicken door not in top state");
      return false;
    }
  }

  // Initialize chicken feed
  void feedNull(){
    Time t = rtc.time();

    if (t.hr == 1 && todayFeed) {
      todayFeed = false;
    }
  }

  // Drive the feed motor by time
  void autoFeed(){
    Time t = rtc.time();
    int rotation = 0;;

    if (t.hr == feedTime && !todayFeed) {
      todayFeed = true;
      printTime();
      Serial.println("->Chicken feed event: Feed chicken");
      if(bluetooth.available() > 0){  
        bluetooth.println("->Chicken feed event: Feed chicken");
      }
      rotateFeedMotor();
      do {
        if (FeedSwitchState()) {
          rotation = rotation + 1;
        }
        delay(1000);
      } while (rotation != feedRotation);
      stopFeedMotorB();
    }
  
  }
 
// ************************************** the loop **************************************
void loop() {  
  switch (handleType) {
    case 1:
      // Timer controll
      timeTimer.run();
      timeDrive_CoopDoor();
      break;
    case 2:
      // Dusk controll
      duskTimer.run();
      duskDrive_CoopDoor();
      break;
    default:
      // Mixed controll
      duskTimer.run();
      timeTimer.run();
      mixedDrive_CoopDoor();
      break;
  }

  delay(wait_time);
  manualDrive_CoopSystem();

}
