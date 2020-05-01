//#include <OneWire.h>        // load the onewire library
#include <SimpleTimer.h>  // load the SimpleTimer library to make timers, instead of delays & too many millis statements
#include <DS1302.h>

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
  const int ledPin = A0;                       // photocell connected to analog 0 

  const int photocellPin = A1;                 // photocell connected to analog 0  
  const int enableCoopDoorMotorB = 6;          // enable motor b - pin 7
  const int directionCloseCoopDoorMotorB = 3;  // direction close motor b - pin 3
  const int directionOpenCoopDoorMotorB = 2;   // direction open motor b - pin 2
  const int lowerSwitchPin = 11;               // bottom switch is connected to pin 11
  const int topSwitchPin = 12;                 // top switch is connected to pin 12

//Not used pins
  const int feedCoopMotorA = 4;                // direction close motor b - pin 4
  const int feedCoopMotorB = 5;                // direction open motor b - pin 5
  const int feedSwitchPin = 13;                // top switch is connected to pin 13  
  const int upDoorButtonPin = 2;              // up door switch is connected to pin 2
  const int downDoorButtonPin = 3;            // down door switch is connected to pin 23 

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

// vars

  // photocell
  int photocellReading;                        // analog reading of the photocel
  int photocellReadingLevel;                   // photocel reading levels (dark, twilight, light)
 
  // sunStates
  int sunIsUp;                                  // Sun is up - calculated from time
  int sunIsDown;                                // Sun is down - calculated from time

  // Controlled the door by manual. (if the door is synchronously with automatic drive then will reinit the state variable)
  bool ManualDrive;                 // bottom switch var for to hold the switch state

  // just need 2 SimpleTimer object
  SimpleTimer duskTimer;
  SimpleTimer timeTimer;

SimpleTimer timer;
// ************************************** the setup **************************************
 
void setup(void) {
  Serial.begin(9600);

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
 
  // coop door switches
  pinMode(lowerSwitchPin, INPUT);                    // set lower switch pin as input
  pinMode(topSwitchPin, INPUT);                      // set top switch pin as input
  pinMode(photocellPin, INPUT);                      // set photocell pin as input

  // coop door switches
  pinMode(downDoorButtonPin, INPUT);                   // set down door button pin as output
  pinMode(upDoorButtonPin, INPUT);                     // set up door button pin as output

  // init driving
  ManualDrive = false;

  printTime();
  // timed actions setup
       duskTimer.setInterval(3000, readPhotoCell);      // read the photocell every 10 minutes
  /*switch (handleType) {
     case 1:
       // Timer controll
       Serial.println(" Checking doCoopDoor: every 10 minutes for the time to open or close door");
       calculateSunTimes();
       timeTimer.setInterval(600000, calculateSunTimes);      // calculate sun states every 10 minutes
       break;
     case 2:
       // Dusk controll
       Serial.println(" Checking doCoopDoor: every 10 minutes for light levels to open or close door");
       //readPhotoCell();
       //duskTimer.setInterval(600000, readPhotoCell);      // read the photocell every 10 minutes
       duskTimer.setInterval(3000, readPhotoCell);      // read the photocell every 10 minutes

       break;
     default:
       // Mixed controll
       Serial.println(" Checking doCoopDoor: every 10 minutes for times and light levels to open or close door");
       calculateSunTimes();
       timeTimer.setInterval(600000, calculateSunTimes);      // calculate sun states every 10 minutes
       readPhotoCell();
       duskTimer.setInterval(600000, readPhotoCell);      // read the photocell every 10 minutes
       break;
   }*/
  Serial.println();

}
 
// functions

void repeatMe() {
    Serial.print("Uptime (s): ");
    Serial.println(millis() / 1000);
}

  // Read the photocell state dark to light
  void readPhotoCell() { // function to be called repeatedly - per dusktimer set in setup
    photocellReading = analogRead(photocellPin);
    printTime(); 
    Serial.print("Photocel Analog Reading = ");
    Serial.print(photocellReading);
 
    //  set photocel threshholds
    if (photocellReading >= 0 && photocellReading <= 3) {
      photocellReadingLevel = 1;
      Serial.print(" - reading level:");
      Serial.println(" Dark");
    }  else if (photocellReading  >= 4 && photocellReading <= 120){
       photocellReadingLevel = 2;
       Serial.print(" - reading level:");
       Serial.println(" Twilight");
    }  else if (photocellReading  >= 125 ) {
       photocellReadingLevel = 3;
       Serial.print(" - reading level:");
       Serial.println(" Light");
    }
  }
  
  //Exemine the door switch states -> debug function
  void readSwitchState() { 
    if (digitalRead(lowerSwitchPin) == HIGH) {
      Serial.print("Chicken door in lower state  --  ");
    } else if (digitalRead(topSwitchPin) == HIGH) { 
      Serial.print("Chicken door in top state  --  ");
    } else  { 
      Serial.print("Chicken door elsewhere in middle state  --  ");
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
  void readButtonState() { 
    if (digitalRead(upDoorButtonPin) == LOW) {
      Serial.print("Pressed up button  --  ");
    } 
    if (digitalRead(downDoorButtonPin) == LOW) {
      Serial.print("Pressed down button  --  ");
    }
  }

  //Exemine the up door button state
  bool UpButtonPressed() { 
    if (digitalRead(upDoorButtonPin) == LOW) {
      Serial.println("->Button Press: Pressed up button");
      return true;
    } else {
      return false;
    }
  }

  //Exemine the down door button state
  bool DownButtonPressed() { 
    if (digitalRead(downDoorButtonPin) == LOW) {
      Serial.println("->Button Press: Pressed down button");
      return true;
    } else {
      return false;
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
      // If twilight or light
      if (photocellReadingLevel  >= 2  && !TopSwitchState()) {
        printTime();
        Serial.println("->Chicken door event: Coop Door open");
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
        closeCoopDoorMotorB();
        do {
          delay(1000);
        } while (!LowerSwitchState());
        delayedstopCoopDoorMotorB();
      }
    }
  }

  // Drive the coop door by manual
  void manualDrive_CoopDoor(){
    if (UpButtonPressed() && !TopSwitchState()) {
      ManualDrive = true;
      Serial.println("->Chicken door event: Manual Coop Door open");
      openCoopDoorMotorB();
      do {
        delay(1000);
      } while (UpButtonPressed() && !TopSwitchState());
      stopCoopDoorMotorB();
    }
    if (DownButtonPressed() && !LowerSwitchState()) {
      ManualDrive = true;
      Serial.println("->Chicken door event: Manual Coop Door close");
      closeCoopDoorMotorB();
      do {
        delay(1000);
      } while (DownButtonPressed() && !LowerSwitchState());
      delayedstopCoopDoorMotorB();
    }   
  }
 
// ************************************** the loop **************************************
void loop() {
 /* switch (handleType) {
    case 1:
      // Timer controll
      duskTimer.run();
      timeDrive_CoopDoor();
      break;
    case 2:
      // Dusk controll
      timeTimer.run();
      duskDrive_CoopDoor();
      break;
    default:
      // Mixed controll
      duskTimer.run();
      timeTimer.run();
      mixedDrive_CoopDoor();
      break;
  }

  delay(wait_time);*/
  //manualDrive_CoopDoor();
      duskTimer.run();
}
