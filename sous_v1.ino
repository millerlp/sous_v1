
#include <LiquidCrystal.h>
#include <PID_v1.h> //from https://github.com/br3ttb/Arduino-PID-Library/

//*****************************************************************
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(3, 4, 5, 6, 7, 8); //(RS,EN,DB4,DB5,DB6,DB7)
//*****************************************************************

//*****************************************
// PID setup
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,200,100,10, REVERSE);

int WindowSize = 5000;
unsigned long windowStartTime; 
//*****************************************



#define airTempPin 0 //analog 0
#define meatTemp1 1 //analog 1
#define meatTemp2 2 //analog 2
#define meatTemp3 3 //analog 3

#define button1 A4 //analog 4, used as digital in for button input
#define ssrPin A5 //analog 5, used as digital out for SSR

volatile int lowTemp = 130; //initial lowTemp limit for heating element control
volatile int highTemp = 132; //initial highTemp limit for heating element control
volatile boolean pwrState = false; //initial power on state

//Temperature conversion constants
const float x3 = 0.000000535; //3rd order constant
const float x2 = 0.0007; //2nd order constant
const float x1 = 0.4849; //1st order constant
const float inter = 12.61; //intercept for thermistor temperature conversion
const int lowlim = 20; //lower temperature limit, values below this are probably due to a 
//disconnected temperature probe

//-----------------------------------------
//Setup array for holding temperatures
const int chs = 4; // # of channels to input
const int numReadings = 10;
const int reps = 9; //number of temp readings to average per channel (count from zero)
long tempInArray[numReadings][chs]; //create array to hold analog values
int index = 0; //index value to iterate through in main loop
long total[chs]; //array of totals, one value per channel
float averages[chs]; //array of averages, one value per channel
//-------------------------------------------
unsigned long loopTime ;  // timer for display updates
unsigned long loopTime2 ; // timer for PID loop
//**************************************************************************
//**************************************************************************
void setup()
{
  analogReference(DEFAULT); //5V reference
  pinMode(button1, INPUT); //button for selecting options
  pinMode(ssrPin, OUTPUT); //pin to toggle solid state relay (SSR)
  digitalWrite(ssrPin, LOW); //turn off power to SSR initially
  pwrState = false;

  // set up the LCD's number of columns and rows: 
  lcd.begin(16,2); //initialize LCD output
  lcd.clear();

  lowTemp = lowTempFunc(); //call lowTempFunc sub-function and return value
 
  //-------------------------
  // PID initialization
  Setpoint = lowTemp;
  windowStartTime = millis();
  myPID.SetOutputLimits(0,WindowSize);
  // Turn the PID on
  myPID.SetMode(AUTOMATIC);
  //-------------------------
  // initialize analog temperature value array
  for (int channel = 0; channel < chs; channel++) { 
    for (int i = 0; i < numReadings; i++) {
      tempInArray[i][channel] = 0; //initialize all values to 0
    }
  }
  lcd.clear();
  loopTime = millis();
  loopTime2 = millis();
} //end of setup loop, move on to main loop()

void loop() {

  //**************************************************************************
  //Start by reading temperature values on each analog input
  //first remove the old value in this row of tempInArray from the running total
  for (int channel = 0; channel < chs; channel++) {
    total[channel] = total[channel] - tempInArray[index][channel];
  }

  //read temperature on each channel airTempPin, meatTemp1, meatTemp2
  int channel = 0;
  analogRead(airTempPin); //take 1 reading to throw away
  delay(20);
  tempInArray[index][channel] = 1023 - analogRead(airTempPin);
  delay(20);
  channel = 1;
  analogRead(meatTemp1);
  delay(20);
  tempInArray[index][channel] = 1023 - analogRead(meatTemp1);
  delay(20);
  channel = 2;
  analogRead(meatTemp2);
  delay(20);
  tempInArray[index][channel] = 1023 - analogRead(meatTemp2);
  channel = 3;
  analogRead(meatTemp3);
  delay(20);
  tempInArray[index][channel] = 1023 - analogRead(meatTemp3);

  for (int channel = 0; channel < chs; channel++) {
    total[channel] = total[channel] + tempInArray[index][channel];
    averages[channel] = total[channel] / numReadings; //calculate each average
    //convert average ADC value to temperature in Fahrenheit using pre-determined conversion equation
    averages[channel] = (x3 * pow(averages[channel],3)) - (x2 * pow(averages[channel],2)) + (x1 * averages[channel]) + inter;
  }
  index = index + 1;
  if (index >= numReadings) {
    index = 0;
  }

  // Try to update PID routine every 200ms
  if (millis() - loopTime2 > 200) {
    loopTime2 = millis(); // update timer
    Input = (double) averages[0];
    myPID.Compute();
    /************************************************
     * Turn the output pin on/off based on pid output
     ************************************************/
    if( (millis() - windowStartTime) > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if( Output < (millis() - windowStartTime) ) {
      digitalWrite(ssrPin,HIGH);
//      digitalWrite(led,HIGH);
      pwrState = true;
    }
    else {
      digitalWrite(ssrPin,LOW);
//      digitalWrite(led,LOW);
      pwrState = false;
    } 
  }

  //Regulate power to bbq by checking air temperature against
  //low and high temperature limits.
  //  if (averages[0] < lowTemp) { //if air temp is below lower limit
  //    digitalWrite(ssrPin, 1); //turn on solid state relay
  //    pwrState = true;
  //  } else if (averages[0] > highTemp) {
  //    digitalWrite(ssrPin, 0); //turn off solid state relay
  //    pwrState = false;
  //  } 
  //Note that when the air temp is between lowTemp and highTemp nothing is done,
  //so as to let the air temperature heat or cool through the temperature window
  //before changing the state of the solid state relay. This provides some hystersis
  //to avoid rapid switching of the heating element. 

  //*********************************
  //Output temperatures to LCD screen
  //The various delays in here seem to be necessary for keeping output
  //properly formatted when hooked up to the electrically noisy smoker
  if (millis() - loopTime > 1000) {
    loopTime = millis();
    lcd.home();
    lcd.print("H20:");
    delay(10);
    if (int(averages[0]) < lowlim) {
      lcd.print("NA "); 
    } 
    else if (int(averages[0]) > lowlim & int(averages[0]) < 100) {
      lcd.print(" "); //pad with a space
      lcd.print(int(averages[0]));
    } 
    else {
      lcd.print(int(averages[0]));
    }
    lcd.print("F "); 
    delay(10);
    ///////////////////////////
    lcd.setCursor(9,0);
    lcd.print("1:");
    delay(10);
    if (int(averages[1]) < lowlim) {
      lcd.print("NA ");
    } 
    else if (int(averages[1]) > lowlim & int(averages[1]) < 100) {
      lcd.print(" "); //pad with a space
      lcd.print(int(averages[1]));
    } 
    else {
      lcd.print(int(averages[1]));
    }
    lcd.print("F");
    delay(10);
    ///////////////////////
    lcd.setCursor(0,1); //start of 2nd row
    lcd.print("  2:");
    delay(10);
    if (int(averages[2]) < lowlim) {
      lcd.print("NA "); 
    } 
    else if (int(averages[2]) > lowlim & int(averages[2]) < 100) {
      lcd.print(" "); //pad with a space
      lcd.print(int(averages[2]));
    } 
    else {
      lcd.print(int(averages[2]));
    }
    delay(10);
    /////////////////////// 
    lcd.setCursor(7,1);
    delay(10);
    lcd.print("F 3:");
    if (int(averages[3]) < lowlim) {
      lcd.print("NA "); 
    } 
    else if (int(averages[3]) > lowlim & int(averages[3]) < 100) {
      lcd.print(" "); //pad with a space
      lcd.print(int(averages[3]));
    } 
    else {
      lcd.print(int(averages[3]));
    }
    delay(10);
    lcd.print("F");
    delay(10);
    lcd.setCursor(15,1);
    if (pwrState) { //if pwrState is true
      lcd.print("x");
    } 
    else {
      lcd.print(" ");
    }


  }  


} //end of main loop


//**********************************************************************************
// lowTempFunc subroutine. This lets the user choose the low temperature limit for 
// the heating element of the bbq. This uses input from button1 and returns a value
// 'lowTemp' to the setup loop.
int lowTempFunc() { //lowTempFunc will return an integer when called
  int buttonValue1;
  int buttonValue2;
  int buttonState;
  //int minutes;
  long startTime = millis(); //get starting time for this loop
  lcd.clear();
  lcd.print("Choose low temp");
  delay(500);
  lcd.setCursor(0,1);
  lcd.print(lowTemp);
  lcd.setCursor(3,1);
  lcd.print("F");
  buttonState = digitalRead(button1); //get current state of button (should be HIGH)
  while (millis() <= startTime + 3000) //while current millis is less than 3sec from startTime
  {
    buttonValue1 = digitalRead(button1);
    delay(10); //perform a crude debounce by checking button twice over 10ms
    buttonValue2 = digitalRead(button1);
    if (buttonValue1 == buttonValue2) {
      if (buttonValue1 != buttonState) { //make sure button state has changed
        if (buttonValue1 == LOW) { //if button is pressed
          if (lowTemp >= 130 & lowTemp < 190) {
            lowTemp = lowTemp + 2;
          } 
          else if(lowTemp >= 190) {
            lowTemp = 130;
          }
          lcd.setCursor(0,1);
          lcd.print("   ");
          lcd.setCursor(0,1);
          lcd.print(lowTemp);
          lcd.setCursor(3,1);
          lcd.print("F");
          startTime = millis(); //update startTime to give user more time 
          //to choose another value
        }
      }
      buttonState = buttonValue1; //update buttonState so that only changes
      //in button status are registered
    }
  }
  lcd.setCursor(6,1);
  lcd.print("Storing");
  delay(400);
  for (int i = 0; i <= 2; i++) {
    lcd.print(".");
    delay(350);
  }
  return lowTemp;
}
//*********************************************************************************
// highTempFunc subroutine. This lets the user choose the low temperature limit for 
// the heating element of the bbq. This uses input from button1 and returns a value
// 'lowTemp' to the setup loop.
//int highTempFunc(int lowT) { //highTempFunc will return an integer when called
//  lowT = lowT + 2; //increment low Temp limit value
//  highTemp = lowT; //start highTemp at value of lowT
//  int buttonValue1;
//  int buttonValue2;
//  int buttonState;
//  long startTime = millis(); //get starting time for this loop
//  lcd.clear();
//  lcd.print("Choose high temp");
//  delay(500);
//  lcd.setCursor(0,1);
//  lcd.print(lowT);
//  lcd.setCursor(3,1);
//  lcd.print("F");
//  buttonState = digitalRead(button1); //get current state of button (should be HIGH)
//  while (millis() <= startTime + 3000) //while current millis is less than 3sec from startTime
//  {
//    buttonValue1 = digitalRead(button1);
//    delay(10); //perform a crude debounce by checking button twice over 10ms
//    buttonValue2 = digitalRead(button1);
//    if (buttonValue1 == buttonValue2) {
//      if (buttonValue1 != buttonState) { //make sure button state has changed
//        if (buttonValue1 == LOW) { //if button is pressed
//          if (highTemp >= lowT & highTemp < 192) {
//            highTemp = highTemp + 2; //raise upper limit
//          } else if(highTemp >= 192) {
//            highTemp = lowT;  //recycle back to lowest limit
//          }
//          lcd.setCursor(0,1); //set cursor at start of 2nd row
//          lcd.print("   "); //clear current temp
//          lcd.setCursor(0,1); //reset cursor position
//          lcd.print(highTemp); //display current highTemp value
//          lcd.setCursor(3,1); //move cursor
//          lcd.print("F");
//          startTime = millis(); //update startTime to give user more time 
//                                //to choose another value
//        }
//      }
//      buttonState = buttonValue1; //update buttonState so that only changes
//                                  //in button status are registered
//    }
//  }
//  lcd.setCursor(6,1);
//  lcd.print("Storing");
//  delay(400);
//  for (int i = 0; i <= 2; i++) {
//    lcd.print(".");
//    delay(350);
//  }
//  return highTemp;
//}


