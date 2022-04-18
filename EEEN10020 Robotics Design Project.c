/*Aidan, Leo, Jennifer (Team 8)
 * 13-04-22
 * This is the final code for our robot. Its functionality include, travelling on desired heading
 * P-controller and interupt counter. It uses the Nine Axes Motion sensor for heading. It goes along
 * a predefined path that we measured in the competition room.
 */


#include <LCD16x2.h> //allows us to use the little LCD shield at the top
#include <Wire.h> //allows us to communicate with the LCD shield using the I2C protocol
#include <NineAxesMotion.h>
// To allow this code to work, you will need to add the 'EnableInterrupt' library. You can do this easily using the menus Sketch -> Include Library -> Manage Libraries...
#include <EnableInterrupt.h> //allows us to use interrupts on any pin

NineAxesMotion mySensor;

LCD16x2 lcd;

int E1 = 5;     //Port for M1 Speed Control
int E2 = 6;     //Port for M2 Speed Control

int M1 = 4;    //Port for M1 Direction Control
int M2 = 7;    //Port for M2 Direction Control

volatile int CycleCount = 0; // use volatile for shared variables
volatile int MatchLength = 180; // 180 seconds is 3 minutes
volatile int count;
volatile int endofloop;

void setup() {
 
  // Put your normal setup code here
  // put your setup code here, to run once:
  int buttons;
  float positions[8][2];

 
  //Peripheral Initialization
  Serial.begin(9600);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  Wire.begin();

  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //NDOF = 9 Degrees of Freedom Sensor (Other operatin modes are available)
  mySensor.setUpdateMode(AUTO);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor

  pinMode(E1,OUTPUT); //Set up motor pins
  pinMode(E2,OUTPUT);
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(4, INPUT_PULLUP); //switch
  enableInterrupt((4), counter, HIGH);



  lcd.lcdClear();
   
  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite("Press white key");

  buttons = lcd.readButtons(); //Sample the state of the large white buttons

while(buttons == 15) // A value of 15 will only be seen if no buttons are pressed
{
  buttons = lcd.readButtons(); //Sample the state of the large white buttons
}


lcd.lcdClear();

arm_and_start(); //call this special function at the end of your setup to run the arming procedure
}

void counter(){
  if((millis()- endofloop)> 5){
    count = count + 1;
  }
  endofloop = millis();
}

float headingDif(float heading1, float heading2){ //heading 1 is current, 2 is desired. Negative means you want to turn CCW
  float answer = heading2 - heading1;
  float answer1;
 
  if(answer>=0){
    answer1 = answer-360;
  }
  else{
    answer1 = answer + 360;
  }

  if(abs(answer1)< abs(answer)){
    return(answer1);
  }
 
  else{
    return(answer);
  }
}

float headingAdd(float heading1, float heading2){ //heading 1 is current, 2 is desired. Negative means you want to turn CCW
  float answer = heading2 + heading1;
  float answer1;
 
  if(answer>=0){
    answer1 = answer-360;
  }
  else{
    answer1 = answer + 360;
  }

  if(abs(answer1)< abs(answer)){
    return(answer1);
  }
 
  else{
    return(answer);
  }
}


void loop() {

// Put your normal loop code here
   
  float desired_heading = mySensor.readEulerHeading(); //read all the data we want from the sensor into our variables
  float kp = 50; //5
  float ki = 0.00;
  float accumulated_error = 0;
  float current_error = 0;
  float endofloop;
  float current_heading = mySensor.readEulerHeading();

  while(count< 343){ //go straight for 343 clicks
    current_heading = mySensor.readEulerHeading();
   
    current_error = headingDif(current_heading, desired_heading);
   
    accumulated_error = accumulated_error + (current_error*(millis() - endofloop)/1000);
   
    float response = ki* accumulated_error +kp*current_error;
   
    GoForwardWithTrim(60, -1 * response);

   
  }

  desired_heading = headingAdd(current_heading, -90); //turn left 90 degrees
  while(count< 579){
    current_heading = mySensor.readEulerHeading();

    current_error = headingDif(current_heading, desired_heading);
   
    accumulated_error = accumulated_error + (current_error*(millis() - endofloop)/1000);
   
    float response = ki* accumulated_error +kp*current_error;
   
    GoForwardWithTrim(60, -1 * response);
    lcd.lcdClear();
    lcd.lcdWrite(response);
  }

  desired_heading = headingAdd(current_heading, -100); //turn left 100 degrees
 
  while(true){
    current_heading = mySensor.readEulerHeading();

    current_error = headingDif(current_heading, desired_heading);
   
    accumulated_error = accumulated_error + (current_error*(millis() - endofloop)/1000);
   
    float response = ki* accumulated_error +kp*current_error;
   
    GoForwardWithTrim(60, -1 * response);
    lcd.lcdClear();
    lcd.lcdWrite(response);
  }

}



//Don't modify the standard startup and shutdown code below this line!

void arm_and_start() {

int buttons = 0;

float timeleft;

long countdown_int;

long fudge = 0;
bool mode = 0;
long timeflipped = 0;

  Wire.begin();
   
  lcd.lcdClear();
  lcd.lcdSetBlacklight(200);

  lcd.lcdGoToXY(1,2);
  lcd.lcdWrite("v");

  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite("SYNC ");

  lcd.lcdGoToXY(12,1);
  lcd.lcdWrite(" SYNC");

  lcd.lcdGoToXY(4,2);
  lcd.lcdWrite("[--set--]");

 
  buttons = lcd.readButtons(); //Sample the state of the large white buttons

do
{
  buttons = lcd.readButtons(); //Sample the state of the large white buttons


  if (buttons == 14) //leftmost button depressed
  {
  fudge = fudge + 450;
  }

  buttons = lcd.readButtons(); //Sample the state of the large white buttons

  if (buttons == 9 && (millis() - timeflipped) > 250) //both middle buttons depressed
  {
  mode = !mode;
  timeflipped = millis();

        if(!mode) //syncing safe mode
        {
          lcd.lcdSetBlacklight(100);
       
         lcd.lcdGoToXY(1,1);
         lcd.lcdWrite("SYNC ");
       
          lcd.lcdGoToXY(12,1);
          lcd.lcdWrite(" SYNC");
       
          lcd.lcdGoToXY(4,2);
          lcd.lcdWrite("[--set--]");
         
        }
        else if(mode) //armed mode
        {
          lcd.lcdSetBlacklight(400);
          lcd.lcdGoToXY(1,1);
          lcd.lcdWrite("ARMED");
       
         lcd.lcdGoToXY(12,1);
         lcd.lcdWrite("ARMED");
       
          lcd.lcdGoToXY(4,2);
          lcd.lcdWrite("[-disarm-]");
        }
         
  }


countdown_int = (millis() + fudge) % 10000;

countdown_int = 9998 - countdown_int;

  lcd.lcdGoToXY(7,1);
  lcd.lcdWrite((float) countdown_int/1000, 2);
 
}
while(!(mode == 1 && countdown_int <= 750));

  lcd.lcdClear();
  lcd.lcdSetBlacklight(200);
  lcd.lcdGoToXY(6,2);
  lcd.lcdWrite("Begin!");

//Now set up an interupt to trigger after the appropriate number of minutes to shut down the robot
 
 enableInterrupt(11, CheckCycle, RISING); //Attach an interrupt to pin 11
 tone(11, 31); //Output a 31 Hz square wave to pin 11 to trigger this interrupt
}

void GoForwardWithTrim(int basespeed, float steering_trim){ // basespeed is between 0 and 100. steering_trim positive creates veer to right
 basespeed = basespeed * 255/100;
 float rightspeed = basespeed + steering_trim;
 float leftspeed = 0.73*basespeed - steering_trim;
 if((rightspeed) >= 100){
    rightspeed = 100;
 }
 if((rightspeed) <= 0){
    rightspeed = 0;
 }
 if((leftspeed) <= 0){
    leftspeed = 0;
 }
  if((leftspeed) >= 100){
    leftspeed = 100;
 }
 analogWrite (E1,rightspeed); //Set M1 speed Right
 digitalWrite(M1,HIGH); //Set M1 direction
 analogWrite (E2,leftspeed); //Set M2 speed Left
 digitalWrite(M2,HIGH); //Set M1 direction

 
}

void CheckCycle(void)
{
    CycleCount = CycleCount + 1;  //Check how many times we've been here
   
  if(CycleCount == 31 * MatchLength)
  {
  //turn off motors
   analogWrite (E1,0); //Turn off M1
   analogWrite (E2,0); //Turn off M2
     
      while(1) //loop forever to shut down Arduino
      {
      }
  }
}
