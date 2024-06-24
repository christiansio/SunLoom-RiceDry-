#define BLYNK_TEMPLATE_ID "TMPL6TfBF6Gpl"
#define BLYNK_TEMPLATE_NAME "SunLoom RiceDry 2"
#define BLYNK_AUTH_TOKEN "7vXKOqDpgaN1AMvs2MpkrOEMJicqQMs0"
#define BLYNK_PRINT Serial

#include <WiFiS3.h>
#include <BlynkSimpleWifi.h>

char auth[] = BLYNK_AUTH_TOKEN; // Auth code sent via Email
char ssid[] = ""; // WiFi name
char pass[] = ""; // WiFi Password

#include <AccelStepper.h>
#include <Servo.h>

//pin not finalized
#define startButton 8
#define rButton1 5
#define rButton2 4
#define cButton1 7
#define cButton2 6
#define moistureSensor A4
#define rain A1
#define tempSensor A3
#define motionSensor A2

#define fanRelay 3
#define heaterRelay 2

#define pul1 13
#define dir1 12
AccelStepper stepper1(1, pul1, dir1);
const int speed1 = 950;

#define pul2 11
#define dir2 10
AccelStepper stepper2(1, pul2, dir2);
const int speed2 = 500;

static const int servoPin = A0;
int posDegree = 75;

int motionOut = 0; // motion sensor parameter

// initialize start times
unsigned long startTimeMoist = 0;
unsigned long startTimeMotion = 0;

// initialize durations
const unsigned long durationMotion = 5000;

// initialize temp variables
float sensorVal = 0;
float mv = 0;
float cel = 0;
int countTemp = 0;
float addedCel = 0;
float aveCel = 0;
float outputCel = 0;

//initialize Moisture Sensor variables
int msValue = 0;
int msAdded = 0;
int countMs = 0;
float calFormula = 0;
float msOutput = 0;
float aveMs = 0;



//initialize prevTime Millis
unsigned long prevTimeStep = millis();
unsigned long prevTimeRain = millis();
unsigned long prevTimeTemp = millis();
unsigned long prevTimeMoist = millis();
unsigned long prevTimeMoist1 = millis();
unsigned long prevTimeMoist2 = millis();
unsigned long prevTimeMotion = millis();
unsigned long prevTimeRButtonDeb = millis();



//initialize intervals
const long intervalButton = 1000;
const long intervalRain = 1000;
const long intervalTemp = 1000;  // Interval in milliseconds (1000 ms = 1 second)
const long intervalMoist = 1000;
const long intervalMoist1 = 10000;
const long intervalMoist2 = 1000;
const long intervalMotion = 1000;
const long intervalRButtonDeb = 1000;

// initialize rake button debounce variables (indicators)
const unsigned long debounceInterval = 50;
bool buttonState1 = LOW;
bool lastButtonState1 = LOW;
unsigned long lastDebounceTime1 = 0;
bool buttonState2 = LOW;
bool lastButtonState2 = LOW;
unsigned long lastDebounceTime2 = 0;

int rakeCount = 0; // count of rake movement

bool rakeTimerActive = false; // time when the rake will move

bool motionTimerActive = false; // State where the motion sensor will detect motion until some time to execute rake movement

// (indicators) States that change when the rake makes contact with the buttons: start and end
bool rButtonE = false;
bool rButtonS = false;

// Rake movement states depending on what sensor reaches the set threshold
bool rakeBasePos = true;
bool moveRakeAtMoist = false;
bool moveRakeAtMotion = false;
bool moveRakeAtStartup = false;


Servo servo1;

bool servoTimerActive = false;

bool forward = false;
bool backward = false;
bool stop = false;

unsigned long prevTimeServo1 = millis();
const long servoDelay = 150;

//initialize states
bool start = false;
bool moistureMet = false; // State that changes depending on set threshold
bool raining = false;
bool tempMet = false;

bool startButtonPressed = false;

enum State1 { INITIAL1, SERVO_MOVED1, STEPPER_RUNNING1 };
enum State2 { INITIAL2, SERVO_MOVED2, STEPPER_RUNNING2 };
State1 currentState1 = INITIAL1;
State2 currentState2 = INITIAL2;



// function that reads Temp in set intervals and sets parameter
void readTemp(){
  unsigned long currentTime = millis();  // Get the current time in milliseconds
  sensorVal = analogRead(tempSensor);
  mv = (sensorVal/1024.0)*5000; 
  cel = mv/10;
  addedCel = addedCel + cel;
  countTemp++;

  if (countTemp == 10){
    aveCel = addedCel/10.0;
    countTemp = 0;
    addedCel = 0;
  }

   // Check if 1 second has passed
  if (currentTime - prevTimeTemp >= intervalTemp) {
    prevTimeTemp = currentTime;  // Save the current time
    outputCel = aveCel;
    Serial.print("Temperature value =");
    Serial.println(outputCel);  // Print the output value
    Serial.print("moisture sensor");
    Serial.println(calFormula);
    if (outputCel >= 45){
      tempMet = true;
    } else if (outputCel < 45){
      tempMet = false;
    }
  }
}

//function that reads grain moisture level and sets moisture met state if value is below 15.00
void readMoist(){
  unsigned long currentTime = millis();  // Get the current time in milliseconds

  msValue = analogRead(moistureSensor);
  msAdded += msValue;
  countMs++;

  if (countMs == 10) {
    msOutput = msAdded / 10.0;
    calFormula = ((-0.038436353 * msOutput) + 52.09050857) - 1.5;

    // Reset counters
    countMs = 0;
    msAdded = 0;
  }

  if (currentTime - prevTimeMoist1 >= intervalMoist1) {
    msOutput = calFormula;
    Serial.print("Moisture level =");
    Serial.println(msOutput);
    if (msOutput <= 15.00){
      moistureMet = true;
    } else if (msOutput > 15.00){
      moistureMet = false;
    }
    if (!moistureMet){
      moveRakeAtMoist = true;
      rakeBasePos = false;
      Blynk.virtualWrite(V0, "Raking");
    } else {
      moveRakeAtMoist = false;
      Blynk.virtualWrite(V0, "Not Raking");
    }
    prevTimeMoist1 = currentTime;

  }
   if (currentTime - prevTimeMoist2 >= intervalMoist2){
    Blynk.virtualWrite(V1, calFormula);
    if (moistureMet){
      Blynk.virtualWrite(V2, "Rice grains have the right moisture.");
      Blynk.logEvent("moisture_level_met", "Rice grains have reached the proper moisture content.");
    } else if (!moistureMet){
      Blynk.virtualWrite(V2, "Rice grains do not have the right moisture.");
    }
   }

  
}


// method that reads time in set intervals
void readRain(){
  unsigned long currentTime = millis();
  int rainVal = analogRead(rain);
  if (currentTime - prevTimeRain >= intervalRain){
    if (rainVal <= 700){
      raining = true;
    } else if (rainVal > 700){
      raining = false;
    }
    Serial.print("Rain Val =");
    Serial.println(rainVal);
     if (raining == true){
      Blynk.virtualWrite(V3, "It is raining");
    } else if (raining == false){
      Blynk.virtualWrite(V3, "It is not raining");
    }

    prevTimeRain = currentTime;
  }

}

void rakeButtons(){
  unsigned long currentTime = millis();

  int rButton1Val = digitalRead(rButton1);

  if (rButton1Val != lastButtonState1){

    lastDebounceTime1 = currentTime;
  }

  if ((currentTime - lastDebounceTime1) >= debounceInterval){

    if (rButton1Val != buttonState1){
      buttonState1 = rButton1Val;

      if (buttonState1 == HIGH){
        rButtonS = true;
        rButtonE = false;
        rakeCount++;
        Serial.println("rButton1 is pressed");
        Serial.print("rButtonS = ");
        Serial.println(rButtonS);
        Serial.print("rButtonE = ");
        Serial.println(rButtonE);
        Serial.print("rake count = ");
        Serial.println(rakeCount);
      } else {
        Serial.println("rButton1 is released");
      }
    }
  }

  lastButtonState1 = rButton1Val;

  int rButton2Val = digitalRead(rButton2);

  if (rButton2Val != lastButtonState2){

    lastDebounceTime2 = currentTime;
  }

  if ((currentTime - lastDebounceTime2) >= debounceInterval){

    if (rButton2Val != buttonState2){
      buttonState2 = rButton2Val;

      if (buttonState2 == HIGH){
        rButtonE = true;
        rButtonS = false;
        Serial.println("rButton2 is pressed");
        Serial.print("rButtonS = ");
        Serial.println(rButtonS);
        Serial.print("rButtonE = ");
        Serial.println(rButtonE);
        Serial.print("rake count = ");
        Serial.println(rakeCount);

      } else {
        Serial.println("rButton2 is released");
      }
    }
  }

  lastButtonState2 = rButton2Val;
}

// Function that reads motion sensor and set moisture parameter depending on set interval
void readMotion(){
  unsigned long currentTime = millis();
  int motionVal = analogRead(motionSensor);
  motionOut = motionVal;
  //Serial.print("Motion Value = ");
 // Serial.println(motionOut);

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  WiFi.begin(ssid, pass);
  Blynk.config(auth);
  stepper1.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000); // set motor max speed

  servo1.attach(servoPin);


  pinMode(startButton, INPUT);
  pinMode(rButton1, INPUT);
  pinMode(rButton2, INPUT);
  pinMode(cButton1, INPUT);
  pinMode(cButton2, INPUT);
  pinMode(rain, INPUT);
  pinMode(motionSensor, INPUT);
  pinMode(moistureSensor, INPUT);

  pinMode(heaterRelay, OUTPUT);
  pinMode(fanRelay, OUTPUT);
  pinMode(tempSensor, INPUT);

  digitalWrite(heaterRelay, LOW);
  digitalWrite(fanRelay, LOW);

  
  int startButtonVal = digitalRead(startButton);
  while (!startButtonPressed) {
    int startButtonVal = digitalRead(startButton);
    int cButton1Val = digitalRead(cButton1);
    int cButton2Val = digitalRead(cButton2);
    readRain();
    stepper1.runSpeed();
  
   if (raining && !cButton2Val == 1){ // when it is raining
      stepper1.setSpeed(-speed1);
      stepper1.runSpeed(); // move ceiling forward
    } else {
      stepper1.setSpeed(0);
      stepper1.runSpeed(); // stop ceiling movement if in contact with cButton2
      Blynk.virtualWrite(V4, "Ceiling Closed");
    }

    if (!raining && cButton1Val == 0){ // when it is not raining
      stepper1.setSpeed(speed1);
      stepper1.runSpeed();   // move ceiling backward
    } else {
      stepper1.setSpeed(0);
      stepper1.runSpeed();  // stop ceiling movement if in contact with cButton1
      Blynk.virtualWrite(V4, "Ceiling Opened");
    }

    startButtonVal = digitalRead(startButton);  
    if (startButtonVal == 1){
      startButtonPressed = !startButtonPressed;
    }
    if (startButtonPressed){
      moveRakeAtStartup = true;
      rakeBasePos = false;
    } else {
      moveRakeAtStartup = false;
    }
    if (rakeCount == 10){
      rakeCount == 0;
    break;
    }

  }
}


void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();

   if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, pass);
    delay(1000); // Delay to avoid flooding connection attempts
    return;
  }

    // Check Blynk connection
  if (!Blynk.connected()) {
    Serial.println("Connecting to Blynk...");
    Blynk.connect(1000); // Try to connect to Blynk for 1 second
    return;
  }

   // Run Blynk
  Blynk.run();



  int startButtonVal = digitalRead(startButton);
  int cButton1Val = digitalRead(cButton1);
  int cButton2Val = digitalRead(cButton2);
  int rButton1Val = digitalRead(rButton1); 
  int rButton2Val = digitalRead(rButton2);

  readRain();
  readTemp();
  readMoist();

  readMotion(); // call motion reading function

  
  if (currentTime - prevTimeStep >= intervalButton){
    Serial.print("start = ");
    Serial.println(startButtonVal);
    Serial.print("cButton1 =");
    Serial.println(cButton1Val);
    Serial.print("cButton2 =");
    Serial.println(cButton2Val);
    prevTimeStep = currentTime;
  }


  if (moistureMet){

    digitalWrite(heaterRelay, HIGH); // turn off heater
    digitalWrite(fanRelay, HIGH);  // turn off fan

    if (moistureMet && !cButton2Val == 1){ // when moisture is below threshold
      stepper1.setSpeed(-speed1); 
      stepper1.runSpeed();  // move ceiling clockwise until it touches cButton2
      Blynk.virtualWrite(V4, "Ceiling Closed");
    } else {
      stepper1.setSpeed(0);
      stepper1.runSpeed();
       Blynk.virtualWrite(V4, "Ceiling Closed");
    }

  } else {

    if (raining && !cButton2Val == 1){ // when it is raining
      stepper1.setSpeed(-speed1);
      stepper1.runSpeed(); // move ceiling forward
    } else {
      stepper1.setSpeed(0);
      stepper1.runSpeed(); // stop ceiling movement if in contact with cButton2
      Blynk.virtualWrite(V4, "Ceiling Closed");
    }

    if (!raining && cButton1Val == 0){ // when it is not raining
      stepper1.setSpeed(speed1);
      stepper1.runSpeed();   // move ceiling backward
    } else {
      stepper1.setSpeed(0);
      stepper1.runSpeed();  // stop ceiling movement if in contact with cButton1
      Blynk.virtualWrite(V4, "Ceiling Opened");
    }

   
    if (!tempMet){
      digitalWrite(heaterRelay, LOW);
      digitalWrite(fanRelay, LOW);
    } else if (tempMet){ // when temp is higher than threshold
      digitalWrite(heaterRelay, HIGH); // turn heater off
    }

  }

  stepper2.runSpeed();
  servo1.write(posDegree);
  if (moveRakeAtStartup){
    moveRakeAtMoist = false;
    moveRakeAtMotion = false;
  }
  if (moveRakeAtMoist){
    moveRakeAtMotion = false;
  } 

  if (forward){
    stepper2.setSpeed(speed2);
  }

  if (backward){
    stepper2.setSpeed(-speed2);
  }
  if (stop){
    stepper2.setSpeed(0);
  }

  if (rakeBasePos){
    rButtonS = false;
    rButtonE = false;
    if (rakeBasePos && rButton1Val == 0){
      stepper2.setSpeed(-speed2);
    } else {
      stepper2.setSpeed(0);
      posDegree = 75;
    }

  }

  if (moveRakeAtStartup){
    rButtonS = true;
    rakeButtons();
    if (rakeCount == 10){
      moveRakeAtStartup = false;
      rakeBasePos = true;
      rakeCount = 0;
    }

     if (rButtonS == true ){
      unsigned long currentTime = millis();

      switch (currentState1){
        case INITIAL1:

        posDegree = 40;
        servo1.write(posDegree);
        stop = true;
        
       

        prevTimeServo1 = currentTime;

        currentState1 = SERVO_MOVED1;

        break;

        case SERVO_MOVED1:

        if (currentTime - prevTimeServo1 >= servoDelay){

          currentState1 = STEPPER_RUNNING1;
        }
        break;

        case STEPPER_RUNNING1:
          rakeBasePos = false;
          backward = false;
          forward = true;
          stepper2.setSpeed(speed2);
          if (rButton1Val == HIGH ) { 
            forward = false;
            currentState1 = INITIAL1; // Reset state for next cycle
            currentState2 = INITIAL2;
          } 

        break;
      }
    } 

    if (rButtonE == true ){
      unsigned long currentTime = millis();

      switch (currentState2){
        case INITIAL2:

        posDegree = 100;
        servo1.write(posDegree);
        stop = true;

        prevTimeServo1 = currentTime;

        currentState2 = SERVO_MOVED2;
        
        break;

        case SERVO_MOVED2:
        
          

        if (currentTime - prevTimeServo1 >= servoDelay){
          currentState2 = STEPPER_RUNNING2;
        }
        break;

        case STEPPER_RUNNING2:
          
          stop = false;
          rakeBasePos = false;
          forward = false;
          backward = true;
          stepper2.setSpeed(-speed2);
          if (rButton2Val == HIGH ) { // Assuming these buttons indicate the end of movement
            backward = false;
            currentState2 = INITIAL2; // Reset state for next cycle
            currentState1 = INITIAL1;
          }
        break;
      }
    } 
  }

  if (moveRakeAtMoist){
    rButtonS = true;
    rakeButtons();
    if (rakeCount == 5){
      moveRakeAtMoist = false;
      rakeBasePos = true;
      rakeCount = 0;
    }

    if (rButtonS == true){
      unsigned long currentTime = millis();

      switch (currentState1){
        case INITIAL1:

        posDegree = 40;

        servo1.write(posDegree);
        
                  stepper2.setSpeed(0);

        prevTimeServo1 = currentTime;

        currentState1 = SERVO_MOVED1;

        break;

        case SERVO_MOVED1:

        if (currentTime - prevTimeServo1 >= servoDelay){

          currentState1 = STEPPER_RUNNING1;
        }
        break;

        case STEPPER_RUNNING1:
          backward = false;
          forward = true;
          stepper2.setSpeed(speed2);
          if (rButton1Val == HIGH ) { 
            forward = false;
            currentState1 = INITIAL1; // Reset state for next cycle
            currentState2 = INITIAL2;
          }
        break;
      }
    } 

    if (rButtonE == true){
      unsigned long currentTime = millis();

      switch (currentState2){
        case INITIAL2:

        posDegree = 100;
        servo1.write(posDegree);
        stepper2.setSpeed(0);

        prevTimeServo1 = currentTime;

        currentState2 = SERVO_MOVED2;
        break;

        case SERVO_MOVED2:

        if (currentTime - prevTimeServo1 >= servoDelay){

          currentState2 = STEPPER_RUNNING2;
        }
        break;

        case STEPPER_RUNNING2:
          forward = false;
          backward = true;
          stepper2.setSpeed(-speed2);
          if (rButton2Val == HIGH ) { // Assuming these buttons indicate the end of movement
            backward = false;
            currentState2 = INITIAL2; // Reset state for next cycle
            currentState1 = INITIAL1;
          }
        break;
        


      }
    } 
  }

  if (moveRakeAtMotion){
    rButtonS = true;
    rakeButtons();
    if (rakeCount == 1){
      moveRakeAtMotion = false;
      rakeBasePos = true;
      rakeCount = 0;
    }
    if (rButtonS == true){
      stepper2.setSpeed(speed2);
    } 
    if (rButtonE == true){
      stepper2.setSpeed(-speed2);
    } 
  }
  
  

  if (motionOut >= 500 && !moveRakeAtMoist){ // when motionOut is equals to 1 and moveRakeAtMoist is false, execute
    if (!motionTimerActive){  // start timer if motionOut is continually 1
      startTimeMotion = millis();
      motionTimerActive = true; 
    } else {
      if (millis() - startTimeMotion >= durationMotion){ // when timer duration is met set moveRakeAtMotion to true (move rake)
          rakeBasePos = false;
          moveRakeAtMotion = true;
      }
    }
  } else {
    motionTimerActive = false;  // when motionOut is 1, set motionTimerActive to false
  }
  
  
}



  


